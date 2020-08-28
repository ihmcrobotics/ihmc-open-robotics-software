package us.ihmc.footstepPlanning.ui.controllers;

import controller_msgs.msg.dds.BipedalSupportPlanarRegionParametersMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import javafx.animation.AnimationTimer;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.communication.UserInterfaceIKMode;
import us.ihmc.footstepPlanning.ui.UIAuxiliaryRobotData;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.kinematics.DdoglegInverseKinematicsCalculator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class RobotOperationTabController
{
   private JavaFXMessager messager;
   private RobotLowLevelMessenger robotLowLevelMessenger;
   private UIAuxiliaryRobotData auxiliaryRobotData;
   private FullHumanoidRobotModel realRobotModel;
   private FullHumanoidRobotModel workRobotModel;

   @FXML
   private Button homeAll;
   @FXML
   private Button freeze;
   @FXML
   private Button standPrep;
   @FXML
   private Button shutdown;
   @FXML
   private Button abortWalking;
   @FXML
   private Button pauseWalking;
   @FXML
   private Button continueWalking;

   @FXML
   private CheckBox enableSupportRegions;
   @FXML
   private Spinner<Double> supportRegionScale;
   private IHMCRealtimeROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;

   @FXML
   private ToggleButton enableIK;
   @FXML
   private Button resetIK;
   @FXML
   private ComboBox<UserInterfaceIKMode> ikMode;
   @FXML
   private Slider xIKSlider;
   @FXML
   private Slider yIKSlider;
   @FXML
   private Slider zIKSlider;
   @FXML
   private Slider yawIKSlider;
   @FXML
   private Slider pitchIKSlider;
   @FXML
   private Slider rollIKSlider;

   private final AnimationTimer ikAnimationTimer;

   private final SideDependentList<DdoglegInverseKinematicsCalculator> armIKSolvers = new SideDependentList<>();
   private final SideDependentList<GeometricJacobian> armJacobians = new SideDependentList<>();

   private DdoglegInverseKinematicsCalculator neckIKSolver;
   private GeometricJacobian neckJacobian;

   private final AtomicBoolean initializeIKFlag = new AtomicBoolean();
   private final AtomicReference<UserInterfaceIKMode> currentMode = new AtomicReference<>();
   private final FramePose3D initialPose = new FramePose3D();
   private final FramePose3D targetPose = new FramePose3D();
   private final RigidBodyTransform targetTransform = new RigidBodyTransform();

   public RobotOperationTabController()
   {
      ikAnimationTimer = new AnimationTimer()
      {
         @Override
         public void handle(long l)
         {
            if (realRobotModel == null || !enableIK.isSelected())
            {
               return;
            }

            if (initializeIKFlag.getAndSet(false))
            {
               try
               {
                  initializeIK();
               }
               catch (Exception e)
               {
                  e.printStackTrace();
                  initializeIKFlag.set(true);
                  return;
               }
            }

            UserInterfaceIKMode selectedMode = currentMode.get();

            targetPose.setIncludingFrame(initialPose);

            if (selectedMode.isArmMode())
            {
               targetPose.getPosition().add(xIKSlider.getValue(), yIKSlider.getValue(), zIKSlider.getValue());
            }

            targetPose.getOrientation().prependYawRotation(yawIKSlider.getValue());
            targetPose.getOrientation().prependPitchRotation(pitchIKSlider.getValue());
            targetPose.getOrientation().prependRollRotation(rollIKSlider.getValue());


            if (selectedMode.isArmMode())
            {
               RobotSide selectedSide = selectedMode.getSide();
               DdoglegInverseKinematicsCalculator ikSolver = armIKSolvers.get(selectedSide);
               targetTransform.set(targetPose.getOrientation(), targetPose.getPosition());
               boolean success = ikSolver.solve(targetTransform);

               GeometricJacobian armJacobian = armJacobians.get(selectedSide);
               double[] solutionJointAngles = new double[armJacobian.getNumberOfColumns()];
               for (int i = 0; i < solutionJointAngles.length; i++)
               {
                  solutionJointAngles[i] = ((OneDoFJoint) armJacobian.getJointsInOrder()[i]).getQ();
               }

               messager.submitMessage(FootstepPlannerMessagerAPI.IKSolution, solutionJointAngles);
            }
            else if (selectedMode == UserInterfaceIKMode.NECK)
            {
               targetTransform.set(targetPose.getOrientation(), targetPose.getPosition());
               boolean success = neckIKSolver.solve(targetTransform);

               double[] solutionJointAngles = new double[neckJacobian.getNumberOfColumns()];
               for (int i = 0; i < solutionJointAngles.length; i++)
               {
                  solutionJointAngles[i] = ((OneDoFJoint) neckJacobian.getJointsInOrder()[i]).getQ();
               }

               messager.submitMessage(FootstepPlannerMessagerAPI.IKSolution, solutionJointAngles);
            }
         }
      };
   }

   public void initialize()
   {
      updateButtons();
      supportRegionScale.setValueFactory(new DoubleSpinnerValueFactory(0.0, 10.0, 2.0, 0.1));

      ikMode.setItems(FXCollections.observableArrayList(UserInterfaceIKMode.values()));
      ikMode.setValue(UserInterfaceIKMode.LEFT_ARM);
      ikMode.itemsProperty().addListener((observable, oldValue, newValue) -> initializeIKFlag.set(true));

      enableIK.selectedProperty().addListener((observable, oldValue, newValue) ->
                                              {
                                                 if (newValue)
                                                 {
                                                    initializeIKFlag.set(true);
                                                    ikAnimationTimer.start();
                                                 }
                                                 else
                                                 {
                                                    messager.submitMessage(FootstepPlannerMessagerAPI.IKEnabled, false);
                                                    ikAnimationTimer.stop();
                                                 }
                                              });
   }

   private void updateButtons()
   {
      homeAll.setDisable(messager == null);
      freeze.setDisable(robotLowLevelMessenger == null);
      standPrep.setDisable(robotLowLevelMessenger == null);
      shutdown.setDisable(robotLowLevelMessenger == null);
      abortWalking.setDisable(robotLowLevelMessenger == null);
      pauseWalking.setDisable(robotLowLevelMessenger == null);
   }

   @FXML
   public void homeAll()
   {
      GoHomeMessage homeLeftArm = new GoHomeMessage();
      homeLeftArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeLeftArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_LEFT);
      messager.submitMessage(FootstepPlannerMessagerAPI.GoHomeTopic, homeLeftArm);

      GoHomeMessage homeRightArm = new GoHomeMessage();
      homeRightArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
      homeRightArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_RIGHT);
      messager.submitMessage(FootstepPlannerMessagerAPI.GoHomeTopic, homeRightArm);
   }

   public void setFullRobotModel(FullHumanoidRobotModel realRobotModel, FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory)
   {
      this.realRobotModel = realRobotModel;
      this.workRobotModel = fullHumanoidRobotModelFactory.createFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         GeometricJacobian armJacobian = new GeometricJacobian(workRobotModel.getChest(),
                                                            workRobotModel.getHand(robotSide),
                                                            workRobotModel.getHand(robotSide).getBodyFixedFrame());
         armJacobians.put(robotSide, armJacobian);

         double positionCost = 1.0;
         double orientationCost = 0.2;
         int maxIterations = 500;
         boolean solveOrientation = true;
         double convergenceTolerance = 4.0e-6;
         double positionTolerance = 0.005;
         double angleTolerance = 0.02;
         double parameterChangePenalty = 0.1;

         armIKSolvers.put(robotSide,
                          new DdoglegInverseKinematicsCalculator(armJacobian,
                                                                  positionCost,
                                                                  orientationCost,
                                                                  maxIterations,
                                                                  solveOrientation,
                                                                  convergenceTolerance,
                                                                  positionTolerance,
                                                                  angleTolerance,
                                                                  parameterChangePenalty));
      }

      this.neckJacobian = new GeometricJacobian(workRobotModel.getChest(), workRobotModel.getHead(), workRobotModel.getHead().getBodyFixedFrame());

      double positionCost = 0.05;
      double orientationCost = 1.0;
      int maxIterations = 200;
      boolean solveOrientation = true;
      double convergenceTolerance = 5.0e-5;
      double positionTolerance = Double.MAX_VALUE;
      double angleTolerance = 0.02;
      double parameterChangePenalty = 0.1;
      neckIKSolver = new DdoglegInverseKinematicsCalculator(neckJacobian,
                                                            positionCost,
                                                            orientationCost,
                                                            maxIterations,
                                                            solveOrientation,
                                                            convergenceTolerance,
                                                            positionTolerance,
                                                            angleTolerance,
                                                            parameterChangePenalty);
   }

   private void initializeIK()
   {
      xIKSlider.setValue(0.0);
      yIKSlider.setValue(0.0);
      zIKSlider.setValue(0.0);
      yawIKSlider.setValue(0.0);
      pitchIKSlider.setValue(0.0);
      rollIKSlider.setValue(0.0);

      UserInterfaceIKMode selectedMode = ikMode.getValue();
      if (selectedMode.isArmMode())
      {
         RobotSide selectedSide = selectedMode.getSide();
         initialPose.setToZero(realRobotModel.getHand(selectedSide).getBodyFixedFrame());
         initialPose.changeFrame(realRobotModel.getChest().getBodyFixedFrame());
      }

      currentMode.set(selectedMode);
      messager.submitMessage(FootstepPlannerMessagerAPI.SelectedIKMode, selectedMode);
      messager.submitMessage(FootstepPlannerMessagerAPI.IKEnabled, true);
   }

   @FXML
   public void freeze()
   {
      robotLowLevelMessenger.sendFreezeRequest();
   }

   @FXML
   public void standPrep()
   {
      robotLowLevelMessenger.sendStandRequest();
   }

   @FXML
   public void shutdown()
   {
      robotLowLevelMessenger.sendShutdownRequest();
   }

   @FXML
   public void abortWalking()
   {
      robotLowLevelMessenger.sendAbortWalkingRequest();
   }

   @FXML
   public void pauseWalking()
   {
      robotLowLevelMessenger.sendPauseWalkingRequest();
   }

   @FXML
   public void continueWalking()
   {
      robotLowLevelMessenger.sendContinueWalkingRequest();
   }

   @FXML
   public void sendSupportRegionParameters()
   {
      BipedalSupportPlanarRegionParametersMessage supportPlanarRegionParametersMessage = new BipedalSupportPlanarRegionParametersMessage();
      supportPlanarRegionParametersMessage.setEnable(enableSupportRegions.isSelected());
      supportPlanarRegionParametersMessage.setSupportRegionScaleFactor(supportRegionScale.getValue());
      messager.submitMessage(FootstepPlannerMessagerAPI.BipedalSupportRegionsParameters, supportPlanarRegionParametersMessage);
   }

   @FXML public void clearREA()
   {
      REAStateRequestMessage clearMessage = new REAStateRequestMessage();
      clearMessage.setRequestClear(true);
      reaStateRequestPublisher.publish(clearMessage);
   }

   @FXML
   public void sendLeftArmIn()
   {
      if (auxiliaryRobotData != null)
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.RequestedArmJointAngles, Pair.of(RobotSide.LEFT, auxiliaryRobotData.getArmsInJointAngles().get(RobotSide.LEFT)));
      }
   }

   @FXML
   public void sendRightArmIn()
   {
      if (auxiliaryRobotData != null)
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.RequestedArmJointAngles, Pair.of(RobotSide.RIGHT, auxiliaryRobotData.getArmsInJointAngles().get(RobotSide.RIGHT)));
      }
   }

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
      updateButtons();
   }

   public void setRobotLowLevelMessenger(RobotLowLevelMessenger robotLowLevelMessenger)
   {
      this.robotLowLevelMessenger = robotLowLevelMessenger;
      updateButtons();
   }

   public void setREAStateRequestPublisher(IHMCRealtimeROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher)
   {
      this.reaStateRequestPublisher = reaStateRequestPublisher;
   }

   public void setAuxiliaryRobotData(UIAuxiliaryRobotData auxiliaryRobotData)
   {
      this.auxiliaryRobotData = auxiliaryRobotData;
   }
}
