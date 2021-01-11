package us.ihmc.footstepPlanning.ui.controllers;

import controller_msgs.msg.dds.*;
import javafx.animation.AnimationTimer;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.communication.UserInterfaceIKMode;
import us.ihmc.footstepPlanning.ui.UIAuxiliaryRobotData;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.kinematics.DdoglegInverseKinematicsCalculator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class RobotIKUI
{
   @FXML private ToggleButton enableIK;
   @FXML private ComboBox<UserInterfaceIKMode> ikMode;
   @FXML private Slider xIKSlider;
   @FXML private Slider yIKSlider;
   @FXML private Slider zIKSlider;
   @FXML private Slider yawIKSlider;
   @FXML private Slider pitchIKSlider;
   @FXML private Slider rollIKSlider;

   private FullHumanoidRobotModel realRobotModel;
   private FullHumanoidRobotModel ikWorkRobotModel;

   private JavaFXMessager messager;
   private UIAuxiliaryRobotData auxiliaryRobotData;
   private static final double minimumTrajectoryDuration = 0.5;
   private static final double defaultMaxRadianPerSecond = 5.0;
   private final AnimationTimer ikAnimationTimer;
   private final Map<UserInterfaceIKMode, DdoglegInverseKinematicsCalculator> ikSolvers = new HashMap<>();
   private final Map<UserInterfaceIKMode, GeometricJacobian> limbJacobians = new HashMap<>();

   private final AtomicBoolean initializeIKFlag = new AtomicBoolean();
   private final AtomicBoolean ikPositionSliderUpdatedFlag = new AtomicBoolean();
   private final AtomicBoolean ikOrientationSliderUpdatedFlag = new AtomicBoolean();
   private final AtomicReference<UserInterfaceIKMode> currentIKMode = new AtomicReference<>();
   private final FramePose3D initialIKPose = new FramePose3D();
   private final FramePose3D targetIKPose = new FramePose3D();
   private final FramePose3D handControlFramePose = new FramePose3D();
   private final RigidBodyTransform targetIKTransform = new RigidBodyTransform();

   private final AtomicReference<double[]> latestIKSolution = new AtomicReference<>();

   public RobotIKUI()
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

            UserInterfaceIKMode selectedMode = currentIKMode.get();
            boolean includePosition = selectedMode.isArmMode() || selectedMode.isLegMode();

            if (!ikOrientationSliderUpdatedFlag.getAndSet(false) && !(includePosition && ikPositionSliderUpdatedFlag.getAndSet(false)))
            {
               return;
            }

            targetIKPose.setIncludingFrame(initialIKPose);
            if (includePosition)
            {
               targetIKPose.getPosition().add(xIKSlider.getValue(), yIKSlider.getValue(), zIKSlider.getValue());
            }

            targetIKPose.getOrientation().prependYawRotation(yawIKSlider.getValue());
            targetIKPose.getOrientation().prependPitchRotation(pitchIKSlider.getValue());
            targetIKPose.getOrientation().prependRollRotation(rollIKSlider.getValue());

            DdoglegInverseKinematicsCalculator ikSolver = ikSolvers.get(selectedMode);
            targetIKTransform.set(targetIKPose.getOrientation(), targetIKPose.getPosition());
            boolean success = ikSolver.solve(targetIKTransform);

            if (!success)
            {
               return;
            }

            GeometricJacobian limbJacobian = limbJacobians.get(selectedMode);
            double[] solutionJointAngles = new double[limbJacobian.getNumberOfColumns()];

            for (int i = 0; i < solutionJointAngles.length; i++)
            {
               solutionJointAngles[i] = ((OneDoFJoint) limbJacobian.getJointsInOrder()[i]).getQ();
            }

            latestIKSolution.set(solutionJointAngles);
            messager.submitMessage(FootstepPlannerMessagerAPI.IKSolution, solutionJointAngles);
         }
      };
   }

   public void setFullRobotModel(FullHumanoidRobotModel realRobotModel, FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory)
   {
      this.realRobotModel = realRobotModel;
      this.ikWorkRobotModel = fullHumanoidRobotModelFactory.createFullRobotModel();

      for (RobotSide robotSide : RobotSide.values)
      {
         UserInterfaceIKMode armIKMode = robotSide == RobotSide.LEFT ? UserInterfaceIKMode.LEFT_ARM : UserInterfaceIKMode.RIGHT_ARM;
         UserInterfaceIKMode legIKMode = robotSide == RobotSide.LEFT ? UserInterfaceIKMode.LEFT_LEG : UserInterfaceIKMode.RIGHT_LEG;

         double positionCost = 1.0;
         double orientationCost = 0.2;
         int maxIterations = 500;
         boolean solveOrientation = true;
         double convergenceTolerance = 1.0e-4;
         double positionTolerance = 0.005;
         double angleTolerance = Math.toRadians(4.0);
         double parameterChangePenalty = 0.1;

         GeometricJacobian armJacobian = new GeometricJacobian(ikWorkRobotModel.getChest(),
                                                               ikWorkRobotModel.getHand(robotSide),
                                                               ikWorkRobotModel.getHand(robotSide).getBodyFixedFrame());
         limbJacobians.put(armIKMode, armJacobian);
         ikSolvers.put(armIKMode,
                       new DdoglegInverseKinematicsCalculator(armJacobian,
                                                              positionCost,
                                                              orientationCost,
                                                              maxIterations,
                                                              solveOrientation,
                                                              convergenceTolerance,
                                                              positionTolerance,
                                                              angleTolerance,
                                                              parameterChangePenalty));

         GeometricJacobian legJacobian = new GeometricJacobian(ikWorkRobotModel.getPelvis(),
                                                               ikWorkRobotModel.getFoot(robotSide),
                                                               ikWorkRobotModel.getFoot(robotSide).getBodyFixedFrame());
         limbJacobians.put(legIKMode, legJacobian);
         ikSolvers.put(legIKMode,
                       new DdoglegInverseKinematicsCalculator(legJacobian,
                                                              positionCost,
                                                              orientationCost,
                                                              maxIterations,
                                                              solveOrientation,
                                                              convergenceTolerance,
                                                              positionTolerance,
                                                              angleTolerance,
                                                              parameterChangePenalty));
      }

      double positionCost = 0.05;
      double orientationCost = 1.0;
      int maxIterations = 500;
      boolean solveOrientation = true;
      double convergenceTolerance = 1.0e-4;
      double positionTolerance = Double.MAX_VALUE;
      double angleTolerance = 0.02;
      double parameterChangePenalty = 0.1;

      GeometricJacobian neckJacobian = new GeometricJacobian(ikWorkRobotModel.getChest(), ikWorkRobotModel.getHead(), ikWorkRobotModel.getHead().getBodyFixedFrame());
      limbJacobians.put(UserInterfaceIKMode.NECK, neckJacobian);
      ikSolvers.put(UserInterfaceIKMode.NECK,
                    new DdoglegInverseKinematicsCalculator(neckJacobian,
                                                           positionCost,
                                                           orientationCost,
                                                           maxIterations,
                                                           solveOrientation,
                                                           convergenceTolerance,
                                                           positionTolerance,
                                                           angleTolerance,
                                                           parameterChangePenalty));

      GeometricJacobian chestJacobian = new GeometricJacobian(ikWorkRobotModel.getPelvis(),
                                                              ikWorkRobotModel.getChest(),
                                                              ikWorkRobotModel.getChest().getBodyFixedFrame());
      limbJacobians.put(UserInterfaceIKMode.CHEST, chestJacobian);
      ikSolvers.put(UserInterfaceIKMode.CHEST,
                    new DdoglegInverseKinematicsCalculator(chestJacobian,
                                                           positionCost,
                                                           orientationCost,
                                                           maxIterations,
                                                           solveOrientation,
                                                           convergenceTolerance,
                                                           positionTolerance,
                                                           angleTolerance,
                                                           parameterChangePenalty));
   }

   /** Called by JavaFX via reflection */
   public void initialize()
   {
      ikMode.setItems(FXCollections.observableArrayList(UserInterfaceIKMode.values()));
      ikMode.setValue(UserInterfaceIKMode.LEFT_ARM);
      ikMode.getSelectionModel().selectedItemProperty().addListener((observable, oldValue, newValue) -> initializeIKFlag.set(true));

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

      xIKSlider.valueProperty().addListener(observable -> ikPositionSliderUpdatedFlag.set(true));
      yIKSlider.valueProperty().addListener(observable -> ikPositionSliderUpdatedFlag.set(true));
      zIKSlider.valueProperty().addListener(observable -> ikPositionSliderUpdatedFlag.set(true));
      yawIKSlider.valueProperty().addListener(observable -> ikOrientationSliderUpdatedFlag.set(true));
      pitchIKSlider.valueProperty().addListener(observable -> ikOrientationSliderUpdatedFlag.set(true));
      rollIKSlider.valueProperty().addListener(observable -> ikOrientationSliderUpdatedFlag.set(true));
   }

   private void initializeIK()
   {
      resetIKOffset();

      ikPositionSliderUpdatedFlag.set(false);
      ikOrientationSliderUpdatedFlag.set(false);

      UserInterfaceIKMode selectedIKMode = ikMode.getValue();
      GeometricJacobian limbJacobian = limbJacobians.get(selectedIKMode);

      initialIKPose.setToZero(limbJacobian.getJacobianFrame());
      ReferenceFrame baseFrame = limbJacobian.getBase().getBodyFixedFrame();
      initialIKPose.changeFrame(baseFrame);

      currentIKMode.set(selectedIKMode);
      messager.submitMessage(FootstepPlannerMessagerAPI.IKEnabled, true);
      messager.submitMessage(FootstepPlannerMessagerAPI.SelectedIKMode, selectedIKMode);
   }

   @FXML
   public void resetIKOffset()
   {
      xIKSlider.setValue(0.0);
      yIKSlider.setValue(0.0);
      zIKSlider.setValue(0.0);
      yawIKSlider.setValue(0.0);
      pitchIKSlider.setValue(0.0);
      rollIKSlider.setValue(0.0);
      copyRobotState(realRobotModel, ikWorkRobotModel);
   }

   @FXML
   public void sendJointspaceTrajectory()
   {
      UserInterfaceIKMode currentIKMode = this.currentIKMode.get();
      double[] latestIKSolution = this.latestIKSolution.get();

      if (latestIKSolution == null)
      {
         return;
      }

      double trajectoryDuration = computeTrajectoryDuration(currentIKMode, latestIKSolution);
      switch (currentIKMode)
      {
         case LEFT_ARM:
         case RIGHT_ARM:
         {
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(currentIKMode.getSide(),
                                                                                                        trajectoryDuration,
                                                                                                        latestIKSolution);
            messager.submitMessage(FootstepPlannerMessagerAPI.ArmTrajectoryMessageTopic, armTrajectoryMessage);
            break;
         }
         case NECK:
         {
            NeckTrajectoryMessage neckTrajectoryMessage = HumanoidMessageTools.createNeckTrajectoryMessage(trajectoryDuration, latestIKSolution);
            messager.submitMessage(FootstepPlannerMessagerAPI.NeckTrajectoryMessageTopic, neckTrajectoryMessage);
            break;
         }
         case CHEST:
         {
            SpineTrajectoryMessage spineTrajectoryMessage = HumanoidMessageTools.createSpineTrajectoryMessage(trajectoryDuration, latestIKSolution);
            messager.submitMessage(FootstepPlannerMessagerAPI.SpineTrajectoryMessageTopic, spineTrajectoryMessage);
         }
         case LEFT_LEG:
         case RIGHT_LEG:
         {
            // TODO message non-existant. maybe add a jointspace "LegTrajectoryMessage"?
         }
      }
   }

   @FXML
   public void sendTaskspaceTrajectory()
   {
      UserInterfaceIKMode currentIKMode = this.currentIKMode.get();
      double[] latestIKSolution = this.latestIKSolution.get();

      if (latestIKSolution == null)
      {
         return;
      }

      double trajectoryDuration = computeTrajectoryDuration(currentIKMode, latestIKSolution);
      switch (currentIKMode)
      {
         case LEFT_ARM:
         case RIGHT_ARM:
         {
            handControlFramePose.setToZero(ikWorkRobotModel.getHandControlFrame(currentIKMode.getSide()));
            handControlFramePose.changeFrame(ikWorkRobotModel.getChest().getBodyFixedFrame());
            HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(currentIKMode.getSide(),
                                                                                                           trajectoryDuration,
                                                                                                           handControlFramePose,
                                                                                                           handControlFramePose.getReferenceFrame());
            messager.submitMessage(FootstepPlannerMessagerAPI.HandTrajectoryMessageTopic, handTrajectoryMessage);
            break;
         }
         case NECK:
         {
            HeadTrajectoryMessage headTrajectoryMessage = HumanoidMessageTools.createHeadTrajectoryMessage(trajectoryDuration,
                                                                                                           targetIKPose.getOrientation(),
                                                                                                           targetIKPose.getReferenceFrame());
            messager.submitMessage(FootstepPlannerMessagerAPI.HeadTrajectoryMessageTopic, headTrajectoryMessage);
            break;
         }
         case CHEST:
         {
            ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryDuration,
                                                                                                              targetIKPose.getOrientation(),
                                                                                                              targetIKPose.getReferenceFrame());
            messager.submitMessage(FootstepPlannerMessagerAPI.ChestTrajectoryMessageTopic, chestTrajectoryMessage);
         }
         case LEFT_LEG:
         case RIGHT_LEG:
         {
            ReferenceFrame originalFrame = targetIKPose.getReferenceFrame();
            targetIKPose.changeFrame(ReferenceFrame.getWorldFrame());
            FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(currentIKMode.getSide(),
                                                                                                           trajectoryDuration,
                                                                                                           targetIKPose);
            messager.submitMessage(FootstepPlannerMessagerAPI.FootTrajectoryMessageTopic, footTrajectoryMessage);
            targetIKPose.changeFrame(originalFrame);
         }
      }
   }

   private double computeTrajectoryDuration(UserInterfaceIKMode currentIKMode, double[] latestIKSolution)
   {
      JointBasics[] limbJoints = limbJacobians.get(currentIKMode).getJointsInOrder();
      if (limbJoints.length != latestIKSolution.length)
      {
         return 3.0;
      }

      double maximumJointDelta = 0.0;
      for (int i = 0; i < latestIKSolution.length; i++)
      {
         double qActual = realRobotModel.getOneDoFJointByName(limbJoints[i].getName()).getQ();
         double qDesired = latestIKSolution[i];
         double qDelta = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(qActual, qDesired));
         if (qDelta > maximumJointDelta)
         {
            maximumJointDelta = qDelta;
         }
      }

      return Math.max(maximumJointDelta * defaultMaxRadianPerSecond, minimumTrajectoryDuration);
   }

   private static void copyRobotState(FullHumanoidRobotModel source, FullHumanoidRobotModel destination)
   {
      OneDoFJointBasics[] sourceJoints = source.getOneDoFJoints();
      OneDoFJointBasics[] destinationJoints = destination.getOneDoFJoints();
      for (int i = 0; i < sourceJoints.length; i++)
      {
         destinationJoints[i].setQ(sourceJoints[i].getQ());
      }

      destination.getRootJoint().getJointPose().set(source.getRootJoint().getJointPose());
      destination.updateFrames();
   }

   @FXML
   public void sendLeftArmIn()
   {
      if (auxiliaryRobotData != null)
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.RequestedArmJointAngles,
                                Pair.of(RobotSide.LEFT, auxiliaryRobotData.getArmsInJointAngles().get(RobotSide.LEFT)));
      }
   }

   @FXML
   public void sendRightArmIn()
   {
      if (auxiliaryRobotData != null)
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.RequestedArmJointAngles,
                                Pair.of(RobotSide.RIGHT, auxiliaryRobotData.getArmsInJointAngles().get(RobotSide.RIGHT)));
      }
   }

   public void setAuxiliaryRobotData(UIAuxiliaryRobotData auxiliaryRobotData)
   {
      this.auxiliaryRobotData = auxiliaryRobotData;
   }
}
