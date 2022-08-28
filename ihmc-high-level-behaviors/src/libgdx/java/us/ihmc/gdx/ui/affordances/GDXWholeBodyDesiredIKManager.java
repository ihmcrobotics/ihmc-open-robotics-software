package us.ihmc.gdx.ui.affordances;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import imgui.ImGui;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.teleoperation.GDXTeleoperationParameters;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.kinematics.DdoglegInverseKinematicsCalculator;
import us.ihmc.robotics.kinematics.InverseKinematicsCalculator;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.tools.thread.MissingThreadTools;

public class GDXWholeBodyDesiredIKManager
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobot;
   private final FullHumanoidRobotModel desiredRobot;
   private FullHumanoidRobotModel workingRobot;
   private final ROS2ControllerHelper ros2Helper;
   private final GDXTeleoperationParameters teleoperationParameters;

   private final ArmJointName[] armJointNames;
   private HandDataType handPoseDataTypeToSend = HandDataType.JOINT_ANGLES;

   // returned as output
   private final SideDependentList<GeometricJacobian> desiredArmJacobians = new SideDependentList<>();

   // passed to IK solvers
   private final SideDependentList<GeometricJacobian> workArmJacobians = new SideDependentList<>();
   private final SideDependentList<GeometricJacobian> actualArmJacobians = new SideDependentList<>();

   private static final int INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE = 5;
   private int maxIterations = 500;

   private final SideDependentList<FramePose3D> correctedDesiredHandControlFramePoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> lastCorrectedDesiredHandControlFramePoses = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<Boolean> ikFoundASolution = new SideDependentList<>();
   private final SideDependentList<InverseKinematicsCalculator> inverseKinematicsCalculators = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> controlToWristTransforms = new SideDependentList<>();
   private final ModifiableReferenceFrame temporaryFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
   private volatile boolean readyToSolve = true;
   private volatile boolean readyToCopySolution = false;

   private enum HandDataType
   {
      JOINT_ANGLES, POSE_WORLD, POSE_CHEST
   }

   public GDXWholeBodyDesiredIKManager(DRCRobotModel robotModel,
                                       ROS2SyncedRobotModel syncedRobot,
                                       FullHumanoidRobotModel desiredRobot,
                                       ROS2ControllerHelper ros2Helper,
                                       GDXTeleoperationParameters teleoperationParameters)
   {
      this.robotModel = robotModel;
      this.syncedRobot = syncedRobot;
      this.desiredRobot = desiredRobot;
      this.ros2Helper = ros2Helper;
      this.teleoperationParameters = teleoperationParameters;
      armJointNames = robotModel.getJointMap().getArmJointNames();
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      workingRobot = robotModel.createFullRobotModel();

      for (RobotSide side : RobotSide.values)
      {
         ikFoundASolution.put(side, false);

         desiredArmJacobians.put(side, new GeometricJacobian(desiredRobot.getChest(),
                                                             desiredRobot.getHand(side),
                                                             desiredRobot.getHand(side).getBodyFixedFrame()));
         actualArmJacobians.put(side, new GeometricJacobian(syncedRobot.getFullRobotModel().getChest(),
                                                            syncedRobot.getFullRobotModel().getHand(side),
                                                            syncedRobot.getFullRobotModel().getHand(side).getBodyFixedFrame()));
         workArmJacobians.put(side, new GeometricJacobian(workingRobot.getChest(),
                                                          workingRobot.getHand(side),
                                                          workingRobot.getHand(side).getBodyFixedFrame()));

         double convergeTolerance = 4.0e-6; //1e-12;
         double parameterChangePenalty = 0.1;
         double positionCost = 1.0;
         double orientationCost = 0.2;
         boolean solveOrientation = true;
         double toleranceForPositionError = 0.005;
         double toleranceForOrientationError = 0.02;
         inverseKinematicsCalculators.put(side, new DdoglegInverseKinematicsCalculator(workArmJacobians.get(side),
                                                                                       positionCost,
                                                                                       orientationCost,
                                                                                       maxIterations,
                                                                                       solveOrientation,
                                                                                       convergeTolerance,
                                                                                       toleranceForPositionError,
                                                                                       toleranceForOrientationError,
                                                                                       parameterChangePenalty));

         RigidBodyTransform wristToControlTransform = new RigidBodyTransform(robotModel.getJointMap().getHandControlFrameToWristTransform(side));
         wristToControlTransform.invert();
         controlToWristTransforms.put(side, wristToControlTransform);
      }
   }

   // TODO this update should be moved into the control ring, and should use the control ring pose.
   public void update(SideDependentList<GDXHandInteractable> handInteractables)
   {
      boolean desiredHandsChanged = false;
      for (RobotSide side : handInteractables.sides())
      {
         computeArmJacobians(side);

         FramePose3D correctedDesiredHandControlFramePose = correctedDesiredHandControlFramePoses.get(side);
         correctedDesiredHandControlFramePose.setToZero(handInteractables.get(side).getControlReferenceFrame());
         correctedDesiredHandControlFramePose.changeFrame(ReferenceFrame.getWorldFrame());
         correctedDesiredHandControlFramePose.get(temporaryFrame.getTransformToParent());
         temporaryFrame.getReferenceFrame().update();
         correctedDesiredHandControlFramePose.setToZero(temporaryFrame.getReferenceFrame());
         correctedDesiredHandControlFramePose.set(controlToWristTransforms.get(side));
         // IDK where these come from; I tuned using JRebel
         correctedDesiredHandControlFramePose.getTranslation().subZ(.045);
         correctedDesiredHandControlFramePose.getTranslation().subX(.007);
         correctedDesiredHandControlFramePose.getTranslation().subY(side.negateIfLeftSide(.0015));

         // TODO: Frame??
         correctedDesiredHandControlFramePose.changeFrame(desiredRobot.getChest().getBodyFixedFrame());

         // We only want to evaluate this when we are going to take action on it
         // Otherwise, we will not notice the desired changed while the solver was still solving
         if (readyToSolve)
         {
            desiredHandsChanged |= !lastCorrectedDesiredHandControlFramePoses.get(side).geometricallyEquals(correctedDesiredHandControlFramePose, 0.0001);
            lastCorrectedDesiredHandControlFramePoses.get(side).setIncludingFrame(correctedDesiredHandControlFramePose);
         }
      }

      // The following puts the solver on a thread as to not slow down the UI
      if (readyToSolve && desiredHandsChanged)
      {
         readyToSolve = false;
         for (RobotSide side : handInteractables.sides())
         {
            copyOneDofJoints(actualArmJacobians.get(side).getJointsInOrder(), workArmJacobians.get(side).getJointsInOrder());
         }

         MissingThreadTools.startAThread("IKSolver", DefaultExceptionHandler.MESSAGE_AND_STACKTRACE, () ->
         {
            for (RobotSide side : handInteractables.sides())
            {
               FramePose3D lastCorrectedDesiredHandControlFramePose = lastCorrectedDesiredHandControlFramePoses.get(side);

               workArmJacobians.get(side).compute();
               ikFoundASolution.put(side, false);
               for (int i = 0; i < INVERSE_KINEMATICS_CALCULATIONS_PER_UPDATE && !ikFoundASolution.get(side); i++)
               {
                  InverseKinematicsCalculator inverseKinematicsCalculator = inverseKinematicsCalculators.get(side);
                  boolean foundASolution = inverseKinematicsCalculator.solve(lastCorrectedDesiredHandControlFramePose);
                  ikFoundASolution.put(side, foundASolution);
               }
            }

            readyToCopySolution = true;
         });
      }
      if (readyToCopySolution)
      {
         readyToCopySolution = false;
         for (RobotSide side : handInteractables.sides())
         {
            copyOneDofJoints(workArmJacobians.get(side).getJointsInOrder(), desiredArmJacobians.get(side).getJointsInOrder());
         }

         readyToSolve = true;
      }

      desiredRobot.getRootJoint().setJointConfiguration(syncedRobot.getFullRobotModel().getRootJoint().getJointPose());

      // TODO Update the spine joints
      desiredRobot.getRootJoint().updateFramesRecursively();
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Arm setpoints:");
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Joint angles"), handPoseDataTypeToSend == HandDataType.JOINT_ANGLES))
      {
         handPoseDataTypeToSend = HandDataType.JOINT_ANGLES;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Pose World"), handPoseDataTypeToSend == HandDataType.POSE_WORLD))
      {
         handPoseDataTypeToSend = HandDataType.POSE_WORLD;
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get("Pose Chest"), handPoseDataTypeToSend == HandDataType.POSE_CHEST))
      {
         handPoseDataTypeToSend = HandDataType.POSE_CHEST;
      }
   }

   public Runnable getSubmitDesiredArmSetpointsCallback(RobotSide robotSide)
   {
      Runnable runnable = () ->
      {
         if (handPoseDataTypeToSend == HandDataType.JOINT_ANGLES)
         {
            double[] jointAngles = new double[armJointNames.length];
            int i = -1;
            for (ArmJointName armJoint : armJointNames)
            {
               jointAngles[++i] = desiredRobot.getArmJoint(robotSide, armJoint).getQ();
            }

            LogTools.info("Sending ArmTrajectoryMessage");
            ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide,
                                                                                                        teleoperationParameters.getTrajectoryTime(),
                                                                                                        jointAngles);
            ros2Helper.publishToController(armTrajectoryMessage);
         }
         else if (handPoseDataTypeToSend == HandDataType.POSE_WORLD || handPoseDataTypeToSend == HandDataType.POSE_CHEST)
         {
            FramePose3D lastCorrectedDesiredHandControlFramePose = lastCorrectedDesiredHandControlFramePoses.get(robotSide);

            ReferenceFrame frame;
            if (handPoseDataTypeToSend == HandDataType.POSE_WORLD)
            {
               frame = ReferenceFrame.getWorldFrame();
            }
            else
            {
               frame = syncedRobot.getReferenceFrames().getChestFrame();
            }

            lastCorrectedDesiredHandControlFramePose.changeFrame(frame);

            LogTools.info("Sending HandTrajectoryMessage");
            HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide,
                                                                                                           teleoperationParameters.getTrajectoryTime(),
                                                                                                           lastCorrectedDesiredHandControlFramePose,
                                                                                                           frame);
            long dataFrameId = MessageTools.toFrameId(frame);
            handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(dataFrameId);
            ros2Helper.publishToController(handTrajectoryMessage);

            lastCorrectedDesiredHandControlFramePose.changeFrame(desiredRobot.getChest().getBodyFixedFrame());
         }
      };
      return runnable;
   }

   public void setDesiredToCurrent()
   {
      lastCorrectedDesiredHandControlFramePoses.clear();
      for (RobotSide robotSide : RobotSide.values)
         copyOneDofJoints(actualArmJacobians.get(robotSide).getJointsInOrder(), desiredArmJacobians.get(robotSide).getJointsInOrder());
   }

   private void computeArmJacobians(RobotSide currentlyUpdatingSide)
   {
      desiredArmJacobians.get(currentlyUpdatingSide).compute();
      actualArmJacobians.get(currentlyUpdatingSide).compute();
   }

   private static void copyOneDofJoints(JointBasics[] inverseDynamicsJoints1, JointBasics[] inverseDynamicsJoints2)
   {
      OneDoFJointBasics[] oneDofJoints1 = MultiBodySystemTools.filterJoints(inverseDynamicsJoints1, OneDoFJointBasics.class);
      OneDoFJointBasics[] oneDofJoints2 = MultiBodySystemTools.filterJoints(inverseDynamicsJoints2, OneDoFJointBasics.class);

      for (int i = 0; i < oneDofJoints1.length; i++)
      {
         oneDofJoints2[i].setQ(oneDofJoints1[i].getQ());
      }
   }
}