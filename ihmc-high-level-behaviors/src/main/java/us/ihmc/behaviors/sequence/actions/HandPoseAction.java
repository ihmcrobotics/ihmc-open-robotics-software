package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.behaviors.sequence.BehaviorActionSequenceTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;
import us.ihmc.tools.thread.Throttler;

public class HandPoseAction extends HandPoseActionData implements BehaviorAction
{
   public static final double TRANSLATION_TOLERANCE = 0.15;
   public static final double ROTATION_TOLERANCE = Math.toRadians(10.0);
   public static final double BROKEN_WRIST_ROTATION_TOLERANCE = Math.toRadians(90.0);

   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final FramePose3D desiredHandControlPose = new FramePose3D();
   private final FramePose3D syncedHandControlPose = new FramePose3D();
   private final HandPoseJointAnglesStatusMessage handPoseJointAnglesStatus = new HandPoseJointAnglesStatusMessage();
   private final Timer executionTimer = new Timer();
   private final Throttler warningThrottler = new Throttler().setFrequency(2.0);
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();

   public HandPoseAction(ROS2ControllerHelper ros2ControllerHelper,
                         ReferenceFrameLibrary referenceFrameLibrary,
                         DRCRobotModel robotModel,
                         ROS2SyncedRobotModel syncedRobot)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      setReferenceFrameLibrary(referenceFrameLibrary);

      for (RobotSide side : RobotSide.values)
      {
         armIKSolvers.put(side, new ArmIKSolver(side, robotModel, syncedRobot.getFullRobotModel()));
      }
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex)
   {
      if (actionIndex == nextExecutionIndex)
      {
         ArmIKSolver armIKSolver = armIKSolvers.get(getSide());
         armIKSolver.copyActualToWork();
         armIKSolver.update(getReferenceFrame());
         armIKSolver.solve();

         // Send the solution back to the UI so the user knows what's gonna happen with the arm.
         handPoseJointAnglesStatus.getActionInformation().setActionIndex(actionIndex);
         handPoseJointAnglesStatus.setRobotSide(getSide().toByte());
         handPoseJointAnglesStatus.setSolutionQuality(armIKSolver.getQuality());
         for (int i = 0; i < armIKSolver.getSolutionOneDoFJoints().length; i++)
         {
            handPoseJointAnglesStatus.getJointAngles()[i] = armIKSolver.getSolutionOneDoFJoints()[i].getQ();
         }
         ros2ControllerHelper.publish(BehaviorActionSequence.HAND_POSE_JOINT_ANGLES_STATUS, handPoseJointAnglesStatus);
      }
   }

   @Override
   public void executeAction()
   {
      ArmIKSolver armIKSolver = armIKSolvers.get(getSide());

      double solutionQuality = armIKSolver.getQuality();
      if (solutionQuality < 1.0)
      {
         LogTools.error("Solution is low quality ({}). Not sending.", solutionQuality);
      }

      OneDoFJointBasics[] solutionOneDoFJoints = armIKSolver.getSolutionOneDoFJoints();
      double[] jointAngles = new double[solutionOneDoFJoints.length];
      for (int i = 0; i < jointAngles.length; i++)
      {
         jointAngles[i] = solutionOneDoFJoints[i].getQ();
      }

      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(getSide(), getTrajectoryDuration(), jointAngles);
      armTrajectoryMessage.setForceExecution(true); // Prevent the command being rejected because robot is still finishing up walking
      ros2ControllerHelper.publishToController(armTrajectoryMessage);

      executionTimer.reset();
   }

   @Override
   public boolean isExecuting()
   {
      desiredHandControlPose.setFromReferenceFrame(getReferenceFrame());
      syncedHandControlPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getHandControlFrame(getSide()));

      // Left hand broke on Nadia and not in the robot model?
      double rotationTolerance = getSide() == RobotSide.LEFT ? BROKEN_WRIST_ROTATION_TOLERANCE : ROTATION_TOLERANCE;
      boolean isExecuting = BehaviorActionSequenceTools.isExecuting(desiredHandControlPose,
                                                                    syncedHandControlPose,
                                                                    TRANSLATION_TOLERANCE,
                                                                    rotationTolerance,
                                                                    getTrajectoryDuration(),
                                                                    executionTimer,
                                                                    warningThrottler);

      executionStatusMessage.setNominalExecutionDuration(getTrajectoryDuration());
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
      ros2ControllerHelper.publish(BehaviorActionSequence.ACTION_EXECUTION_STATUS, executionStatusMessage);

      return isExecuting;
   }
}
