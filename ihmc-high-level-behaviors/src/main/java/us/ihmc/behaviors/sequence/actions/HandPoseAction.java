package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.HandPoseJointAnglesStatusMessage;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;

public class HandPoseAction extends HandPoseActionData implements BehaviorAction
{
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final HandPoseJointAnglesStatusMessage handPoseJointAnglesStatus = new HandPoseJointAnglesStatusMessage();
   private final Timer executionTimer = new Timer();

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
      else
      {
         LogTools.error("Solution is low quality ({}). Not sending.", solutionQuality);
      }
   }

   @Override
   public boolean isExecuting()
   {
      boolean trajectoryTimerRunning = executionTimer.isRunning(getTrajectoryDuration());

      RigidBodyTransform desired = getReferenceFrame().getTransformToRoot();
      RigidBodyTransform actual = syncedRobot.getFullRobotModel().getHandControlFrame(getSide()).getTransformToRoot();

      double reasonableAchievementWindowMeters = 0.02;
      double translationError = actual.getTranslation().differenceNorm(desired.getTranslation());
      boolean desiredTranslationAcheived = translationError <= reasonableAchievementWindowMeters;

      double reasonableAchievementWindowRadians = 0.04;
      double rotationError = actual.getRotation().distance(desired.getRotation(), true);
      boolean desiredRotationAcheived = rotationError <= reasonableAchievementWindowRadians;

      boolean desiredAchieved = desiredTranslationAcheived && desiredRotationAcheived;

      if (!trajectoryTimerRunning && !desiredAchieved && executionTimer.getElapsedTime() > (getTrajectoryDuration() + 0.1))
      {
         LogTools.warn("""
                       We didn't achieve the desired in time.
                          Elapsed time: %s s
                          Desired translation acheived: %b
                          Translation error: %.5f
                          Desired rotation achieved: %b
                          Rotation error: %.5f
                       """.formatted(executionTimer.getElapsedTime(),
                                     desiredTranslationAcheived,
                                     translationError,
                                     desiredRotationAcheived,
                                     rotationError));
      }

      return trajectoryTimerRunning | !desiredAchieved;
   }
}
