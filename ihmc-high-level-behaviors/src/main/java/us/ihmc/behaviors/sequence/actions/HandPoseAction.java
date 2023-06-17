package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ArmJointAnglesActionMessage;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.inverseKinematics.ArmIKSolver;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class HandPoseAction extends HandPoseActionData implements BehaviorAction
{
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final SideDependentList<ArmIKSolver> armIKSolvers = new SideDependentList<>();
   private final ArmJointAnglesActionMessage armJointAnglesStatus = new ArmJointAnglesActionMessage();

   public HandPoseAction(ROS2ControllerHelper ros2ControllerHelper,
                         ReferenceFrameLibrary referenceFrameLibrary,
                         DRCRobotModel robotModel,
                         ROS2SyncedRobotModel syncedRobot)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
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

         armJointAnglesStatus.getActionInformation().setActionIndex(actionIndex);
         armJointAnglesStatus.setRobotSide(getSide().toByte());
         for (int i = 0; i < armIKSolver.getSolutionOneDoFJoints().length; i++)
         {
            armJointAnglesStatus.getJointAngles()[i] = armIKSolver.getSolutionOneDoFJoints()[i].getQ();
         }
         ros2ControllerHelper.publish(BehaviorActionSequence.HAND_POSE_JOINT_ANGLES_STATUS, armJointAnglesStatus);
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
         ros2ControllerHelper.publishToController(armTrajectoryMessage);
      }
      else
      {
         LogTools.error("Solution is low quality ({}). Not sending.", solutionQuality);
      }
   }
}
