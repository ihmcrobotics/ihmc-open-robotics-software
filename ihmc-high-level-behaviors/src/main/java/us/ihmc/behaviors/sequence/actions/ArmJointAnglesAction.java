package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.tools.Timer;

public class ArmJointAnglesAction extends ArmJointAnglesActionData implements BehaviorAction
{
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final Timer executionTimer = new Timer();

   public ArmJointAnglesAction(ROS2ControllerHelper ros2ControllerHelper)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
   }

   @Override
   public void executeAction()
   {
      double[] jointAngleArray = new double[NUMBER_OF_JOINTS];
      for (int i = 0; i < NUMBER_OF_JOINTS; i++)
      {
         jointAngleArray[i] = getJointAngles()[i];
      }
      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(getSide(), getTrajectoryDuration(), jointAngleArray);
      ros2ControllerHelper.publishToController(armTrajectoryMessage);
      executionTimer.reset();
   }

   @Override
   public boolean isExecuting()
   {
      return executionTimer.isRunning(getTrajectoryDuration());
   }
}
