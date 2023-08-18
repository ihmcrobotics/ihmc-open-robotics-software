package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.tools.Timer;

public class WaitDurationAction extends WaitDurationActionData implements BehaviorAction
{
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final Timer timer = new Timer();
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();

   public WaitDurationAction(ROS2ControllerHelper ros2ControllerHelper)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
   }

   @Override
   public void executeAction()
   {
      timer.reset();
   }

   @Override
   public boolean isExecuting()
   {
      executionStatusMessage.setNominalExecutionDuration(getWaitDuration());
      executionStatusMessage.setElapsedExecutionTime(timer.getElapsedTime());
      ros2ControllerHelper.publish(BehaviorActionSequence.ACTION_EXECUTION_STATUS, executionStatusMessage);

      return timer.isRunning(getWaitDuration());
   }
}
