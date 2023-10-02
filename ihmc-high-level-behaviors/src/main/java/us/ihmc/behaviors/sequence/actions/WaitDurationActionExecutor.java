package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionExecutor;
import us.ihmc.tools.Timer;

public class WaitDurationActionExecutor extends WaitDurationActionDefinition implements BehaviorActionExecutor
{
   private final ROS2ControllerHelper ros2ControllerHelper;
   private int actionIndex;
   private final Timer executionTimer = new Timer();
   private boolean isExecuting;
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();

   public WaitDurationActionExecutor(ROS2ControllerHelper ros2ControllerHelper)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex, boolean concurrentActionIsNextForExecution)
   {
      update();

      this.actionIndex = actionIndex;
   }

   @Override
   public void triggerActionExecution()
   {
      executionTimer.reset();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      isExecuting = executionTimer.isRunning(getWaitDuration());

      executionStatusMessage.setActionIndex(actionIndex);
      executionStatusMessage.setNominalExecutionDuration(getWaitDuration());
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
   }

   @Override
   public ActionExecutionStatusMessage getExecutionStatusMessage()
   {
      return executionStatusMessage;
   }

   @Override
   public boolean isExecuting()
   {
      return isExecuting;
   }
}
