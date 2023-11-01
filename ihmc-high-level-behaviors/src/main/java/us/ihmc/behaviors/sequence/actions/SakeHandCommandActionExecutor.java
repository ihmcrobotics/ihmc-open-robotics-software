package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.tools.Timer;

public class SakeHandCommandActionExecutor extends ActionNodeExecutor<SakeHandCommandActionState, SakeHandCommandActionDefinition>
{
   /** TODO: Make this variable. */
   private static final double WAIT_TIME = 0.5;

   private final SakeHandCommandActionState state;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final Timer executionTimer = new Timer();

   public SakeHandCommandActionExecutor(long id, ROS2ControllerHelper ros2ControllerHelper)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;

      state = new SakeHandCommandActionState(id, ROS2ActorDesignation.ROBOT);
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void triggerActionExecution()
   {
      SakeHandDesiredCommandMessage message = new SakeHandDesiredCommandMessage();
      message.setRobotSide(getDefinition().getSide().toByte());
      message.setDesiredHandConfiguration((byte) SakeHandCommandOption.values[getDefinition().getHandConfigurationIndex()].getCommandNumber());
      message.setPostionRatio(getDefinition().getGoalPosition());
      message.setTorqueRatio(getDefinition().getGoalTorque());

      ros2ControllerHelper.publish(ROS2Tools::getHandSakeCommandTopic, message);

      executionTimer.reset();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(executionTimer.isRunning(WAIT_TIME));

      state.setActionIndex(state.getActionIndex());
      state.setNominalExecutionDuration(WAIT_TIME);
      state.setElapsedExecutionTime(executionTimer.getElapsedTime());
   }

   @Override
   public SakeHandCommandActionState getState()
   {
      return state;
   }
}
