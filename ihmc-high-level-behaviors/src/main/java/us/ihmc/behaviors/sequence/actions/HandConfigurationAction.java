package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.tools.Timer;

public class HandConfigurationAction extends HandConfigurationActionData implements BehaviorAction
{
   /** TODO: Make this variable. */
   private static final double WAIT_TIME = 0.5;

   private final ROS2ControllerHelper ros2ControllerHelper;
   private final Timer executionTimer = new Timer();
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();

   public HandConfigurationAction(ROS2ControllerHelper ros2ControllerHelper)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
   }

   @Override
   public void executeAction()
   {
      HandDesiredConfigurationMessage message
            = HumanoidMessageTools.createHandDesiredConfigurationMessage(getSide(),
                                                                         HandConfiguration.values[getHandConfigurationIndex()]);
      ros2ControllerHelper.publish(ROS2Tools::getHandConfigurationTopic, message);

      executionTimer.reset();
   }

   @Override
   public boolean isExecuting()
   {
      executionStatusMessage.setNominalExecutionDuration(WAIT_TIME);
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
      ros2ControllerHelper.publish(BehaviorActionSequence.ACTION_EXECUTION_STATUS, executionStatusMessage);

      return executionTimer.isRunning(WAIT_TIME);
   }
}
