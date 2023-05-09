package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;

public class HandConfigurationAction extends HandConfigurationActionData implements BehaviorAction
{
   private final ROS2ControllerHelper ros2ControllerHelper;

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
   }
}
