package us.ihmc.communication;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.Ros2NodeInterface;

public class ROS2PlanarRegionsInput extends ROS2Input<PlanarRegionsListMessage>
{
   public ROS2PlanarRegionsInput(Ros2NodeInterface ros2Node, Class<PlanarRegionsListMessage> messageType, String topicName)
   {
      super(ros2Node, messageType, topicName);
   }

   public PlanarRegionsList getLatestAndConvert()
   {
      return PlanarRegionMessageConverter.convertToPlanarRegionsList(getLatest());
   }
}
