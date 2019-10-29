package us.ihmc.humanoidBehaviors;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2NodeInterface;

public class RemoteREAInterface
{
   private final IHMCROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;
   private final ROS2Input<PlanarRegionsListMessage> planarRegionsList;

   public RemoteREAInterface(Ros2NodeInterface ros2Node)
   {
      reaStateRequestPublisher = new IHMCROS2Publisher<>(ros2Node, REAStateRequestMessage.class, null, ROS2Tools.REA);
      planarRegionsList = new ROS2Input<>(ros2Node, PlanarRegionsListMessage.class, null, ROS2Tools.REA);
   }

   public PlanarRegionsList getLatestPlanarRegionList()
   {
      return PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsList.getLatest());
   }

   public PlanarRegionsListMessage getLatestPlanarRegionListMessage()
   {
      return planarRegionsList.getLatest();
   }

   public void clearREA()
   {
      REAStateRequestMessage clearMessage = new REAStateRequestMessage();
      clearMessage.setRequestClear(true);
      reaStateRequestPublisher.publish(clearMessage);
   }
}
