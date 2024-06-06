package us.ihmc.rdx.ui.graphics.ros2;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

public class RDXROS2PlanarRegionsVisualizer extends RDXPlanarRegionsVisualizerBasics implements ROS2TopicHolder<PlanarRegionsListMessage>
{
   private final ROS2Topic<PlanarRegionsListMessage> topic;

   public RDXROS2PlanarRegionsVisualizer(String title, ROS2NodeInterface ros2Node, ROS2Topic<PlanarRegionsListMessage> topic)
   {
      super(title);
      this.topic = topic;
      new ROS2Callback<>(ros2Node, topic, this::acceptMessage);
   }

   private void acceptMessage(PlanarRegionsListMessage planarRegionsListMessage)
   {
      getFrequencyPlot().recordEvent();
      if (isActive())
      {
         getExecutorService().clearQueueAndExecute(() ->
         {
            PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
            setNumberOfPlanarRegions(planarRegionsList.getNumberOfPlanarRegions());
            getPlanarRegionsGraphic().generateMeshes(planarRegionsList);
         });
      }
   }

   @Override
   public ROS2Topic<PlanarRegionsListMessage> getTopic()
   {
      return topic;
   }
}
