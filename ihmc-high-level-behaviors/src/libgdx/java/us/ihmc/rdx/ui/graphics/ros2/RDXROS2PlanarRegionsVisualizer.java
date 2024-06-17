package us.ihmc.rdx.ui.graphics.ros2;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.rdx.ui.graphics.RDXPlanarRegionsVisualizer;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

public class RDXROS2PlanarRegionsVisualizer extends RDXROS2SingleTopicVisualizer<PlanarRegionsListMessage>
{
   private final RDXPlanarRegionsVisualizer planarRegionsVisualizer;
   private final ROS2Topic<PlanarRegionsListMessage> topic;

   public RDXROS2PlanarRegionsVisualizer(String title, ROS2NodeInterface ros2Node, ROS2Topic<PlanarRegionsListMessage> topic)
   {
      super(title);
      this.planarRegionsVisualizer = new RDXPlanarRegionsVisualizer(title);
      this.topic = topic;
      new ROS2Callback<>(ros2Node, topic, this::acceptMessage);
   }

   private void acceptMessage(PlanarRegionsListMessage planarRegionsListMessage)
   {
      if (isActive())
      {
         planarRegionsVisualizer.getExecutorService().clearQueueAndExecute(() ->
         {
            PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
            planarRegionsVisualizer.setNumberOfPlanarRegions(planarRegionsList.getNumberOfPlanarRegions());
            planarRegionsVisualizer.getPlanarRegionsGraphic().generateMeshes(planarRegionsList);
         });
      }

      getFrequency().ping();
   }

   @Override
   public void renderImGuiWidgets()
   {

   }

   @Override
   public ROS2Topic<PlanarRegionsListMessage> getTopic()
   {
      return topic;
   }
}
