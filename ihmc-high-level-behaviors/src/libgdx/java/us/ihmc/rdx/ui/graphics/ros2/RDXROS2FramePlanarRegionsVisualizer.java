package us.ihmc.rdx.ui.graphics.ros2;

import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.rdx.ui.graphics.RDXPlanarRegionsVisualizer;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

public class RDXROS2FramePlanarRegionsVisualizer extends RDXROS2SingleTopicVisualizer<FramePlanarRegionsListMessage>
{
   private final RDXPlanarRegionsVisualizer planarRegionsVisualizer;
   private final ROS2Topic<FramePlanarRegionsListMessage> topic;

   public RDXROS2FramePlanarRegionsVisualizer(String title, ROS2NodeInterface ros2Node, ROS2Topic<FramePlanarRegionsListMessage> topic)
   {
      super(title);
      planarRegionsVisualizer = new RDXPlanarRegionsVisualizer(title);
      this.topic = topic;
      new ROS2Callback<>(ros2Node, topic, this::acceptMessage);
   }

   private void acceptMessage(FramePlanarRegionsListMessage framePlanarRegionsListMessage)
   {
      if (isActive())
      {
         planarRegionsVisualizer.getExecutorService().clearQueueAndExecute(() ->
         {
            PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsListInWorld(framePlanarRegionsListMessage);
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
   public ROS2Topic<FramePlanarRegionsListMessage> getTopic()
   {
      return topic;
   }
}
