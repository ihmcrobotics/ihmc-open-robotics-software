package us.ihmc.rdx.ui.graphics.ros2;

import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

public class RDXROS2FramePlanarRegionsVisualizer extends RDXPlanarRegionsVisualizerBasics
{
   public RDXROS2FramePlanarRegionsVisualizer(String title, ROS2NodeInterface ros2Node, ROS2Topic<FramePlanarRegionsListMessage> topic)
   {
      super(title, topic.getName());
      new IHMCROS2Callback<>(ros2Node, topic, this::acceptMessage);
   }

   private void acceptMessage(FramePlanarRegionsListMessage framePlanarRegionsListMessage)
   {
      getFrequencyPlot().recordEvent();
      if (isActive())
      {
         getExecutorService().clearQueueAndExecute(() ->
         {
            PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsListInWorld(framePlanarRegionsListMessage);
            setNumberOfPlanarRegions(planarRegionsList.getNumberOfPlanarRegions());
            getPlanarRegionsGraphic().generateMeshes(planarRegionsList);
         });
      }
   }
}
