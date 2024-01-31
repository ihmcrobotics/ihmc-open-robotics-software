package us.ihmc.rdx.ui.graphics.ros2;

import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

public class RDXROS2PlanarRegionsVisualizer extends RDXPlanarRegionsVisualizerBasics
{
   public RDXROS2PlanarRegionsVisualizer(String title, ROS2NodeInterface ros2Node, ROS2Topic<PlanarRegionsListMessage> topic)
   {
      super(title + " (ROS 2)", topic.getName());
      new IHMCROS2Callback<>(ros2Node, topic, this::acceptMessage);
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
}
