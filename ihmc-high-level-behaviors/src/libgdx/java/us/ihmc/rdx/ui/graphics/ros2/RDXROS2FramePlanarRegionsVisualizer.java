package us.ihmc.rdx.ui.graphics.ros2;

import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

public class RDXROS2FramePlanarRegionsVisualizer extends RDXPlanarRegionsVisualizerBasics
{
   public RDXROS2FramePlanarRegionsVisualizer(String title, ROS2NodeInterface ros2Node, ROS2Topic<FramePlanarRegionsListMessage> topic)
   {
      super(title + " (ROS 2)", topic.getName());
      new IHMCROS2Callback<>(ros2Node, topic, this::acceptMessage);
   }

   private void acceptMessage(FramePlanarRegionsListMessage framePlanarRegionsListMessage)
   {
      getFrequencyPlot().recordEvent();
      if (isActive())
      {
         getExecutorService().clearQueueAndExecute(() ->
         {
            FramePlanarRegionsList framePlanarRegionsList = PlanarRegionMessageConverter.convertToFramePlanarRegionsList(framePlanarRegionsListMessage);
            framePlanarRegionsList.changeFrameToWorld();
            setNumberOfPlanarRegions(framePlanarRegionsList.getPlanarRegionsList().getNumberOfPlanarRegions());
            getPlanarRegionsGraphic().generateMeshes(framePlanarRegionsList.getPlanarRegionsList());
         });
      }
   }
}
