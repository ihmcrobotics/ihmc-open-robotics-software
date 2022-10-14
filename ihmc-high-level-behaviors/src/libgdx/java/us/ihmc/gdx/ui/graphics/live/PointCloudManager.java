package us.ihmc.gdx.ui.graphics.live;

import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

public class PointCloudManager
{
   private final ROS2PointCloudProvider pointCloudProvider;
   private final GDXROS2PointCloudVisualizer2 visualizer2;

   public PointCloudManager(ROS2Node ros2Node, ROS2Topic<?> topic, String visualizerTitle, int pointsPerSegment, int numberOfSegments)
   {
      pointCloudProvider = new ROS2PointCloudProvider(ros2Node,topic, pointsPerSegment, numberOfSegments);
      visualizer2 = new GDXROS2PointCloudVisualizer2(visualizerTitle, topic.getName(), pointsPerSegment, numberOfSegments);
   }

   public void create()
   {
      visualizer2.create();
      pointCloudProvider.create(visualizer2.getVertexBuffer());
   }

   public void update()
   {
      if (visualizer2.isActive())
      {
         visualizer2.update();
         if (pointCloudProvider.hasFusedPointCloud())
         {
            visualizer2.updateMeshFastest(pointCloudProvider.updateFusedPointCloudNumberOfPoints());
         }
         else
         {
            visualizer2.updateMeshFastest(pointCloudProvider.updateAndGetBufferConsumer());
         }
         visualizer2.setLatestSegmentIndex(pointCloudProvider.getLatestSegmentIndex());
      }
   }

   public GDXROS2PointCloudVisualizer2 getGDXROS2PointCloudVisualizer2()
   {
      return visualizer2;
   }
}
