package us.ihmc.gdx.ui.graphics.live;

import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

public class PointCloudManager
{
   private final ROS2PointCloudProvider pointCloudProvider;
   private final GDXPointCloudVisualizer pointCloudVisualizer;

   public PointCloudManager(ROS2Node ros2Node, ROS2Topic<?> topic, String visualizerTitle, int pointsPerSegment, int numberOfSegments)
   {
      pointCloudProvider = new ROS2PointCloudProvider(ros2Node,topic, pointsPerSegment, numberOfSegments);
      pointCloudVisualizer = new GDXPointCloudVisualizer(visualizerTitle, topic.getName(), pointsPerSegment, numberOfSegments);
   }

   public void create()
   {
      pointCloudVisualizer.create();
      pointCloudProvider.create(pointCloudVisualizer.getVertexBuffer());
   }

   public void update()
   {
      if (pointCloudVisualizer.isActive())
      {
         pointCloudVisualizer.update();
//         if (pointCloudProvider.hasFusedPointCloud())
//         {
            pointCloudVisualizer.updateMeshFastest(pointCloudProvider.updateFusedPointCloudNumberOfPoints());
            pointCloudVisualizer.setLatestSegmentIndex(pointCloudProvider.getLatestSegmentIndex());
//         }
//         else
//         {
//            pointCloudVisualizer.updateMeshFastest(pointCloudProvider.updateAndGetBufferConsumer());
//         }
      }
   }

   public GDXPointCloudVisualizer getGDXPointCloudVisualizer()
   {
      return pointCloudVisualizer;
   }
}
