package us.ihmc.rdx.ui.graphics.live;

import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

public class PointCloudManager
{
   private final ROS2PointCloudProvider pointCloudProvider;
   private final RDXPointCloudVisualizer pointCloudVisualizer;

   public PointCloudManager(ROS2Node ros2Node, ROS2Topic<?> topic, String visualizerTitle, int pointsPerSegment, int numberOfSegments)
   {
      pointCloudProvider = new ROS2PointCloudProvider(ros2Node,topic, pointsPerSegment, numberOfSegments);
      pointCloudVisualizer = new RDXPointCloudVisualizer(visualizerTitle, topic.getName(), pointsPerSegment, numberOfSegments);
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
         pointCloudVisualizer.setTotalNumberOfPoints(pointCloudProvider.updateFusedPointCloudNumberOfPoints());
         pointCloudVisualizer.setLatestSegmentIndex(pointCloudProvider.getLatestSegmentIndex());
         pointCloudVisualizer.updateMeshFastest();
//         }
//         else
//         {
//            pointCloudVisualizer.updateMeshFastest(pointCloudProvider.updateAndGetBufferConsumer());
//         }
      }
   }

   public RDXPointCloudVisualizer getGDXPointCloudVisualizer()
   {
      return pointCloudVisualizer;
   }

   public ROS2PointCloudProvider getPointCloudProvider()
   {
      return pointCloudProvider;
   }
}
