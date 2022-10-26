package us.ihmc.rdx.ui.graphics.live;

import perception_msgs.msg.dds.FusedSensorHeadPointCloudMessage;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

/**
 * <p>
 * {@link PointCloudManager} holds {@link ROS2PointCloudProvider} and {@link RDXPointCloudVisualizer} and works as a communicator between the two.
 * One can create another Manager that holds other PointCloudProvider and {@link RDXPointCloudVisualizer} to visualize PointCloud from another source.
 * This class is not intended to be re-usable. Providers and visualizers are made to be reusable (e.g. {@link ROS2PointCloudProvider} and
 * {@link RDXPointCloudVisualizer})
 * </p>
 **/

public class PointCloudManager
{
   // NOTE: Reads in pointCloud data and provides pointCloud data type to visualizer.
   private final ROS2PointCloudProvider pointCloudProvider;
   // NOTE: Renders pointCloud received
   private final RDXPointCloudVisualizer pointCloudVisualizer;

   public PointCloudManager(ROS2NodeInterface ros2Node, ROS2Topic<?> topic, String visualizerTitle, int pointsPerSegment, int numberOfSegments)
   {
      pointCloudProvider = new ROS2PointCloudProvider(ros2Node,topic, pointsPerSegment, numberOfSegments);
      pointCloudVisualizer = new RDXPointCloudVisualizer(visualizerTitle,
                                                         topic.getName(),
                                                         pointsPerSegment,
                                                         numberOfSegments,
                                                         pointCloudProvider.getPointCloud().getNumberOfElementsPerPoint());
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
         if (pointCloudProvider.updateFusedPointCloudNumberOfPoints())
         {
            pointCloudVisualizer.setLatestSegmentIndex(pointCloudProvider.getLatestSegmentIndex());
            pointCloudVisualizer.updatePointCloud(pointCloudProvider.getPointCloud());
            pointCloudVisualizer.recordEventFrequency(pointCloudProvider.pollMessageQueued());
            pointCloudVisualizer.updateMeshFastest();
         }
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
