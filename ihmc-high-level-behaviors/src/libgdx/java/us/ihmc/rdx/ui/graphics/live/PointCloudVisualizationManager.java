package us.ihmc.rdx.ui.graphics.live;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLIntBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;

import java.nio.FloatBuffer;

/**
 * <p>
 * {@link PointCloudVisualizationManager} holds {@link ROS2PointCloudProvider} and {@link RDXPointCloudVisualizer} and works as a communicator between the two.
 * One can create another Manager that holds other PointCloudProvider and {@link RDXPointCloudVisualizer} to visualize PointCloud from another source.
 * This class is not intended to be re-usable. Providers and visualizers are made to be reusable (e.g. {@link ROS2PointCloudProvider} and
 * {@link RDXPointCloudVisualizer})
 * </p>
 **/

public class PointCloudVisualizationManager
{
   private final int pointsPerSegment;
   private final int numberOfSegments;
   private final int numberOfElementsPerPoint;
   private static final float POINT_SIZE = 0.01f;

   // Reads in pointCloud data and provides pointCloud data type to visualizer.
   private final ROS2PointCloudProvider pointCloudProvider;
   // Renders pointCloud received
   private final RDXPointCloudVisualizer pointCloudVisualizer;

   // set up openCL kernel to read in data and store to vertexBuffer (discretized)
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private OpenCLFloatBuffer undiscretizedVertexBuffer;
   private OpenCLIntBuffer decompressedOpenCLIntBuffer;
   private OpenCLFloatBuffer parametersOpenCLFloatBuffer;

   public PointCloudVisualizationManager(ROS2NodeInterface ros2Node,
                                         ROS2Topic<?> topic,
                                         String visualizerTitle,
                                         int pointsPerSegment,
                                         int numberOfSegments,
                                         int numberOfElementsPerPoint)
   {
      this.pointsPerSegment = pointsPerSegment;
      this.numberOfSegments = numberOfSegments;
      this.numberOfElementsPerPoint = numberOfElementsPerPoint;
      pointCloudProvider = new ROS2PointCloudProvider(ros2Node,topic, pointsPerSegment, numberOfSegments, numberOfElementsPerPoint);
      pointCloudVisualizer = new RDXPointCloudVisualizer(visualizerTitle,
                                                         topic.getName(),
                                                         pointsPerSegment,
                                                         numberOfSegments,
                                                         pointCloudProvider.getPointCloud().getNumberOfElementsPerPoint());

      decompressedOpenCLIntBuffer = new OpenCLIntBuffer(pointCloudProvider.getPointCloud().getNumberOfPoints(),
                                                        pointCloudProvider.getPointCloud().getData().asIntBuffer());
   }

   public void create()
   {
      pointCloudVisualizer.create();
      pointCloudProvider.create(pointCloudVisualizer.getVertexBuffer());

      // Set up for OpenCL kernel
      openCLManager = new OpenCLManager();
      openCLManager.create();
      openCLProgram = openCLManager.loadProgram("FusedSensorPointCloudSubscriberVisualizer");
      unpackPointCloudKernel = openCLManager.createKernel(openCLProgram, "unpackPointCloud");
      parametersOpenCLFloatBuffer = new OpenCLFloatBuffer(2);
      parametersOpenCLFloatBuffer.createOpenCLBufferObject(openCLManager);
      decompressedOpenCLIntBuffer = new OpenCLIntBuffer(pointsPerSegment * 4);
      decompressedOpenCLIntBuffer.createOpenCLBufferObject(openCLManager);
      undiscretizedVertexBuffer = new OpenCLFloatBuffer(pointsPerSegment * numberOfElementsPerPoint, pointCloudVisualizer.getVertexBuffer());
      undiscretizedVertexBuffer.createOpenCLBufferObject(openCLManager);
   }

   public void update()
   {
      if (pointCloudVisualizer.isActive())
      {
         pointCloudVisualizer.update();
         if (pointCloudProvider.updateFusedPointCloud())
         {
            pointCloudVisualizer.setLatestSegmentIndex(pointCloudProvider.getLatestSegmentIndex());
            pointCloudVisualizer.updatePointCloud(getUndiscretizedPointCloud());
            pointCloudVisualizer.recordEventFrequency(pointCloudProvider.pollMessageQueued());
            pointCloudVisualizer.updateMeshFastest();
         }
      }
   }

   // outputs in the form of float[] (x,y,z,r,g,b,a, pointSize)
   private FloatBuffer getUndiscretizedPointCloud()
   {
      parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(0, pointCloudProvider.getLatestSegmentIndex());
      parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(1, POINT_SIZE);
      parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(2, pointsPerSegment);

      parametersOpenCLFloatBuffer.writeOpenCLBufferObject(openCLManager);
      decompressedOpenCLIntBuffer.writeOpenCLBufferObject(openCLManager);

      openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 1, decompressedOpenCLIntBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(unpackPointCloudKernel, 2, undiscretizedVertexBuffer.getOpenCLBufferObject());
      openCLManager.execute1D(unpackPointCloudKernel, pointsPerSegment);
      undiscretizedVertexBuffer.readOpenCLBufferObject(openCLManager);

      return undiscretizedVertexBuffer.getBackingDirectFloatBuffer();
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
