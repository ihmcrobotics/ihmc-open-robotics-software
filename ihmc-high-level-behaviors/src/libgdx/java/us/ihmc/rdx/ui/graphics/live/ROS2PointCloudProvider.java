package us.ihmc.rdx.ui.graphics.live;

import com.badlogic.gdx.graphics.Color;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import net.jpountz.lz4.LZ4Factory;
import net.jpountz.lz4.LZ4FastDecompressor;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import perception_msgs.msg.dds.FusedSensorHeadPointCloudMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.communication.packets.StereoPointCloudCompression;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLIntBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.BufferOverflowException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Function;

public class ROS2PointCloudProvider
{
   private final ROS2Node ros2Node;
   private final ROS2Topic<?> topic;
   private final float pointSize = 0.01f;

   private final int pointsPerSegment;
   private final int numberOfSegments;
   private final int totalNumberOfPoints;
   private final int inputBytesPerPoint = 4 * Integer.BYTES;
   private final ResettableExceptionHandlingExecutorService threadQueue;
   private final LZ4FastDecompressor lz4Decompressor = LZ4Factory.nativeInstance().fastDecompressor();
   private final ByteBuffer decompressionInputDirectBuffer;
   private final AtomicReference<FusedSensorHeadPointCloudMessage> latestFusedSensorHeadPointCloudMessageReference = new AtomicReference<>(null);
   private final AtomicReference<LidarScanMessage> latestLidarScanMessageReference = new AtomicReference<>(null);
   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionMessageReference = new AtomicReference<>(null);
   private final Color color = new Color();
   private int latestSegmentIndex = -1;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private OpenCLFloatBuffer pointCloudVertexBuffer;
   private OpenCLIntBuffer decompressedOpenCLIntBuffer;
   private OpenCLFloatBuffer parametersOpenCLFloatBuffer;
   private final PointCloud pointCloud;
   private boolean messageQueued = false;

   public ROS2PointCloudProvider(ROS2Node ros2Node, ROS2Topic<?> topic, int pointsPerSegment, int numberOfSegments)
   {
      this.ros2Node = ros2Node;
      this.topic = topic;
      this.pointsPerSegment = pointsPerSegment;
      this.numberOfSegments = numberOfSegments;
      totalNumberOfPoints = pointsPerSegment * numberOfSegments;
      threadQueue = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
      decompressionInputDirectBuffer = ByteBuffer.allocateDirect(pointsPerSegment * inputBytesPerPoint);
      decompressionInputDirectBuffer.order(ByteOrder.nativeOrder());
      pointCloud = new PointCloud(pointsPerSegment, 8);
      subscribe();
   }

   public void create(FloatBuffer vertexBuffer)
   {
      openCLManager = new OpenCLManager();
      openCLManager.create();
      openCLProgram = openCLManager.loadProgram("FusedSensorPointCloudSubscriberVisualizer");
      unpackPointCloudKernel = openCLManager.createKernel(openCLProgram, "unpackPointCloud");

      parametersOpenCLFloatBuffer = new OpenCLFloatBuffer(2);
      parametersOpenCLFloatBuffer.createOpenCLBufferObject(openCLManager);
      decompressedOpenCLIntBuffer = new OpenCLIntBuffer(pointsPerSegment * 4);
      decompressedOpenCLIntBuffer.createOpenCLBufferObject(openCLManager);
      pointCloudVertexBuffer = new OpenCLFloatBuffer(pointsPerSegment * 8, vertexBuffer);
      pointCloudVertexBuffer.createOpenCLBufferObject(openCLManager);
   }

   // TODO: old one, need to be deleted at some point
   private void queueRenderStereoVisionPointCloud(StereoVisionPointCloudMessage message)
   {
      //      frequencyPlot.recordEvent();
      // TODO: Possibly decompress on a thread here
      // TODO: threadQueue.clearQueueAndExecute(() ->
      latestStereoVisionMessageReference.set(message);
   }

   // TODO: old one, need to be deleted at some point
   private void queueRenderLidarScan(LidarScanMessage message)
   {
      //      frequencyPlot.recordEvent();
      latestLidarScanMessageReference.set(message);
   }

   private void queueRenderFusedSensorHeadPointCloud(FusedSensorHeadPointCloudMessage message)
   {
      messageQueued = true;
      latestFusedSensorHeadPointCloudMessageReference.set(message);
   }

   private void subscribe()
   {
      if (topic.getType().equals(LidarScanMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node, topic.withType(LidarScanMessage.class), this::queueRenderLidarScan);
      }
      else if (topic.getType().equals(StereoVisionPointCloudMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node, topic.withType(StereoVisionPointCloudMessage.class), this::queueRenderStereoVisionPointCloud);
      }
      else if (topic.getType().equals(FusedSensorHeadPointCloudMessage.class))
      {
         new IHMCROS2Callback<>(ros2Node,
                                topic.withType(FusedSensorHeadPointCloudMessage.class),
                                ROS2QosProfile.BEST_EFFORT(),
                                this::queueRenderFusedSensorHeadPointCloud);
      }
   }

   public int getLatestSegmentIndex()
   {
      return latestSegmentIndex;
   }

   public boolean updateFusedPointCloudNumberOfPoints()
   {
      FusedSensorHeadPointCloudMessage fusedMessage = latestFusedSensorHeadPointCloudMessageReference.getAndSet(null);
      if (fusedMessage != null)
      {
         decompressionInputDirectBuffer.rewind();
         int numberOfBytes = fusedMessage.getScan().size();
         decompressionInputDirectBuffer.limit(numberOfBytes);
         for (int i = 0; i < numberOfBytes; i++)
         {
            decompressionInputDirectBuffer.put(fusedMessage.getScan().get(i));
         }
         decompressionInputDirectBuffer.flip();
         decompressedOpenCLIntBuffer.getBackingDirectByteBuffer().rewind();
         // TODO: Look at using bytedeco LZ4 1.9.X, which is supposed to be 12% faster than 1.8.X
         lz4Decompressor.decompress(decompressionInputDirectBuffer, decompressedOpenCLIntBuffer.getBackingDirectByteBuffer());
         decompressedOpenCLIntBuffer.getBackingDirectByteBuffer().rewind();

         latestSegmentIndex = (int) fusedMessage.getSegmentIndex();

         parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(0, latestSegmentIndex);
         parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(1, pointSize);
         parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(2, pointsPerSegment);

         parametersOpenCLFloatBuffer.writeOpenCLBufferObject(openCLManager);
         decompressedOpenCLIntBuffer.writeOpenCLBufferObject(openCLManager);

         openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(unpackPointCloudKernel, 1, decompressedOpenCLIntBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(unpackPointCloudKernel, 2, pointCloudVertexBuffer.getOpenCLBufferObject());
         openCLManager.execute1D(unpackPointCloudKernel, pointsPerSegment);
         pointCloudVertexBuffer.readOpenCLBufferObject(openCLManager);

         pointCloud.setData(pointCloudVertexBuffer.getBytedecoFloatBufferPointer(), pointsPerSegment * 8);

         return true;
      }

      return false;
   }

   public Function<FloatBuffer, Integer> updateAndGetBufferConsumer()
   {
      LidarScanMessage latestLidarScanMessage = latestLidarScanMessageReference.getAndSet(null);
      if (latestLidarScanMessage != null)
      {
         int numberOfScanPoints = latestLidarScanMessage.getNumberOfPoints();

         return xyzRGBASizeFloatBuffer ->
         {
            LidarPointCloudCompression.decompressPointCloud(latestLidarScanMessage.getScan(), numberOfScanPoints, (i, x, y, z) ->
            {
               xyzRGBASizeFloatBuffer.put((float) x);
               xyzRGBASizeFloatBuffer.put((float) y);
               xyzRGBASizeFloatBuffer.put((float) z);
               xyzRGBASizeFloatBuffer.put(color.r);
               xyzRGBASizeFloatBuffer.put(color.g);
               xyzRGBASizeFloatBuffer.put(color.b);
               xyzRGBASizeFloatBuffer.put(color.a);
            });

            return numberOfScanPoints;
         };
      }

      StereoVisionPointCloudMessage latestStereoVisionMessage = latestStereoVisionMessageReference.getAndSet(null);
      if (latestStereoVisionMessage != null)
      {
         return xyzRGBASizeFloatBuffer ->
         {
            StereoPointCloudCompression.decompressPointCloud(latestStereoVisionMessage, (x, y, z) ->
            {
               try
               {
                  xyzRGBASizeFloatBuffer.put((float) x);
                  xyzRGBASizeFloatBuffer.put((float) y);
                  xyzRGBASizeFloatBuffer.put((float) z);
                  xyzRGBASizeFloatBuffer.put(color.r);
                  xyzRGBASizeFloatBuffer.put(color.g);
                  xyzRGBASizeFloatBuffer.put(color.b);
                  xyzRGBASizeFloatBuffer.put(color.a);
               }
               catch (BufferOverflowException e)
               {
                  e.printStackTrace();
               }
            });

            return latestStereoVisionMessage.getNumberOfPoints();
         };
      }
      return xyzRGBASizeFloatBuffer-> 0;
   }

   public PointCloud getPointCloud()
   {
      return pointCloud;
   }

   public boolean isMessageQueued()
   {
      return messageQueued;
   }

   public void setMessageQueued(boolean messageQueued)
   {
      this.messageQueued = messageQueued;
   }
}
