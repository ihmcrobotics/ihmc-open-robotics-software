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
import us.ihmc.ros2.ROS2NodeInterface;
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

/**
 * {@link ROS2PointCloudProvider} subscribes to ROS2 topic and reads in pointCloud data, stores into OpenCLFloatBuffer, and packs into {@link PointCloud}.
 * <p>
 * It expects to subscribe to {@link FusedSensorHeadPointCloudMessage}.
 * </p>
 **/

public class ROS2PointCloudProvider
{
   // NOTE: Subscribe to receive message.
   private final ROS2NodeInterface ros2Node;
   private final ROS2Topic<?> topic;
   private boolean messageQueued = false;

   // NOTE: Reference to message. (gets set when new message received from ros2 callback)
   private final AtomicReference<FusedSensorHeadPointCloudMessage> latestFusedSensorHeadPointCloudMessageReference = new AtomicReference<>(null);
   private final AtomicReference<LidarScanMessage> latestLidarScanMessageReference = new AtomicReference<>(null);
   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionMessageReference = new AtomicReference<>(null);

   // NOTE: Some parameters for data processing
   private static final float POINT_SIZE = 0.01f;
   private final int pointsPerSegment;
   private static final int inputBytesPerPoint = 4 * Integer.BYTES;
   private final Color color = new Color();
   private int latestSegmentIndex = -1;

   // NOTE: Decompress incoming binary data (compressed)
   private final ResettableExceptionHandlingExecutorService threadQueue;
   private final LZ4FastDecompressor lz4Decompressor = LZ4Factory.nativeInstance().fastDecompressor();
   private final ByteBuffer decompressionInputDirectBuffer;

   // NOTE: set up openCL kernel to read in data and store to vertexBuffer (discretized)
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel unpackPointCloudKernel;
   private OpenCLFloatBuffer discretizedPointBuffer;
   private OpenCLIntBuffer decompressedOpenCLIntBuffer;
   private OpenCLFloatBuffer parametersOpenCLFloatBuffer;

   // NOTE: Data type PointCloud, to be updated and used globally in the future whenever sending / receiving pointCloud data.
   private final PointCloud pointCloud;

   public ROS2PointCloudProvider(ROS2NodeInterface ros2Node, ROS2Topic<?> topic, int pointsPerSegment, int numberOfSegments)
   {
      this.ros2Node = ros2Node;
      this.topic = topic;
      this.pointsPerSegment = pointsPerSegment;
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
      discretizedPointBuffer = new OpenCLFloatBuffer(pointsPerSegment * 8, vertexBuffer);
      discretizedPointBuffer.createOpenCLBufferObject(openCLManager);
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
         parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(1, POINT_SIZE);
         parametersOpenCLFloatBuffer.getBytedecoFloatBufferPointer().put(2, pointsPerSegment);

         parametersOpenCLFloatBuffer.writeOpenCLBufferObject(openCLManager);
         decompressedOpenCLIntBuffer.writeOpenCLBufferObject(openCLManager);

         openCLManager.setKernelArgument(unpackPointCloudKernel, 0, parametersOpenCLFloatBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(unpackPointCloudKernel, 1, decompressedOpenCLIntBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(unpackPointCloudKernel, 2, discretizedPointBuffer.getOpenCLBufferObject());
         openCLManager.execute1D(unpackPointCloudKernel, pointsPerSegment);
         discretizedPointBuffer.readOpenCLBufferObject(openCLManager);

         pointCloud.setData(discretizedPointBuffer.getBytedecoFloatBufferPointer(), pointsPerSegment * 8);

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
      return xyzRGBASizeFloatBuffer -> 0;
   }

   public PointCloud getPointCloud()
   {
      return pointCloud;
   }

   public boolean pollMessageQueued()
   {
      boolean ret = messageQueued;
      messageQueued = false;
      return ret;
   }
}
