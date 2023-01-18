package us.ihmc.perception.ouster;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.*;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.ByteBuffer;
import java.util.*;
import java.util.function.Supplier;

/**
 * This class publishes a PNG compressed depth image from the Ouster as fast as the frames come in.
 */
public class OusterDriverAndDepthPublisher
{
   private final Activator nativesLoadedActivator;

   private final RealtimeROS2Node realtimeROS2Node;
   private final ROS2Topic<?>[] outputTopics;
   private final List<Class<?>> outputTopicsTypes = new ArrayList<>();
   private final HashMap<ROS2Topic<?>, IHMCRealtimeROS2Publisher> publisherMap = new HashMap<>();

   private final Supplier<ReferenceFrame> sensorFrameUpdater;
   private final FramePose3D cameraPose = new FramePose3D();
   private final ResettableExceptionHandlingExecutorService extractCompressAndPublishThread;
   private final NettyOuster ouster;
   private int depthWidth;
   private int depthHeight;
   private int numberOfPointsPerFullScan;
   private OusterDepthExtractionKernel depthExtractionKernel;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private IntPointer compressionParameters;
   private ByteBuffer pngImageBuffer;
   private BytePointer pngImageBytePointer;
   private long sequenceNumber = 0;

   private final ImageMessage outputImageMessage = new ImageMessage();
   private final LidarScanMessage lidarScanMessage = new LidarScanMessage();

   public OusterDriverAndDepthPublisher(Supplier<ReferenceFrame> sensorFrameUpdater, ROS2Topic<?>... outputTopics)
   {
      this.sensorFrameUpdater = sensorFrameUpdater;
      this.outputTopics = outputTopics;

      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ouster = new NettyOuster();
      ouster.bind();

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_depth_image_node");

      for (ROS2Topic<?> outputTopic : outputTopics)
      {
         outputTopicsTypes.add(outputTopic.getType());
         LogTools.info("Publishing ROS 2 depth images: {}", outputTopic);
         publisherMap.put(outputTopic, ROS2Tools.createPublisher(realtimeROS2Node, outputTopic, ROS2QosProfile.BEST_EFFORT()));
      }
      LogTools.info("Spinning Realtime ROS 2 node");
      realtimeROS2Node.spin();

      extractCompressAndPublishThread = MissingThreadTools.newSingleThreadExecutor("CopyAndPublish", true, 1);
      // Using incoming Ouster UDP Netty events as the thread scheduler. Only called on last datagram of frame.
      ouster.setOnFrameReceived(this::onFrameReceived);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         ouster.setOnFrameReceived(null);
         ouster.destroy();
         ThreadTools.sleepSeconds(0.5);
         extractCompressAndPublishThread.destroy();
      }, getClass().getSimpleName() + "Shutdown"));
   }

   // If we aren't doing anything, copy the data and publish it.
   private synchronized void onFrameReceived()
   {
      if (nativesLoadedActivator.poll())
      {
         if (nativesLoadedActivator.isNewlyActivated())
         {
            openCLManager = new OpenCLManager();
            openCLManager.create();
         }

         if (depthExtractionKernel == null)
         {
            LogTools.info("Ouster has been initialized.");
            depthWidth = ouster.getImageWidth();
            depthHeight = ouster.getImageHeight();
            numberOfPointsPerFullScan = depthWidth * depthHeight;
            LogTools.info("Ouster width: {} height: {} # points: {}", depthWidth, depthHeight, numberOfPointsPerFullScan);
            depthExtractionKernel = new OusterDepthExtractionKernel(ouster, openCLManager, outputTopicsTypes);
            compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_PNG_COMPRESSION, 1);
            pngImageBuffer = NativeMemoryTools.allocate(depthWidth * depthHeight * 2);
            pngImageBytePointer = new BytePointer(pngImageBuffer);
         }

         // Fast memcopy while the ouster thread is blocked
         depthExtractionKernel.copyLidarFrameBuffer();
         extractCompressAndPublishThread.clearQueueAndExecute(this::extractCompressAndPublish);
      }
   }

   /**
    * Synchronized to make sure it's only running ever once at a time.
    * This should also be guaranteed by the ResettableExceptionHandlingExecutorService.
    */
   private synchronized void extractCompressAndPublish()
   {
      for (ROS2Topic<?> topic : outputTopics)
      {
         if (topic.getType().equals(ImageMessage.class))
         {
            // Important not to store as a field, as update() needs to be called each frame
            ReferenceFrame cameraFrame = sensorFrameUpdater.get();
            cameraPose.setToZero(cameraFrame);
            cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

            depthExtractionKernel.runKernel(cameraPose);
            // Encode as PNG which is lossless and handles single channel images.
            opencv_imgcodecs.imencode(".png", depthExtractionKernel.getExtractedDepthImage().getBytedecoOpenCVMat(), pngImageBytePointer, compressionParameters);

            outputImageMessage.getPosition().set(cameraPose.getPosition());
            outputImageMessage.getOrientation().set(cameraPose.getOrientation());
            MessageTools.toMessage(ouster.getAquisitionInstant(), outputImageMessage.getAcquisitionTime());
            // Sadly, Pub Sub makes us go through a TByteArrayList. If we rewrite our DDS layer, we should allow a memcpy to native DDS buffer.
            outputImageMessage.getData().resetQuick();
            for (int i = 0; i < pngImageBytePointer.limit(); i++)
            {
               outputImageMessage.getData().add(pngImageBytePointer.get(i));
            }
            outputImageMessage.setFormat(OpenCVImageFormat.PNG.ordinal());
            outputImageMessage.setSequenceNumber(sequenceNumber++);
            outputImageMessage.setImageWidth(depthWidth);
            outputImageMessage.setImageHeight(depthHeight);

            publisherMap.get(topic).publish(outputImageMessage);
         }
         else if (topic.getType().equals(LidarScanMessage.class))
         {
            lidarScanMessage.setUniqueId(sequenceNumber);
            lidarScanMessage.getLidarPosition().set(cameraPose.getPosition());
            lidarScanMessage.getLidarOrientation().set(cameraPose.getOrientation());
            lidarScanMessage.setRobotTimestamp(Conversions.secondsToNanoseconds(ouster.getAquisitionInstant().getEpochSecond()) + ouster.getAquisitionInstant().getNano());
            lidarScanMessage.getScan().reset();
            //            compressedPointCloudBuffer.rewind();
            LidarPointCloudCompression.compressPointCloud(numberOfPointsPerFullScan, lidarScanMessage, (i, j) -> depthExtractionKernel.getPointCloudInSensorFrame().get(3 * i + j));

            publisherMap.get(topic).publish(lidarScanMessage);
         }
         else
         {
            throw new RuntimeException("We don't have the ability to publish this type of message");
         }
      }
   }

   public static void main(String[] args)
   {
      new OusterDriverAndDepthPublisher(ReferenceFrame::getWorldFrame, ROS2Tools.OUSTER_DEPTH_IMAGE);
   }
}
