package us.ihmc.perception.ouster;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.*;
import us.ihmc.perception.memory.NativeMemoryTools;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.ByteBuffer;
import java.util.function.Supplier;

/**
 * This class publishes a PNG compressed depth image from the Ouster as fast as the frames come in.
 */
public class OusterDepthImagePublisher
{
   private final Activator nativesLoadedActivator;
   private final RealtimeROS2Node realtimeROS2Node;
   private final IHMCRealtimeROS2Publisher<ImageMessage> ros2DepthImagePublisher;
   private Supplier<ReferenceFrame> sensorFrameUpdater;
   private final FramePose3D cameraPose = new FramePose3D();
   private final ResettableExceptionHandlingExecutorService extractCompressAndPublishThread;
   private NettyOuster ouster;
   private int depthWidth;
   private int depthHeight;
   private int numberOfPointsPerFullScan;
   private ByteBuffer lidarFrameByteBufferCopy;
   private BytePointer lidarFrameByteBufferPointerCopy;
   private BytePointer lidarFrameByteBufferPointer;
   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;
   private _cl_kernel extractDepthImageKernel;
   private final OpenCLFloatParameters parametersBuffer = new OpenCLFloatParameters();
   private OpenCLIntBuffer pixelShiftOpenCLBuffer;
   private _cl_mem lidarFrameBufferObject;
   private BytedecoImage compressionInputImage;
   private IntPointer compressionParameters;
   private ByteBuffer pngImageBuffer;
   private BytePointer pngImageBytePointer;
   private long sequenceNumber = 0;
   private final ImageMessage outputImageMessage = new ImageMessage();

   public OusterDepthImagePublisher(Supplier<ReferenceFrame> sensorFrameUpdater)
   {
      this.sensorFrameUpdater = sensorFrameUpdater;
      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ouster = new NettyOuster();
      ouster.bind();

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_depth_image_publisher");
      ROS2Topic<ImageMessage> topic = ROS2Tools.OUSTER_DEPTH_IMAGE;
      LogTools.info("Publishing ROS 2 depth images: {}", topic);
      ros2DepthImagePublisher = ROS2Tools.createPublisher(realtimeROS2Node, topic, ROS2QosProfile.BEST_EFFORT());
      LogTools.info("Spinning Realtime ROS 2 node");
      realtimeROS2Node.spin();

      extractCompressAndPublishThread = MissingThreadTools.newSingleThreadExecutor("CopyAndPublish", true, 1);
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

         if (lidarFrameByteBufferCopy == null)
         {
            LogTools.info("Ouster has been initialized.");
            depthWidth = ouster.getImageWidth();
            depthHeight = ouster.getImageHeight();
            numberOfPointsPerFullScan = depthWidth * depthHeight;
            LogTools.info("Ouster width: {} height: {} # points: {}", depthWidth, depthHeight, numberOfPointsPerFullScan);
            lidarFrameByteBufferCopy = ByteBuffer.allocateDirect(ouster.getLidarFrameByteBuffer().limit());
            lidarFrameByteBufferPointerCopy = new BytePointer(lidarFrameByteBufferCopy);
            lidarFrameByteBufferPointer = new BytePointer(ouster.getLidarFrameByteBuffer());
            compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_PNG_COMPRESSION, 1);
            compressionInputImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
            pngImageBuffer = NativeMemoryTools.allocate(depthWidth * depthHeight * 2);
            pngImageBytePointer = new BytePointer(pngImageBuffer);
         }

         // copy while the ouster thread is blocked
         lidarFrameByteBufferPointer.position(0);
         lidarFrameByteBufferPointerCopy.position(0);
         NativeMemoryTools.copy(lidarFrameByteBufferPointer, lidarFrameByteBufferPointerCopy);

         lidarFrameByteBufferCopy.put(ouster.getLidarFrameByteBuffer());

         extractCompressAndPublishThread.clearQueueAndExecute(this::extractCompressAndPublish);
      }
   }

   /**
    * Synchronized to make sure it's only running ever once at a time.
    * This should also be guaranteed by the ResettableExceptionHandlingExecutorService.
    */
   private synchronized void extractCompressAndPublish()
   {
      if (openCLProgram == null)
      {
         openCLProgram = openCLManager.loadProgram(getClass().getSimpleName());
         extractDepthImageKernel = openCLManager.createKernel(openCLProgram, "extractDepthImage");

         lidarFrameBufferObject = openCLManager.createBufferObject(lidarFrameByteBufferCopy.capacity(), lidarFrameByteBufferPointerCopy);
         compressionInputImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);
         pixelShiftOpenCLBuffer = new OpenCLIntBuffer(ouster.getPixelShiftBuffer());
         pixelShiftOpenCLBuffer.createOpenCLBufferObject(openCLManager);
      }

      // Important not to store as a field, as update() needs to be called each frame
      ReferenceFrame cameraFrame = sensorFrameUpdater.get();
      cameraPose.setToZero(cameraFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      parametersBuffer.setParameter(ouster.getColumnsPerFrame());
      parametersBuffer.setParameter(ouster.getMeasurementBlockSize());
      parametersBuffer.setParameter(NettyOuster.HEADER_BLOCK_BYTES);
      parametersBuffer.setParameter(NettyOuster.CHANNEL_DATA_BLOCK_BYTES);
      parametersBuffer.setParameter(NettyOuster.MEASUREMENT_BLOCKS_PER_UDP_DATAGRAM);
      parametersBuffer.writeOpenCLBufferObject(openCLManager);

      pixelShiftOpenCLBuffer.writeOpenCLBufferObject(openCLManager);
      openCLManager.enqueueWriteBuffer(lidarFrameBufferObject, lidarFrameByteBufferCopy.capacity(), lidarFrameByteBufferPointerCopy);

      openCLManager.setKernelArgument(extractDepthImageKernel, 0, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(extractDepthImageKernel, 1, pixelShiftOpenCLBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(extractDepthImageKernel, 2, lidarFrameBufferObject);
      openCLManager.setKernelArgument(extractDepthImageKernel, 3, compressionInputImage.getOpenCLImageObject());
      openCLManager.execute2D(extractDepthImageKernel, depthWidth, depthHeight);
      compressionInputImage.readOpenCLImage(openCLManager);
      openCLManager.finish();

      opencv_imgcodecs.imencode(".png", compressionInputImage.getBytedecoOpenCVMat(), pngImageBytePointer, compressionParameters);

      outputImageMessage.getPosition().set(cameraPose.getPosition());
      outputImageMessage.getOrientation().set(cameraPose.getOrientation());
      outputImageMessage.setAcquisitionTimeSecondsSinceEpoch(ouster.getAquisitionInstant().getEpochSecond());
      outputImageMessage.setAcquisitionTimeAdditionalNanos(ouster.getAquisitionInstant().getNano());
      outputImageMessage.getData().resetQuick();
      for (int i = 0; i < pngImageBytePointer.limit(); i++)
      {
         outputImageMessage.getData().add(pngImageBytePointer.get(i));
      }
      outputImageMessage.setFormat(OpenCVImageFormat.PNG.ordinal());
      outputImageMessage.setSequenceNumber(sequenceNumber++);
      outputImageMessage.setImageWidth(depthWidth);
      outputImageMessage.setImageHeight(depthHeight);
      ros2DepthImagePublisher.publish(outputImageMessage);
   }

   public static void main(String[] args)
   {
      new OusterDepthImagePublisher(ReferenceFrame::getWorldFrame);
   }
}
