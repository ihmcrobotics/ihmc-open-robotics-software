package us.ihmc.perception.ouster;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.*;
import us.ihmc.perception.memory.MemoryTools;
import us.ihmc.perception.netty.NettyOuster;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
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
   private final ROS2Helper ros2Helper;
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
   private OpenCLFloatBuffer parametersBuffer;
   private _cl_mem lidarFrameBufferObject;
   private BytedecoImage compressionInputImage;
   private ByteBuffer pngImageBuffer;
   private long sequenceNumber = 0;
   private final ImageMessage outputImageMessage = new ImageMessage();

   public OusterDepthImagePublisher(Supplier<ReferenceFrame> sensorFrameUpdater)
   {
      this.sensorFrameUpdater = sensorFrameUpdater;
      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_standalone_node");
      ros2Helper = new ROS2Helper(ros2Node);

      ouster = new NettyOuster();
      ouster.bind();

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
            compressionInputImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
            pngImageBuffer = ByteBuffer.allocateDirect(depthWidth * depthHeight * 2);
         }

         // copy while the ouster thread is blocked
         lidarFrameByteBufferPointer.position(0);
         lidarFrameByteBufferPointerCopy.position(0);
         MemoryTools.memoryCopy(lidarFrameByteBufferPointer, lidarFrameByteBufferPointerCopy);

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

         parametersBuffer = new OpenCLFloatBuffer(19);
         parametersBuffer.createOpenCLBufferObject(openCLManager);

         lidarFrameBufferObject = openCLManager.createBufferObject(lidarFrameByteBufferCopy.capacity(), lidarFrameByteBufferPointerCopy);
         compressionInputImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);
      }

      // Important not to store as a field, as update() needs to be called each frame
      ReferenceFrame cameraFrame = sensorFrameUpdater.get();
      cameraPose.setToZero(cameraFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      parametersBuffer.getBytedecoFloatBufferPointer().put(0, ouster.getColumnsPerFrame());
      parametersBuffer.getBytedecoFloatBufferPointer().put(1, ouster.getMeasurementBlockSize());
      parametersBuffer.getBytedecoFloatBufferPointer().put(2, NettyOuster.HEADER_BLOCK_BYTES);
      parametersBuffer.getBytedecoFloatBufferPointer().put(3, NettyOuster.CHANNEL_DATA_BLOCK_BYTES);
      parametersBuffer.writeOpenCLBufferObject(openCLManager);

      openCLManager.enqueueWriteBuffer(lidarFrameBufferObject, lidarFrameByteBufferCopy.capacity(), lidarFrameByteBufferPointerCopy);

      openCLManager.setKernelArgument(extractDepthImageKernel, 0, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(extractDepthImageKernel, 1, lidarFrameBufferObject);
      openCLManager.setKernelArgument(extractDepthImageKernel, 2, compressionInputImage.getOpenCLImageObject());
      openCLManager.execute2D(extractDepthImageKernel, depthWidth, depthHeight);
      compressionInputImage.readOpenCLImage(openCLManager);
      openCLManager.finish();

      pngImageBuffer.position(0);
      pngImageBuffer.limit(pngImageBuffer.capacity());
      opencv_imgcodecs.imencode(".png", compressionInputImage.getBytedecoOpenCVMat(), pngImageBuffer);

      outputImageMessage.getPosition().set(cameraPose.getPosition());
      outputImageMessage.getOrientation().set(cameraPose.getOrientation());
      outputImageMessage.setAcquisitionTimeSecondsSinceEpoch(ouster.getAquisitionInstant().getEpochSecond());
      outputImageMessage.setAcquisitionTimeAdditionalNanos(ouster.getAquisitionInstant().getNano());
      outputImageMessage.getData().resetQuick();
      for (int i = 0; i < pngImageBuffer.limit(); i++)
      {
         outputImageMessage.getData().add(pngImageBuffer.get(i));
      }
      outputImageMessage.setFormat(OpenCVImageFormat.PNG.ordinal());
      outputImageMessage.setSequenceNumber(sequenceNumber++);
      outputImageMessage.setImageWidth(depthWidth);
      outputImageMessage.setImageHeight(depthHeight);
      ros2Helper.publish(ROS2Tools.OUSTER_DEPTH_IMAGE, outputImageMessage);
   }

   public static void main(String[] args)
   {
      new OusterDepthImagePublisher(ReferenceFrame::getWorldFrame);
   }
}
