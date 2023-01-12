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
import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
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
import us.ihmc.utilities.ros.RosTools;

import java.nio.ByteBuffer;
import java.util.*;
import java.util.function.Supplier;

/**
 * This class publishes a PNG compressed depth image from the Ouster as fast as the frames come in.
 */
public class OusterProcess
{
   private static final double horizontalFieldOfView = Math.PI * 2.0;
   private static final double verticalFieldOfView = Math.PI / 2.0;

   public enum SensorDataTypes {DEPTH_IMAGE, LIDAR_SCAN_MESSAGE};

   private final Activator nativesLoadedActivator;
   private final RealtimeROS2Node realtimeROS2Node;
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
   private _cl_kernel imageToPointCloudKernel;
   private final OpenCLFloatParameters parametersBuffer = new OpenCLFloatParameters();
   private final OpenCLFloatParameters sensorValuesBuffer = new OpenCLFloatParameters();
   private OpenCLIntBuffer pointCloudXYZBuffer;
   private ByteBuffer compressedPointCloudBuffer;
   private OpenCLIntBuffer pixelShiftOpenCLBuffer;
   private _cl_mem lidarFrameBufferObject;
   private BytedecoImage compressionInputImage;
   private IntPointer compressionParameters;
   private ByteBuffer pngImageBuffer;
   private BytePointer pngImageBytePointer;
   private long sequenceNumber = 0;
   private final ImageMessage outputImageMessage = new ImageMessage();
   private final LidarScanMessage lidarScanMessage = new LidarScanMessage();

   private final Collection<ROS2Topic<?>> outputTopics;
   private final List<Class<?>> outputTopicsTypes = new ArrayList<>();
   private final HashMap<ROS2Topic<?>, IHMCRealtimeROS2Publisher> publisherMap = new HashMap<>();

   public OusterProcess(Supplier<ReferenceFrame> sensorFrameUpdater, Collection<ROS2Topic<?>> outputTopics)
   {
      this.sensorFrameUpdater = sensorFrameUpdater;
      this.outputTopics = outputTopics;

      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ouster = new NettyOuster();
      ouster.bind();

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "ouster_depth_image_publisher");

      for (ROS2Topic<?> outputTopic : outputTopics)
      {
         outputTopicsTypes.add(outputTopic.getType());
         LogTools.info("Publishing ROS 2 depth images: {}", outputTopic);
         publisherMap.put(outputTopic, ROS2Tools.createPublisher(realtimeROS2Node, outputTopic, ROS2QosProfile.BEST_EFFORT()));
      }
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
            pointCloudXYZBuffer = new OpenCLIntBuffer(3 * depthHeight * depthWidth);
         }

         // copy while the ouster thread is blocked
         lidarFrameByteBufferPointer.position(0);
         lidarFrameByteBufferPointerCopy.position(0);
         NativeMemoryTools.copy(lidarFrameByteBufferPointer, lidarFrameByteBufferPointerCopy);

         extractCompressAndPublishThread.clearQueueAndExecute(this::extractCompressAndPublish);
      }
   }

   private void populateSensorValuesBuffer(RigidBodyTransformReadOnly sensorTransformToWorld)
   {
      sensorValuesBuffer.setParameter((float) horizontalFieldOfView);
      sensorValuesBuffer.setParameter((float) verticalFieldOfView);
      sensorValuesBuffer.setParameter(sensorTransformToWorld.getTranslation().getX32());
      sensorValuesBuffer.setParameter(sensorTransformToWorld.getTranslation().getY32());
      sensorValuesBuffer.setParameter(sensorTransformToWorld.getTranslation().getZ32());
      RotationMatrix rotationMatrix = new RotationMatrix(sensorTransformToWorld.getRotation());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM00());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM01());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM02());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM10());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM11());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM12());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM20());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM21());
      sensorValuesBuffer.setParameter((float) rotationMatrix.getM22());
      sensorValuesBuffer.setParameter(depthWidth);
      sensorValuesBuffer.setParameter(depthHeight);
      sensorValuesBuffer.setParameter((float) LidarPointCloudCompression.POINT_RESOLUTION);

      sensorValuesBuffer.writeOpenCLBufferObject(openCLManager);
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
         imageToPointCloudKernel = openCLManager.createKernel(openCLProgram, "imageToPointCloud");

         lidarFrameBufferObject = openCLManager.createBufferObject(lidarFrameByteBufferCopy.capacity(), lidarFrameByteBufferPointerCopy);
         compressionInputImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         pixelShiftOpenCLBuffer = new OpenCLIntBuffer(ouster.getPixelShiftBuffer());
         pixelShiftOpenCLBuffer.createOpenCLBufferObject(openCLManager);
         pointCloudXYZBuffer.createOpenCLBufferObject(openCLManager);
         compressedPointCloudBuffer = ByteBuffer.allocate(depthWidth * depthHeight * Integer.BYTES * 3);
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

      if (outputTopicsTypes.contains(LidarScanMessage.class))
      {
         populateSensorValuesBuffer(cameraPose);
         openCLManager.setKernelArgument(imageToPointCloudKernel, 0, sensorValuesBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(imageToPointCloudKernel, 1, compressionInputImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(imageToPointCloudKernel, 2, pointCloudXYZBuffer.getOpenCLBufferObject());
         openCLManager.execute2D(imageToPointCloudKernel, depthWidth, depthHeight);
         pointCloudXYZBuffer.readOpenCLBufferObject(openCLManager);
      }

      openCLManager.finish();


      for (ROS2Topic<?> topic : outputTopics)
      {
         if (topic.getType().equals(ImageMessage.class))
         {
            opencv_imgcodecs.imencode(".png", compressionInputImage.getBytedecoOpenCVMat(), pngImageBytePointer, compressionParameters);

            outputImageMessage.getPosition().set(cameraPose.getPosition());
            outputImageMessage.getOrientation().set(cameraPose.getOrientation());
            MessageTools.toMessage(ouster.getAquisitionInstant(), outputImageMessage.getAcquisitionTime());
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
            lidarScanMessage.getLidarPosition().set(cameraPose.getPosition());
            lidarScanMessage.getLidarOrientation().set(cameraPose.getOrientation());
            lidarScanMessage.setRobotTimestamp(Conversions.secondsToNanoseconds(ouster.getAquisitionInstant().getEpochSecond()) + ouster.getAquisitionInstant().getNano());
            LidarPointCloudCompression.compressPointCloud(depthHeight * depthWidth,
                                                          lidarScanMessage,
                                                          pointCloudXYZBuffer.getOpenCLBufferObject().asByteBuffer(),
                                                          compressedPointCloudBuffer);

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
      new OusterProcess(ReferenceFrame::getWorldFrame, Arrays.asList(ROS2Tools.OUSTER_DEPTH_IMAGE));
   }
}
