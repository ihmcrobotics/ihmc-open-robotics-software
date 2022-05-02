package us.ihmc.avatar.gpuPlanarRegions;

import boofcv.struct.calib.CameraPinholeBrown;
import controller_msgs.msg.dds.StoredPropertySetMessage;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.Time;
import sensor_msgs.Image;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.property.StoredPropertySetMessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.realsense.BytedecoRealsenseL515;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.utilities.ros.ROS1Helper;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.publisher.RosCameraInfoPublisher;
import us.ihmc.utilities.ros.publisher.RosImagePublisher;

import java.util.function.Consumer;

public class L515AndGPUPlanarRegionsOnRobotProcess
{
   private static final String SERIAL_NUMBER = System.getProperty("l515.serial.number", "F0000000");

   private final PausablePeriodicThread thread;
   private final Activator nativesLoadedActivator;
   private final ROS1Helper ros1Helper;
   private RosImagePublisher ros1DepthPublisher;
   private RosCameraInfoPublisher ros1DepthCameraInfoPublisher;
   private ChannelBuffer ros1DepthChannelBuffer;
   private final ROS2Helper ros2Helper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2Input<StoredPropertySetMessage> parametersSubscription;
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsenseL515 l515;
   private Mat depthU16C1Image;
   private BytedecoImage depth32FC1Image;
   private int depthWidth;
   private int depthHeight;
   private CameraPinholeBrown depthCameraIntrinsics;
   private GPUPlanarRegionExtraction gpuPlanarRegionExtraction = new GPUPlanarRegionExtraction();
   private Runnable doNothingRunnable = () -> { };
   private Consumer<GPURegionRing> doNothingRingConsumer = ring -> { };
   private Consumer<GPUPlanarRegionIsland> doNothingIslandConsumer = island -> { };

   public L515AndGPUPlanarRegionsOnRobotProcess(DRCRobotModel robotModel)
   {
      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ros1Helper = new ROS1Helper("l515_node");

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_node");
      ros2Helper = new ROS2ControllerHelper(ros2Node, robotModel);
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);

      parametersSubscription = ros2Helper.subscribe(GPUPlanarRegionExtractionComms.PARAMETERS);

      thread = new PausablePeriodicThread("L515Node", UnitConversions.hertzToSeconds(31.0), false, this::update);
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "L515Shutdown"));
      thread.start();
   }

   private void update()
   {
      if (nativesLoadedActivator.poll())
      {
         if (nativesLoadedActivator.isNewlyActivated())
         {
            realSenseHardwareManager = new RealSenseHardwareManager();
            l515 = realSenseHardwareManager.createFullFeaturedL515(SERIAL_NUMBER);
            if (l515.getDevice() == null)
            {
               thread.stop();
               throw new RuntimeException("Device not found. Set -Dl515.serial.number=F0000000");
            }
            l515.initialize();

            depthWidth = l515.getDepthWidth();
            depthHeight = l515.getDepthHeight();

            String ros1DepthImageTopic = RosTools.L515_DEPTH;
            String ros1DepthCameraInfoTopic = RosTools.L515_DEPTH_CAMERA_INFO;
            LogTools.info("Publishing ROS 1 depth: {} {}", ros1DepthImageTopic, ros1DepthCameraInfoTopic);
            ros1DepthPublisher = new RosImagePublisher();
            ros1DepthCameraInfoPublisher = new RosCameraInfoPublisher();
            ros1Helper.attachPublisher(ros1DepthCameraInfoTopic, ros1DepthCameraInfoPublisher);
            ros1Helper.attachPublisher(ros1DepthImageTopic, ros1DepthPublisher);
            ros1DepthChannelBuffer = ros1DepthPublisher.getChannelBufferFactory().getBuffer(2 * depthWidth * depthHeight);

            depthCameraIntrinsics = new CameraPinholeBrown();
         }

         if (l515.readFrameData())
         {
            double dataAquisitionTime = Conversions.nanosecondsToSeconds(System.nanoTime());

            l515.updateDataBytePointers();

            if (depthU16C1Image == null)
            {
               MutableBytePointer depthFrameData = l515.getDepthFrameData();
               depthU16C1Image = new Mat(depthHeight, depthWidth, opencv_core.CV_16UC1, depthFrameData);
               depth32FC1Image = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_32FC1);

               depthCameraIntrinsics.setFx(l515.getFocalLengthPixelsX());
               depthCameraIntrinsics.setFy(l515.getFocalLengthPixelsY());
               depthCameraIntrinsics.setSkew(0.0);
               depthCameraIntrinsics.setCx(l515.getPrincipalOffsetXPixels());
               depthCameraIntrinsics.setCy(l515.getPrincipalOffsetYPixels());

               gpuPlanarRegionExtraction = new GPUPlanarRegionExtraction();
               gpuPlanarRegionExtraction.create(depthWidth,
                                                depthHeight,
                                                depth32FC1Image.getBackingDirectByteBuffer(),
                                                depthCameraIntrinsics.fx,
                                                depthCameraIntrinsics.fy,
                                                depthCameraIntrinsics.cx,
                                                depthCameraIntrinsics.cy);
            }

            depthU16C1Image.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, l515.getDepthToMeterConversion(), 0.0);

            syncedRobot.update();
            ReferenceFrame cameraFrame =
                  syncedRobot.hasReceivedFirstMessage() ? syncedRobot.getReferenceFrames().getSteppingCameraFrame() : ReferenceFrame.getWorldFrame();
            // TODO: Wait for frame at time of data aquisition?

            if (parametersSubscription.getMessageNotification().poll())
            {
               StoredPropertySetMessageTools.copyToStoredPropertySet(parametersSubscription.getMessageNotification().read(),
                                                                     gpuPlanarRegionExtraction.getParameters(),
                                                                     () -> LogTools.info("Accepted new parameters."));
            }

            gpuPlanarRegionExtraction.extractPlanarRegions(cameraFrame, doNothingRunnable);
            gpuPlanarRegionExtraction.findRegions(doNothingIslandConsumer);
            gpuPlanarRegionExtraction.findBoundariesAndHoles(doNothingRingConsumer);
            gpuPlanarRegionExtraction.growRegionBoundaries();
            gpuPlanarRegionExtraction.computePlanarRegions(cameraFrame);
            PlanarRegionsList planarRegionsList = gpuPlanarRegionExtraction.getPlanarRegionsList();
            ros2Helper.publish(ROS2Tools.MAPSENSE_REGIONS, PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));

            int depthFrameDataSize = l515.getDepthFrameDataSize();
            BytePointer dataPointer = depthU16C1Image.ptr();

//            VideoPacket videoPacket = new VideoPacket();
//            videoPacket.setImageHeight(depthHeight);
//            videoPacket.setImageWidth(depthWidth);
//            for (int i = 0; i < depthFrameDataSize; i++)
//            {
//               videoPacket.getData().add(dataPointer.get(i));
//            }
//            ros2Helper.publish(ROS2Tools.L515_DEPTH, videoPacket);

            if (ros1DepthPublisher.isConnected() && ros1DepthCameraInfoPublisher.isConnected())
            {
               ros1DepthChannelBuffer.clear();
               for (int i = 0; i < depthFrameDataSize; i++)
               {
                  ros1DepthChannelBuffer.writeByte(dataPointer.get(i));
               }

               ros1DepthChannelBuffer.readerIndex(0);
               ros1DepthChannelBuffer.writerIndex(depthFrameDataSize);

               ros1DepthCameraInfoPublisher.publish("camera_depth_optical_frame", depthCameraIntrinsics, new Time(dataAquisitionTime));
               int bytesPerValue = 2;
               Image message = ros1DepthPublisher.createMessage(depthWidth, depthHeight, bytesPerValue, "16UC1", ros1DepthChannelBuffer);
               message.getHeader().setStamp(new Time(dataAquisitionTime));

               ros1DepthPublisher.publish(message);
            }
         }
      }
   }

   private void destroy()
   {
      gpuPlanarRegionExtraction.destroy();
      l515.deleteDevice();
      realSenseHardwareManager.deleteContext();
   }
}
