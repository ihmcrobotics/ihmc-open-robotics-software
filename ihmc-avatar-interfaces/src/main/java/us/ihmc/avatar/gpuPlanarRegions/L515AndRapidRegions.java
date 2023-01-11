package us.ihmc.avatar.gpuPlanarRegions;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.property.ROS2StoredPropertySet;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.*;
import us.ihmc.perception.rapidRegions.GPUPlanarRegionIsland;
import us.ihmc.perception.rapidRegions.GPURegionRing;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsCustomizer;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.terrain.GPUPlanarRegionExtractionComms;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.tools.thread.Throttler;

import java.time.Instant;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class L515AndRapidRegions
{
   private static final String SERIAL_NUMBER = System.getProperty("l515.serial.number", "F1121365");

   private final PausablePeriodicThread thread;
   private final Activator nativesLoadedActivator;
   private final RealtimeROS2Node realtimeROS2Node;

   private final BigVideoPacket depthImagePacket = new BigVideoPacket();
   private final BigVideoPacket debugExtractionImagePacket = new BigVideoPacket();
   private BytedecoImage debugExtractionImage;
   private final ROS2Helper ros2Helper;
   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private final Notification patchSizeChangedNotification;
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense l515;
   private Mat depthU16C1Image;
   private BytedecoImage depthBytedecoImage;
   private int depthWidth;
   private int depthHeight;
   private Mat depthNormalizedScaled;
   private Mat depthRGB;
   private Mat depthYUV420Image;
   private BytePointer depthJPEGImageBytePointer;
   private Mat debugYUV420Image;
   private Mat flippedDebugImage;
   private BytePointer debugJPEGImageBytePointer;
   private IntPointer compressionParameters;
   private CameraPinholeBrown depthCameraIntrinsics;
   private final RapidPlanarRegionsExtractor rapidRegionsExtractor;
   private final RapidPlanarRegionsCustomizer rapidRegionsCustomizer;

   private final OpenCLManager openCLManager;
   private _cl_program openCLProgram;

   private boolean publishTimestamped = false;

   public L515AndRapidRegions()
   {
      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_videopub");
      realtimeROS2Node.spin();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_node");
      ros2Helper = new ROS2Helper(ros2Node);

      openCLManager = new OpenCLManager();
      rapidRegionsExtractor = new RapidPlanarRegionsExtractor();
      rapidRegionsCustomizer = new RapidPlanarRegionsCustomizer();

      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
      ROS2StoredPropertySet<?> ros2GPURegionParameters = ros2PropertySetGroup.registerStoredPropertySet(GPUPlanarRegionExtractionComms.PARAMETERS,
                                                                                                        rapidRegionsExtractor.getParameters());
      patchSizeChangedNotification = ros2GPURegionParameters.getCommandInput()
                                                            .registerPropertyChangedNotification(GPUPlanarRegionExtractionParameters.patchSize);
      ros2PropertySetGroup.registerStoredPropertySet(GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS,
                                                     rapidRegionsCustomizer.getPolygonizerParameters());
      ros2PropertySetGroup.registerStoredPropertySet(GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS,
                                                     rapidRegionsCustomizer.getConcaveHullFactoryParameters());

      thread = new PausablePeriodicThread("L515Node", UnitConversions.hertzToSeconds(31.0), 1, false, this::update);
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "L515Shutdown"));
      LogTools.info("Starting loop.");
      thread.start();
   }

   private void update()
   {
      if (nativesLoadedActivator.poll())
      {
         if (nativesLoadedActivator.isNewlyActivated())
         {
            LogTools.info("Natives loaded.");

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

            compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 80);
            depthNormalizedScaled = new Mat(depthHeight, depthWidth, opencv_core.CV_32FC1);
            depthRGB = new Mat(depthHeight, depthWidth, opencv_core.CV_8UC3);
            depthYUV420Image = new Mat();
            depthJPEGImageBytePointer = new BytePointer();
            debugYUV420Image = new Mat();
            flippedDebugImage = new Mat();
            debugJPEGImageBytePointer = new BytePointer();

            depthCameraIntrinsics = new CameraPinholeBrown();
         }

         if (l515.readFrameData())
         {
            Instant now = Instant.now();
            double dataAquisitionTime = Conversions.nanosecondsToSeconds(System.nanoTime());

            l515.updateDataBytePointers();

            if (depthU16C1Image == null)
            {
               LogTools.info("Is now reading frames.");

               MutableBytePointer depthFrameData = l515.getDepthFrameData();
               depthU16C1Image = new Mat(depthHeight, depthWidth, opencv_core.CV_16UC1, depthFrameData);
               depthBytedecoImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);

               depthCameraIntrinsics.setFx(l515.getDepthFocalLengthPixelsX());
               depthCameraIntrinsics.setFy(l515.getDepthFocalLengthPixelsY());
               depthCameraIntrinsics.setSkew(0.0);
               depthCameraIntrinsics.setCx(l515.getDepthPrincipalOffsetXPixels());
               depthCameraIntrinsics.setCy(l515.getDepthPrincipalOffsetYPixels());

               this.openCLManager.create();
               openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");
               rapidRegionsExtractor.create(openCLManager, openCLProgram, depthWidth,
                                                depthHeight,
                                                depthCameraIntrinsics.fx,
                                                depthCameraIntrinsics.fy,
                                                depthCameraIntrinsics.cx,
                                                depthCameraIntrinsics.cy);

               onPatchSizeResized();

               LogTools.info("Initialized.");
            }

            depthU16C1Image.convertTo(depthBytedecoImage.getBytedecoOpenCVMat(), opencv_core.CV_16UC1, 1, 0);

            PlanarRegionsListWithPose planarRegionsListWithPose = new PlanarRegionsListWithPose();
            rapidRegionsExtractor.update(depthBytedecoImage, true);
            rapidRegionsCustomizer.createCustomPlanarRegionsList(rapidRegionsExtractor.getGPUPlanarRegions(), ReferenceFrame.getWorldFrame(), planarRegionsListWithPose);
            PlanarRegionsList planarRegionsList = planarRegionsListWithPose.getPlanarRegionsList();

            LogTools.info("Planar regions: {}", planarRegionsList.getNumberOfPlanarRegions());

            // Filter out regions that are colliding with the body
            PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);

            if (publishTimestamped)
            {
               TimestampedPlanarRegionsListMessage timestampedPlanarRegionsListMessage = new TimestampedPlanarRegionsListMessage();
               timestampedPlanarRegionsListMessage.getPlanarRegions().set(planarRegionsListMessage);
               timestampedPlanarRegionsListMessage.setLastUpdatedSecondsSinceEpoch(now.getEpochSecond());
               timestampedPlanarRegionsListMessage.setLastUpdatedAdditionalNanos(now.getNano());
               ros2Helper.publish(ROS2Tools.RAPID_REGIONS, timestampedPlanarRegionsListMessage);
            }
            else
            {
               ros2Helper.publish(ROS2Tools.MAPSENSE_REGIONS, planarRegionsListMessage);
            }
         }
      }
   }

   private void onPatchSizeResized()
   {
      int patchImageWidth = rapidRegionsExtractor.getPatchImageWidth();
      int patchImageHeight = rapidRegionsExtractor.getPatchImageHeight();
      debugExtractionImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_8UC4);
   }

   private void destroy()
   {
      realtimeROS2Node.destroy();
      rapidRegionsExtractor.destroy();
      l515.deleteDevice();
      realSenseHardwareManager.deleteContext();
   }

   public static void main(String[] args)
   {
      new L515AndRapidRegions();
   }
}
