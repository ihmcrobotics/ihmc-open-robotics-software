package us.ihmc.avatar.gpuPlanarRegions;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.BigVideoPacket;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage;
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
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.rapidRegions.GPUPlanarRegionIsland;
import us.ihmc.perception.rapidRegions.GPURegionRing;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.perception.terrain.GPUPlanarRegionExtractionComms;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegionsList;
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

public class L515AndRapidRegions
{
   private static final String SERIAL_NUMBER = System.getProperty("l515.serial.number", "F0245563");

   private final PausablePeriodicThread thread;
   private final Activator nativesLoadedActivator;
   private final RealtimeROS2Node realtimeROS2Node;
   private final IHMCRealtimeROS2Publisher<BigVideoPacket> ros2DepthVideoPublisher;
   private final IHMCRealtimeROS2Publisher<BigVideoPacket> ros2DebugExtractionVideoPublisher;

   private final BigVideoPacket depthImagePacket = new BigVideoPacket();
   private final BigVideoPacket debugExtractionImagePacket = new BigVideoPacket();
   private BytedecoImage debugExtractionImage;
   private final ROS2Helper ros2Helper;
   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private final Notification patchSizeChangedNotification;
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense l515;
   private Mat depthU16C1Image;
   private BytedecoImage depth32FC1Image;
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
   private final GPUPlanarRegionExtraction gpuPlanarRegionExtraction;
   private final Runnable onPatchSizeResized = this::onPatchSizeResized;
   private final Consumer<GPUPlanarRegionIsland> doNothingIslandConsumer = this::onFindRegionIsland;
   private final Consumer<GPURegionRing> doNothingRingConsumer = this::onFindBoundariesAndHolesRing;
   private final Throttler parameterOutputThrottler = new Throttler();
   private final Mat BLACK_OPAQUE_RGBA8888 = new Mat((byte) 0, (byte) 0, (byte) 0, (byte) 255);

   private boolean publishTimestamped = false;

   public L515AndRapidRegions()
   {
      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_videopub");

      ROS2Topic<BigVideoPacket> depthTopic = ROS2Tools.L515_DEPTH_LARGE;
      LogTools.info("Publishing ROS 2 depth video: {}", depthTopic);
      ros2DepthVideoPublisher = ROS2Tools.createPublisher(realtimeROS2Node, depthTopic, ROS2QosProfile.BEST_EFFORT());
      ROS2Topic<BigVideoPacket> debugExtractionTopic = ROS2Tools.L515_DEBUG_EXTRACTION;
      LogTools.info("Publishing ROS 2 debug extraction video: {}", debugExtractionTopic);
      ros2DebugExtractionVideoPublisher = ROS2Tools.createPublisher(realtimeROS2Node, debugExtractionTopic, ROS2QosProfile.BEST_EFFORT());
      realtimeROS2Node.spin();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_node");
      ros2Helper = new ROS2Helper(ros2Node);

      gpuPlanarRegionExtraction = new GPUPlanarRegionExtraction();
      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
      ROS2StoredPropertySet<?> ros2GPURegionParameters = ros2PropertySetGroup.registerStoredPropertySet(GPUPlanarRegionExtractionComms.PARAMETERS,
                                                                                                        gpuPlanarRegionExtraction.getParameters());
      patchSizeChangedNotification = ros2GPURegionParameters.getCommandInput()
                                                            .registerPropertyChangedNotification(GPUPlanarRegionExtractionParameters.patchSize);
      ros2PropertySetGroup.registerStoredPropertySet(GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS,
                                                     gpuPlanarRegionExtraction.getPolygonizerParameters());
      ros2PropertySetGroup.registerStoredPropertySet(GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS,
                                                     gpuPlanarRegionExtraction.getConcaveHullFactoryParameters());

      thread = new PausablePeriodicThread("L515Node", UnitConversions.hertzToSeconds(31.0), 1, false, this::update);
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "L515Shutdown"));
      LogTools.info("Starting loop.");
      thread.start();
   }

   int val0;
   int val1;
   int val2;
   int val3;

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
               depth32FC1Image = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_32FC1);

               depthCameraIntrinsics.setFx(l515.getDepthFocalLengthPixelsX());
               depthCameraIntrinsics.setFy(l515.getDepthFocalLengthPixelsY());
               depthCameraIntrinsics.setSkew(0.0);
               depthCameraIntrinsics.setCx(l515.getDepthPrincipalOffsetXPixels());
               depthCameraIntrinsics.setCy(l515.getDepthPrincipalOffsetYPixels());

               gpuPlanarRegionExtraction.create(depthWidth,
                                                depthHeight,
                                                depth32FC1Image.getBackingDirectByteBuffer(),
                                                depthCameraIntrinsics.fx,
                                                depthCameraIntrinsics.fy,
                                                depthCameraIntrinsics.cx,
                                                depthCameraIntrinsics.cy);

               onPatchSizeResized();

               LogTools.info("Initialized.");
            }

            val0 = Short.toUnsignedInt(depthU16C1Image.ptr(0, 0).getShort());
            val1 = Short.toUnsignedInt(depthU16C1Image.ptr(100, 200).getShort());
            val2 = Short.toUnsignedInt(depthU16C1Image.ptr(400, 200).getShort());
            val3 = Short.toUnsignedInt(depthU16C1Image.ptr(600, 50).getShort());

            depthU16C1Image.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, l515.getDepthToMeterConversion(), 0.0);

            int previousPatchSize = gpuPlanarRegionExtraction.getParameters().getPatchSize();
            ros2PropertySetGroup.update();
            if (patchSizeChangedNotification.poll())
            {
               int newPatchSize = gpuPlanarRegionExtraction.getParameters().getPatchSize();
               if (depthWidth % newPatchSize == 0 && depthHeight % newPatchSize == 0)
               {
                  LogTools.info("New patch size accepted: {}", newPatchSize);
                  gpuPlanarRegionExtraction.setPatchSizeChanged(true);
               }
               else // Reject the parameter and revert to the previous value because it's not an even divisor
               {
                  gpuPlanarRegionExtraction.getParameters().setPatchSize(previousPatchSize);
               }
            }

            gpuPlanarRegionExtraction.readFromSourceImage();
            gpuPlanarRegionExtraction.extractPlanarRegions(onPatchSizeResized);

            debugExtractionImage.getBytedecoOpenCVMat().setTo(BLACK_OPAQUE_RGBA8888);

            gpuPlanarRegionExtraction.findRegions(doNothingIslandConsumer);
            gpuPlanarRegionExtraction.findBoundariesAndHoles(doNothingRingConsumer);
            gpuPlanarRegionExtraction.growRegionBoundaries();
            gpuPlanarRegionExtraction.computePlanarRegions(ReferenceFrame.getWorldFrame());
            PlanarRegionsList planarRegionsList = gpuPlanarRegionExtraction.getPlanarRegionsList();

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

            int depthFrameDataSize = l515.getDepthFrameDataSize();

            depth32FC1Image.rewind();
            BytedecoOpenCVTools.clampTo8BitUnsignedChar(depth32FC1Image.getBytedecoOpenCVMat(), depthNormalizedScaled, 0.0, 255.0);
            BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(depthNormalizedScaled, depthRGB);
            opencv_imgproc.cvtColor(depthRGB, depthYUV420Image, opencv_imgproc.COLOR_RGB2YUV_I420);
            opencv_imgcodecs.imencode(".jpg", depthYUV420Image, depthJPEGImageBytePointer, compressionParameters);

            byte[] heapByteArrayData = new byte[depthJPEGImageBytePointer.asBuffer().remaining()];
            depthJPEGImageBytePointer.asBuffer().get(heapByteArrayData);
            depthImagePacket.getData().resetQuick();
            depthImagePacket.getData().add(heapByteArrayData);
            depthImagePacket.setImageHeight(depthHeight);
            depthImagePacket.setImageWidth(depthWidth);
            depthImagePacket.setAcquisitionTimeSecondsSinceEpoch(now.getEpochSecond());
            depthImagePacket.setAcquisitionTimeAdditionalNanos(now.getNano());
            ros2DepthVideoPublisher.publish(depthImagePacket);

            BytedecoOpenCVTools.flipY(debugExtractionImage.getBytedecoOpenCVMat(), flippedDebugImage);
            opencv_imgproc.cvtColor(flippedDebugImage, debugYUV420Image, opencv_imgproc.COLOR_RGB2YUV_I420);
            opencv_imgcodecs.imencode(".jpg", debugYUV420Image, debugJPEGImageBytePointer, compressionParameters);

            heapByteArrayData = new byte[debugJPEGImageBytePointer.asBuffer().remaining()];
            debugJPEGImageBytePointer.asBuffer().get(heapByteArrayData);
            debugExtractionImagePacket.getData().resetQuick();
            debugExtractionImagePacket.getData().add(heapByteArrayData);
            debugExtractionImagePacket.setImageHeight(debugExtractionImage.getImageHeight());
            debugExtractionImagePacket.setImageWidth(debugExtractionImage.getImageWidth());
            debugExtractionImagePacket.setAcquisitionTimeSecondsSinceEpoch(now.getEpochSecond());
            debugExtractionImagePacket.setAcquisitionTimeAdditionalNanos(now.getNano());
            ros2DebugExtractionVideoPublisher.publish(debugExtractionImagePacket);
         }
      }
   }

   private void onPatchSizeResized()
   {
      int patchImageWidth = gpuPlanarRegionExtraction.getPatchImageWidth();
      int patchImageHeight = gpuPlanarRegionExtraction.getPatchImageHeight();
      debugExtractionImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_8UC4);
   }

   private void onFindRegionIsland(GPUPlanarRegionIsland island)
   {
      for (Point2D regionIndex : island.planarRegion.getRegionIndices())
      {
         int x = (int) regionIndex.getX();
         int y = (int) regionIndex.getY();
         int r = (island.planarRegionIslandIndex + 1) * 312 % 255;
         int g = (island.planarRegionIslandIndex + 1) * 123 % 255;
         int b = (island.planarRegionIslandIndex + 1) * 231 % 255;
         BytePointer pixel = debugExtractionImage.getBytedecoOpenCVMat().ptr(y, x);
         pixel.put(0, (byte) r);
         pixel.put(1, (byte) g);
         pixel.put(2, (byte) b);
      }
   }

   private void onFindBoundariesAndHolesRing(GPURegionRing regionRing)
   {
      for (Vector2D boundaryIndex : regionRing.getBoundaryIndices())
      {
         int x = (int) boundaryIndex.getX();
         int y = (int) boundaryIndex.getY();
         int r = (regionRing.getIndex() + 1) * 130 % 255;
         int g = (regionRing.getIndex() + 1) * 227 % 255;
         int b = (regionRing.getIndex() + 1) * 332 % 255;
         BytePointer pixel = debugExtractionImage.getBytedecoOpenCVMat().ptr(y, x);
         pixel.put(0, (byte) r);
         pixel.put(1, (byte) g);
         pixel.put(2, (byte) b);
      }
   }

   private void destroy()
   {
      realtimeROS2Node.destroy();
      gpuPlanarRegionExtraction.destroy();
      l515.deleteDevice();
      realSenseHardwareManager.deleteContext();
   }

   public static void main(String[] args)
   {
      new L515AndRapidRegions();
   }
}
