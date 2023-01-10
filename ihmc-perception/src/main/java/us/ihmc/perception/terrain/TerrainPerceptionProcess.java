package us.ihmc.perception.terrain;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.ros.message.Time;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.TimestampedPlanarRegionsListMessage;
import sensor_msgs.Image;
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
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.ihmcPerception.depthData.CollisionShapeTester;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.rapidRegions.GPUPlanarRegionIsland;
import us.ihmc.perception.rapidRegions.GPURegionRing;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsCustomizer;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.tools.thread.Throttler;
import us.ihmc.utilities.ros.ROS1Helper;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.publisher.RosCameraInfoPublisher;
import us.ihmc.utilities.ros.publisher.RosImagePublisher;

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class TerrainPerceptionProcess
{
   private static final String SERIAL_NUMBER = System.getProperty("l515.serial.number", "F0245563");

   private final PausablePeriodicThread thread;
   private final Activator nativesLoadedActivator;
   private final RealtimeROS2Node realtimeROS2Node;
   private final IHMCRealtimeROS2Publisher<ImageMessage> depthPublisher;
   private final IHMCRealtimeROS2Publisher<ImageMessage> debugImagePublisher;

   private final RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private final RapidPlanarRegionsCustomizer rapidPlanarRegionsCustomizer;

   private final ImageMessage depthImagePacket = new ImageMessage();
   private final ImageMessage debugExtractionImagePacket = new ImageMessage();

   private final ROS2Helper ros2Helper;
   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private final Notification patchSizeChangedNotification;

   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense l515;

   private Mat depthU16C1Image;
   private BytedecoImage depth32FC1Image;
   private int depthWidth;
   private int depthHeight;

   private CameraPinholeBrown depthCameraIntrinsics;
   private final Runnable onPatchSizeResized = this::onPatchSizeResized;
   private final CollidingScanRegionFilter collisionFilter;

   private boolean publishTimestamped = false;

   public TerrainPerceptionProcess(Supplier<ReferenceFrame> sensorFrameUpdater)
   {
      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_videopub");

      ROS2Topic<ImageMessage> depthTopic = ROS2Tools.L515_DEPTH_IMAGE;
      LogTools.info("Publishing ROS 2 depth: {}", depthTopic);
      depthPublisher = ROS2Tools.createPublisher(realtimeROS2Node, depthTopic, ROS2QosProfile.BEST_EFFORT());
      ROS2Topic<ImageMessage> debugExtractionTopic = ROS2Tools.TERRAIN_DEBUG_IMAGE;
      LogTools.info("Publishing ROS 2 debug image: {}", debugExtractionTopic);
      debugImagePublisher = ROS2Tools.createPublisher(realtimeROS2Node, debugExtractionTopic, ROS2QosProfile.BEST_EFFORT());
      realtimeROS2Node.spin();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_node");
      ros2Helper = new ROS2Helper(ros2Node);

      rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor();
      rapidPlanarRegionsCustomizer = new RapidPlanarRegionsCustomizer();

      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
      ROS2StoredPropertySet<?> ros2GPURegionParameters = ros2PropertySetGroup.registerStoredPropertySet(GPUPlanarRegionExtractionComms.PARAMETERS,
                                                                                                        rapidPlanarRegionExtractor.getParameters());
      patchSizeChangedNotification = ros2GPURegionParameters.getCommandInput()
                                                            .registerPropertyChangedNotification(GPUPlanarRegionExtractionParameters.patchSize);
      ros2PropertySetGroup.registerStoredPropertySet(GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS,
                                                     rapidPlanarRegionExtractor.getPolygonizerParameters());
      ros2PropertySetGroup.registerStoredPropertySet(GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS,
                                                     rapidPlanarRegionExtractor.getConcaveHullFactoryParameters());
      ros2Helper.subscribeViaCallback(GPUPlanarRegionExtractionComms.RECONNECT_ROS1_NODE, reconnectROS1Notification::set);

      CollisionBoxProvider collisionBoxProvider = robotModel.getCollisionBoxProvider();
      CollisionShapeTester shapeTester = new CollisionShapeTester();
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      for (RobotSide robotSide : RobotSide.values)
      {
         List<JointBasics> joints = new ArrayList<>();
         RigidBodyBasics shin = fullRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getPredecessor();
         MultiBodySystemTools.collectJointPath(fullRobotModel.getPelvis(), shin, joints);
         joints.forEach(joint -> shapeTester.addJoint(collisionBoxProvider, joint));
      }
      collisionFilter = new CollidingScanRegionFilter(shapeTester);

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

               rapidPlanarRegionExtractor.create(depthWidth,
                                                depthHeight,
                                                depth32FC1Image.getBackingDirectByteBuffer(),
                                                depthCameraIntrinsics.fx,
                                                depthCameraIntrinsics.fy,
                                                depthCameraIntrinsics.cx,
                                                depthCameraIntrinsics.cy);

               if (enableROS1)
               {
                  ros1DebugExtractionImagePublisher = new RosImagePublisher();
                  ros1Helper.attachPublisher(GPUPlanarRegionExtractionComms.DEBUG_EXTRACTION_IMAGE, ros1DebugExtractionImagePublisher);
               }
               onPatchSizeResized();

               LogTools.info("Initialized.");
            }

            val0 = Short.toUnsignedInt(depthU16C1Image.ptr(0, 0).getShort());
            val1 = Short.toUnsignedInt(depthU16C1Image.ptr(100, 200).getShort());
            val2 = Short.toUnsignedInt(depthU16C1Image.ptr(400, 200).getShort());
            val3 = Short.toUnsignedInt(depthU16C1Image.ptr(600, 50).getShort());

            depthU16C1Image.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, l515.getDepthToMeterConversion(), 0.0);

            syncedRobot.update();
            ReferenceFrame cameraFrame = syncedRobot.hasReceivedFirstMessage() ?
                  syncedRobot.getReferenceFrames().getSteppingCameraFrame() :
                  ReferenceFrame.getWorldFrame();
            // TODO: Wait for frame at time of data aquisition?

            int previousPatchSize = rapidPlanarRegionExtractor.getParameters().getPatchSize();
            ros2PropertySetGroup.update();
            if (patchSizeChangedNotification.poll())
            {
               int newPatchSize = rapidPlanarRegionExtractor.getParameters().getPatchSize();
               if (depthWidth % newPatchSize == 0 && depthHeight % newPatchSize == 0)
               {
                  LogTools.info("New patch size accepted: {}", newPatchSize);
                  rapidPlanarRegionExtractor.setPatchSizeChanged(true);
               }
               else // Reject the parameter and revert to the previous value because it's not an even divisor
               {
                  rapidPlanarRegionExtractor.getParameters().setPatchSize(previousPatchSize);
               }
            }

            rapidPlanarRegionExtractor.readFromSourceImage();
            rapidPlanarRegionExtractor.extractPlanarRegions(onPatchSizeResized);

            debugExtractionImage.getBytedecoOpenCVMat().setTo(BLACK_OPAQUE_RGBA8888);

            rapidPlanarRegionExtractor.findRegions(doNothingIslandConsumer);
            rapidPlanarRegionExtractor.findBoundariesAndHoles(doNothingRingConsumer);
            rapidPlanarRegionExtractor.growRegionBoundaries();
            rapidPlanarRegionExtractor.computePlanarRegions(cameraFrame);
            PlanarRegionsList planarRegionsList = rapidPlanarRegionExtractor.getPlanarRegionsList();

            // Filter out regions that are colliding with the body
            collisionFilter.update();
            int regionIndex = 0;
            while (regionIndex < planarRegionsList.getNumberOfPlanarRegions())
            {
               if (!collisionFilter.test(regionIndex, planarRegionsList.getPlanarRegion(regionIndex)))
                  planarRegionsList.pollPlanarRegion(regionIndex);
               else
                  ++regionIndex;
            }

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
            depthPublisher.publish(depthImagePacket);

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
            debugImagePublisher.publish(debugExtractionImagePacket);

            if (enableROS1 && reconnectROS1Notification.poll())
            {
               ros1Helper.reconnectEverything();
            }

            if (enableROS1 && ros1DepthPublisher.isConnected() && ros1DepthCameraInfoPublisher.isConnected())
            {
               BytePointer dataPointer = depthU16C1Image.ptr();

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

            if (enableROS1 && ros1DebugExtractionImagePublisher.isConnected())
            {
               ros1DebugExtractionImageChannelBuffer.clear();
               debugExtractionImage.rewind();
               int imageSizeInBytes = debugExtractionImage.getBackingDirectByteBuffer().limit();
               for (int i = 0; i < imageSizeInBytes; i++)
               {
                  ros1DebugExtractionImageChannelBuffer.writeByte(debugExtractionImage.getBackingDirectByteBuffer().get());
               }
               ros1DebugExtractionImageChannelBuffer.readerIndex(0);
               ros1DebugExtractionImageChannelBuffer.writerIndex(imageSizeInBytes);

               int bytesPerValue = 4;
               Image message = ros1DebugExtractionImagePublisher.createMessage(debugExtractionImage.getImageWidth(),
                                                                               debugExtractionImage.getImageHeight(),
                                                                               bytesPerValue,
                                                                               "8UC4",
                                                                               ros1DebugExtractionImageChannelBuffer);
               message.getHeader().setStamp(new Time(dataAquisitionTime));

               ros1DebugExtractionImagePublisher.publish(message);
            }
         }
      }
   }

   private void onPatchSizeResized()
   {
      int patchImageWidth = rapidPlanarRegionExtractor.getPatchImageWidth();
      int patchImageHeight = rapidPlanarRegionExtractor.getPatchImageHeight();
      debugExtractionImage = new BytedecoImage(patchImageWidth, patchImageHeight, opencv_core.CV_8UC4);
      if (enableROS1)
      {
         int numberOfBytes = 4 * patchImageWidth * patchImageHeight;
         ros1DebugExtractionImageChannelBuffer = ros1DebugExtractionImagePublisher.getChannelBufferFactory().getBuffer(numberOfBytes);
      }
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
      rapidPlanarRegionExtractor.destroy();
      l515.deleteDevice();
      realSenseHardwareManager.deleteContext();
   }
}
