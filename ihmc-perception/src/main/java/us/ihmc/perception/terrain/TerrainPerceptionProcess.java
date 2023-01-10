package us.ihmc.perception.terrain;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
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
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.ihmcPerception.depthData.CollisionShapeTester;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.perception.*;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsCustomizer;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListWithPose;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.PausablePeriodicThread;

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

   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense l515;

   private Mat depthU16C1Image;
   private BytedecoImage depth32FC1Image;
   private int depthWidth;
   private int depthHeight;

   private CameraPinholeBrown depthCameraIntrinsics;
   private final CollidingScanRegionFilter collisionFilter;

   private Supplier<ReferenceFrame> sensorFrameUpdater;

   private OpenCLManager openCLManager;
   private _cl_program openCLProgram;

   private boolean publishTimestamped = false;

   public TerrainPerceptionProcess(Supplier<ReferenceFrame> sensorFrameUpdater, CollisionBoxProvider collisionBoxProvider, FullHumanoidRobotModel fullRobot)
   {
      this.sensorFrameUpdater = sensorFrameUpdater;
      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      this.openCLManager = new OpenCLManager();
      this.openCLManager.create();

      openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");

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
                                                                                                        rapidPlanarRegionsExtractor.getParameters());
      ros2PropertySetGroup.registerStoredPropertySet(GPUPlanarRegionExtractionComms.POLYGONIZER_PARAMETERS,
                                                     rapidPlanarRegionsCustomizer.getPolygonizerParameters());
      ros2PropertySetGroup.registerStoredPropertySet(GPUPlanarRegionExtractionComms.CONVEX_HULL_FACTORY_PARAMETERS,
                                                     rapidPlanarRegionsCustomizer.getConcaveHullFactoryParameters());

      CollisionShapeTester shapeTester = new CollisionShapeTester();
      for (RobotSide robotSide : RobotSide.values)
      {
         List<JointBasics> joints = new ArrayList<>();
         RigidBodyBasics shin = fullRobot.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getPredecessor();
         MultiBodySystemTools.collectJointPath(fullRobot.getPelvis(), shin, joints);
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
               LogTools.info("Reading L515 Depth Image Now.");

               MutableBytePointer depthFrameData = l515.getDepthFrameData();
               depthU16C1Image = new Mat(depthHeight, depthWidth, opencv_core.CV_16UC1, depthFrameData);
               depth32FC1Image = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_32FC1);

               depthCameraIntrinsics.setFx(l515.getDepthFocalLengthPixelsX());
               depthCameraIntrinsics.setFy(l515.getDepthFocalLengthPixelsY());
               depthCameraIntrinsics.setSkew(0.0);
               depthCameraIntrinsics.setCx(l515.getDepthPrincipalOffsetXPixels());
               depthCameraIntrinsics.setCy(l515.getDepthPrincipalOffsetYPixels());

               rapidPlanarRegionsExtractor.create(openCLManager, openCLProgram,
                                                  depth32FC1Image,
                                                  depthWidth,
                                                  depthHeight,
                                                  depthCameraIntrinsics.fx,
                                                  depthCameraIntrinsics.fy,
                                                  depthCameraIntrinsics.cx,
                                                  depthCameraIntrinsics.cy);

               LogTools.info("Rapid Regions Extractor Initialized.");
            }

            val0 = Short.toUnsignedInt(depthU16C1Image.ptr(0, 0).getShort());
            val1 = Short.toUnsignedInt(depthU16C1Image.ptr(100, 200).getShort());
            val2 = Short.toUnsignedInt(depthU16C1Image.ptr(400, 200).getShort());
            val3 = Short.toUnsignedInt(depthU16C1Image.ptr(600, 50).getShort());

            depthU16C1Image.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, l515.getDepthToMeterConversion(), 0.0);

            // Calls syncedRobot.update()
            ReferenceFrame cameraFrame = sensorFrameUpdater.get();
            ros2PropertySetGroup.update();

            PlanarRegionsListWithPose planarRegionsListWithPose = new PlanarRegionsListWithPose();
            rapidPlanarRegionsExtractor.update(true);
            rapidPlanarRegionsCustomizer.createCustomPlanarRegionsList(rapidPlanarRegionsExtractor.getGPUPlanarRegions(),
                                                                       cameraFrame, planarRegionsListWithPose);
            PlanarRegionsList planarRegionsList = planarRegionsListWithPose.getPlanarRegionsList();

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

         }
      }
   }

   private void destroy()
   {
      realtimeROS2Node.destroy();
      rapidPlanarRegionsExtractor.destroy();
      l515.deleteDevice();
      realSenseHardwareManager.deleteContext();
   }
}
