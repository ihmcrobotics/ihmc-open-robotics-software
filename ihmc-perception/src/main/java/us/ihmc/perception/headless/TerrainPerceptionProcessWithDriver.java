package us.ihmc.perception.headless;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.depthData.CollisionShapeTester;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.perception.*;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/**
 * TerrainPerceptionProcessWithDriver is a headless process that runs the perception frontend for terrain-specific measurements such as planar regions.
 * color, depth, and point cloud data using the depth data obtained from the terrain sensor on the robot. (L515 currently). This class may be extended
 * in the future to support height map extraction, iterative-closest point based registration, LidarScanMessage publisher, and more.
 * <p>
 * Primary responsibilities include (but are not limited to):
 * 1. Loads depth data from the sensor.
 * 2. Loads color data from the sensor.
 * 3. Extracts planar regions from the depth data.
 * 4. Publishes color images on the color topic
 * 5. Publishes depth images on the depth topic
 * 6. Publishes planar regions on the planar regions topic
 */
public class TerrainPerceptionProcessWithDriver
{
   private final RealtimeROS2Node realtimeROS2Node;

   private final PerceptionConfigurationParameters parameters = new PerceptionConfigurationParameters();
   private final Notification destroyedNotification = new Notification();
   private final FramePose3D colorPoseInDepthFrame = new FramePose3D();
   private final ImageMessage depthImageMessage = new ImageMessage();
   private final ImageMessage colorImageMessage = new ImageMessage();
   private final FramePose3D cameraPose = new FramePose3D();
   private final Throttler throttler = new Throttler();

   private final ROS2Topic<FramePlanarRegionsListMessage> frameRegionsTopic;
   private final Supplier<ReferenceFrame> sensorFrameUpdater;
   private final ROS2Topic<ImageMessage> colorTopic;
   private final ROS2Topic<ImageMessage> depthTopic;

   private final RapidPlanarRegionsExtractor rapidRegionsExtractor;
   private final OpenCLManager openCLManager;
   private final ROS2Helper ros2Helper;
   private final RealSenseHardwareManager realSenseHardwareManager;
   private final RealsenseConfiguration realsenseConfiguration;
   private final _cl_program openCLProgram;
   private final BytedecoRealsense realsense;

   private ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private CollidingScanRegionFilter collisionFilter;
   private BytedecoImage depthBytedecoImage;

   private Mat depth16UC1Image;
   private Mat color8UC3Image;
   private Mat yuvColorImage;

   private final double outputPeriod;
   private boolean initialized = false;
   private volatile boolean running = true;

   private int depthWidth;
   private int depthHeight;
   private int colorWidth;
   private int colorHeight;
   private long depthSequenceNumber = 0;
   private long colorSequenceNumber = 0;

   public TerrainPerceptionProcessWithDriver(String serialNumber,
                                             RealsenseConfiguration realsenseConfiguration,
                                             ROS2Topic<ImageMessage> depthTopic,
                                             ROS2Topic<ImageMessage> colorTopic,
                                             ROS2Topic<FramePlanarRegionsListMessage> frameRegionsTopic,
                                             Supplier<ReferenceFrame> sensorFrameUpdater)
   {
      this.realsenseConfiguration = realsenseConfiguration;
      this.depthTopic = depthTopic;
      this.colorTopic = colorTopic;
      this.frameRegionsTopic = frameRegionsTopic;
      this.sensorFrameUpdater = sensorFrameUpdater;

      this.outputPeriod = UnitConversions.hertzToSeconds(31.0f);

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_videopub");
      realtimeROS2Node.spin();

      openCLManager = new OpenCLManager();
      rapidRegionsExtractor = new RapidPlanarRegionsExtractor();

      realSenseHardwareManager = new RealSenseHardwareManager();

      LogTools.info("Creating Bytedeco Realsense Using: {}", serialNumber);
      realsense = realSenseHardwareManager.createBytedecoRealsenseDevice(serialNumber, realsenseConfiguration);
      if (realsense.getDevice() == null)
      {
         destroy();
         throw new RuntimeException("Realsense device not found. Set -D<model>.serial.number=00000000000");
      }
      realsense.enableColor(realsenseConfiguration);
      realsense.initialize();

      depthWidth = realsense.getDepthWidth();
      depthHeight = realsense.getDepthHeight();
      colorWidth = realsense.getColorWidth();
      colorHeight = realsense.getColorHeight();

      LogTools.info("Depth width: " + depthWidth + ", height: " + depthHeight);
      LogTools.info("Color width: " + colorWidth + ", height: " + colorHeight);

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "realsense_color_and_depth_publisher");
      ros2Helper = new ROS2Helper(ros2Node);

      openCLProgram = openCLManager.loadProgram("RapidRegionsExtractor");

      depthBytedecoImage = new BytedecoImage(realsense.getDepthWidth(), realsense.getDepthHeight(), opencv_core.CV_16UC1);

      LogTools.info(String.format("Sensor Fx: %.2f, Sensor Fy: %.2f, Sensor Cx: %.2f, Sensor Cy: %.2f", realsense.getDepthFocalLengthPixelsX(),
              realsense.getDepthFocalLengthPixelsY(), realsense.getDepthPrincipalOffsetXPixels(), realsense.getDepthPrincipalOffsetYPixels()));

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
   }

   /**
    * Must be called from the sensor-specific calling class, after the sensor and logger initialization have succeeded.
    * We run in a daemon thread, because otherwise it will get killed on Ctrl+C before the shutdown hooks are finished running.
    * See {@link Runtime#addShutdownHook(Thread)} for details.
    */
   public void run()
   {
      ThreadTools.startAsDaemon(this::updateThread, getClass().getSimpleName() + "UpdateThread");
   }

   private void updateThread()
   {
      while (running)
      {
         update();
         throttler.waitAndRun(outputPeriod); // do the waiting after we send to remove unnecessary latency
      }

      // Make sure the Realsense
      ThreadTools.sleep(100);

      if (realsense != null)
         realsense.deleteDevice();
      realSenseHardwareManager.deleteContext();

      destroyedNotification.set();
   }

   /**
    * Update the sensor and logger, and publish the data to ROS2
    */
   private void update()
   {
      parameters.setRapidRegionsEnabled(true);
      if (realsense.readFrameData())
      {
         realsense.updateDataBytePointers();

         Instant acquisitionTime = Instant.now();

         if (!initialized)
         {
            if (realsense.getDepthFocalLengthPixelsX() == 0.0)
            {
               LogTools.warn("Realsense focal length is 0.0, not publishing data");
               return;
            }

            rapidRegionsExtractor.create(openCLManager,
                    openCLProgram,
                    depthHeight,
                    depthWidth,
                    realsense.getDepthFocalLengthPixelsX(),
                    realsense.getDepthFocalLengthPixelsY(),
                    realsense.getDepthPrincipalOffsetXPixels(),
                    realsense.getDepthPrincipalOffsetYPixels());

            rapidRegionsExtractor.getDebugger().setEnabled(false);

            ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
            ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERCEPTION_CONFIGURATION_PARAMETERS, parameters);
            ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_RAPID_REGION_PARAMETERS, rapidRegionsExtractor.getParameters());
            ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_POLYGONIZER_PARAMETERS,
                    rapidRegionsExtractor.getRapidPlanarRegionsCustomizer().getPolygonizerParameters());
            ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_CONVEX_HULL_FACTORY_PARAMETERS,
                    rapidRegionsExtractor.getRapidPlanarRegionsCustomizer().getConcaveHullFactoryParameters());

            initialized = true;
         }

         // Important not to store as a field, as update() needs to be called each frame
         ReferenceFrame cameraFrame = sensorFrameUpdater.get();
         cameraPose.setToZero(cameraFrame);
         cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

         BytePointer compressedDepthPointer = new BytePointer(); // deallocate later
         BytePointer compressedColorPointer = new BytePointer(); // deallocate later

         if (parameters.getPublishDepth() || parameters.getRapidRegionsEnabled())
         {
            depth16UC1Image = new Mat(realsense.getDepthHeight(),
                    realsense.getDepthWidth(),
                    opencv_core.CV_16UC1,
                    realsense.getDepthFrameData()); // deallocate later
         }

         if (parameters.getPublishDepth())
         {
            OpenCVTools.compressImagePNG(depth16UC1Image, compressedDepthPointer);
            PerceptionMessageTools.setDepthIntrinsicsFromRealsense(realsense, depthImageMessage);
            CameraModel.PINHOLE.packMessageFormat(depthImageMessage);
            PerceptionMessageTools.publishCompressedDepthImage(compressedDepthPointer,
                                                               depthTopic,
                                                               depthImageMessage,
                                                               ros2Helper,
                                                               cameraPose,
                                                               acquisitionTime,
                                                               depthSequenceNumber++,
                                                               realsense.getDepthHeight(),
                                                               realsense.getDepthWidth(),
                                                               (float) realsense.getDepthDiscretization());
         }

         if (parameters.getPublishColor())
         {
            colorPoseInDepthFrame.set(realsense.getDepthToColorTranslation(), realsense.getDepthToColorRotation());

            color8UC3Image = new Mat(realsense.getColorHeight(),
                    realsense.getColorWidth(),
                    opencv_core.CV_8UC3,
                    realsense.getColorFrameData()); // deallocate later

            // YUV I420 has 1.5 times the height of the image
            yuvColorImage = new Mat(realsense.getColorHeight() * 1.5, realsense.getColorWidth(), opencv_core.CV_8UC1); // deallocate later

            OpenCVTools.compressRGBImageJPG(color8UC3Image, yuvColorImage, compressedColorPointer);

            PerceptionMessageTools.setColorIntrinsicsFromRealsense(realsense, colorImageMessage);
            CameraModel.PINHOLE.packMessageFormat(colorImageMessage);
            PerceptionMessageTools.publishJPGCompressedColorImage(compressedColorPointer,
                    colorTopic,
                    colorImageMessage,
                    ros2Helper,
                    colorPoseInDepthFrame,
                    acquisitionTime,
                    colorSequenceNumber++,
                    realsense.getColorHeight(),
                    realsense.getColorWidth(),
                    (float) realsense.getDepthDiscretization());

            color8UC3Image.deallocate();
            yuvColorImage.deallocate();
            compressedColorPointer.deallocate();
         }

         if (parameters.getRapidRegionsEnabled())
         {
//            PerceptionDebugTools.displayDepth("Depth", depth16UC1Image, 1);
            depth16UC1Image.convertTo(depthBytedecoImage.getBytedecoOpenCVMat(), opencv_core.CV_16UC1, 1, 0);
            FramePlanarRegionsList framePlanarRegionsList = new FramePlanarRegionsList();
            extractFramePlanarRegionsList(depthBytedecoImage, cameraFrame, framePlanarRegionsList);

            PerceptionMessageTools.publishFramePlanarRegionsList(framePlanarRegionsList, frameRegionsTopic, ros2Helper);

            LogTools.info("Total Planar Regions: " + framePlanarRegionsList.getPlanarRegionsList().getNumberOfPlanarRegions());
         }

         if (parameters.getPublishDepth() || parameters.getRapidRegionsEnabled())
         {
            depth16UC1Image.deallocate();
            compressedDepthPointer.deallocate();
         }

         ros2PropertySetGroup.update();
      }
   }

   private void extractFramePlanarRegionsList(BytedecoImage depthImage, ReferenceFrame cameraFrame, FramePlanarRegionsList framePlanarRegionsList)
   {
      rapidRegionsExtractor.update(depthImage, cameraFrame, framePlanarRegionsList);
      rapidRegionsExtractor.setProcessing(false);
   }

   /**
    * Must be called in the shutdown hook from the sensor-specific calling class. Handles Ctrl + C based closing gracefully.
    */
   public void destroy()
   {
      running = false;
      rapidRegionsExtractor.destroy();
      depthBytedecoImage.destroy(openCLManager);
      openCLManager.destroy();
      destroyedNotification.blockingPoll();
   }

   public void setupCollisionFilter(FullHumanoidRobotModel fullRobotModel, CollisionBoxProvider collisionBoxProvider)
   {
      CollisionShapeTester shapeTester = new CollisionShapeTester();
      for (RobotSide robotSide : RobotSide.values)
      {
         List<JointBasics> joints = new ArrayList<>();
         RigidBodyBasics shin = fullRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getPredecessor();
         MultiBodySystemTools.collectJointPath(fullRobotModel.getPelvis(), shin, joints);
         joints.forEach(joint -> shapeTester.addJoint(collisionBoxProvider, joint));
      }
      collisionFilter = new CollidingScanRegionFilter(shapeTester);
   }

   public void applyCollisionFilter(PlanarRegionsList planarRegionsList)
   {
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
   }

   public static void main(String[] args)
   {
      // Benchtop L515: F1120592, Tripod: F1121365, Local: F0245563, Nadia: F112114, D435: 108522071219, D435: 213522252883, 215122254074
      String realsenseSerialNumber = System.getProperty("d455.serial.number", "213522252883");
      TerrainPerceptionProcessWithDriver process = new TerrainPerceptionProcessWithDriver(realsenseSerialNumber,
                                                                                          RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ,
                                                                                          PerceptionAPI.D455_DEPTH_IMAGE,
                                                                                          PerceptionAPI.D455_COLOR_IMAGE,
                                                                                          PerceptionAPI.PERSPECTIVE_RAPID_REGIONS,
                                                                                          ReferenceFrame::getWorldFrame);
      process.run();
      ThreadTools.sleepForever();
   }
}
