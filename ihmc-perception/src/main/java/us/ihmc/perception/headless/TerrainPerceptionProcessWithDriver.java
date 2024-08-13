package us.ihmc.perception.headless;

import controller_msgs.msg.dds.RobotConfigurationData;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.StateEstimatorAPI;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.realsense.RealsenseDevice;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.tools.thread.Throttler;

import java.time.Instant;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

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
 * <p>
 * Benchtop L515: F1120592, Tripod: F1121365, Local: F0245563, Nadia: F112114, D435: 108522071219, D435: 213522252883, 215122254074, 752112070330
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

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 8);
   //   private final ExecutorService executorService = Executors.newFixedThreadPool(16);

   protected final ScheduledExecutorService scheduledExecutorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                                   getClass(),
                                                                                                                   ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private final ROS2Topic<ImageMessage> colorTopic;
   private final ROS2Topic<ImageMessage> depthTopic;

   private final OpenCLManager openCLManager;
   private final ROS2Helper ros2Helper;
   private final RealsenseDeviceManager realsenseDeviceManager;
   private final RealsenseDevice realsense;
   private final BytedecoImage depthBytedecoImage;
   private final Runnable syncedRobotUpdater;
   private final HumanoidReferenceFrames referenceFrames;

   private final HumanoidPerceptionModule humanoidPerception;

   private Mat depth16UC1Image;
   private Mat color8UC3Image;
   private Mat yuvColorImage;
   private Mat sourceDepthImage;
   private Mat sourceColorImage;

   private final double outputPeriod = UnitConversions.hertzToSeconds(30.0f);
   private volatile boolean running = true;

   private final int depthWidth;
   private final int depthHeight;
   private final int colorWidth;
   private final int colorHeight;
   private long depthSequenceNumber = 0;
   private long colorSequenceNumber = 0;

   public TerrainPerceptionProcessWithDriver(String serialNumber,
                                             String robotName,
                                             CollisionBoxProvider collisionBoxProvider,
                                             FullHumanoidRobotModel fullRobotModel,
                                             RealsenseConfiguration realsenseConfiguration,
                                             ROS2StoredPropertySetGroup ros2PropertySetGroup,
                                             ROS2Helper ros2Helper,
                                             ROS2Topic<ImageMessage> depthTopic,
                                             ROS2Topic<ImageMessage> colorTopic,
                                             HumanoidReferenceFrames referenceFrames,
                                             Runnable syncedRobotUpdater)
   {
      this.syncedRobotUpdater = syncedRobotUpdater;
      this.ros2Helper = ros2Helper;
      this.depthTopic = depthTopic;
      this.colorTopic = colorTopic;
      this.referenceFrames = referenceFrames;

      if (fullRobotModel == null)
         LogTools.info("Creating terrain process with no robot model.");
      if (collisionBoxProvider == null)
         LogTools.info("Creating terrain process with no collision provider.");

      openCLManager = new OpenCLManager();
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_videopub");
      realtimeROS2Node.spin();
      realsenseDeviceManager = new RealsenseDeviceManager();

      LogTools.info("Creating Bytedeco Realsense Using: {}", serialNumber);
      realsense = realsenseDeviceManager.createBytedecoRealsenseDevice(serialNumber, realsenseConfiguration);
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

      depthBytedecoImage = new BytedecoImage(realsense.getDepthWidth(), realsense.getDepthHeight(), opencv_core.CV_16UC1);

      ros2Node.createSubscription(StateEstimatorAPI.getRobotConfigurationDataTopic(robotName).withTypeName(RobotConfigurationData.class), s ->
      {
         LogTools.warn("Realsense focal length is 0.0, not publishing data");
      });

      LogTools.info(String.format("Sensor Fx: %.2f, Sensor Fy: %.2f, Sensor Cx: %.2f, Sensor Cy: %.2f",
                                  realsense.getDepthFocalLengthPixelsX(),
                                  realsense.getDepthFocalLengthPixelsY(),
                                  realsense.getDepthPrincipalOffsetXPixels(),
                                  realsense.getDepthPrincipalOffsetYPixels()));

      humanoidPerception = new HumanoidPerceptionModule(openCLManager);
      humanoidPerception.initializeBodyCollisionFilter(fullRobotModel, collisionBoxProvider);
      humanoidPerception.initializeRealsenseDepthImage(realsense.getDepthHeight(), realsense.getDepthWidth());
      humanoidPerception.initializePerspectiveRapidRegionsExtractor(realsense.getDepthCameraIntrinsics());
      humanoidPerception.initializeHeightMapExtractor(ros2Helper, referenceFrames, realsense.getDepthCameraIntrinsics());
      humanoidPerception.getRapidRegionsExtractor().setEnabled(true);

      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERCEPTION_CONFIGURATION_PARAMETERS, parameters);
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.HEIGHT_MAP_PARAMETERS,
                                                     humanoidPerception.getRapidHeightMapExtractor().getHeightMapParameters());
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_RAPID_REGION_PARAMETERS,
                                                     humanoidPerception.getRapidRegionsExtractor().getParameters());
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_POLYGONIZER_PARAMETERS,
                                                     humanoidPerception.getRapidRegionsExtractor()
                                                                       .getRapidPlanarRegionsCustomizer()
                                                                       .getPolygonizerParameters());
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERSPECTIVE_CONVEX_HULL_FACTORY_PARAMETERS,
                                                     humanoidPerception.getRapidRegionsExtractor()
                                                                       .getRapidPlanarRegionsCustomizer()
                                                                       .getConcaveHullFactoryParameters());

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
   }

   /**
    * Must be called from the sensor-specific calling class, after the sensor and logger initialization have succeeded.
    * We run in a daemon thread, because otherwise it will get killed on Ctrl+C before the shutdown hooks are finished running.
    * See {@link Runtime#addShutdownHook(Thread)} for details.
    */
   public void run()
   {
      scheduledExecutorService.scheduleAtFixedRate(this::updateThread, 0, 33, TimeUnit.MILLISECONDS);
   }

   private void updateThread()
   {
      while (running)
      {
         update();
         throttler.waitAndRun(outputPeriod); // do the waiting after we send to remove unnecessary latency
      }

      if (realsense != null)
         realsense.deleteDevice();
      realsenseDeviceManager.deleteContext();

      destroyedNotification.set();
   }

   /**
    * Update the sensor and logger, and publish the data to ROS2
    */
   private void update()
   {
      if (realsense.readFrameData())
      {
         realsense.updateDataBytePointers();

         Instant acquisitionTime = Instant.now();

         // Important not to store as a field, as update() needs to be called each frame
         syncedRobotUpdater.run();
         ReferenceFrame cameraFrame = referenceFrames.getSteppingCameraFrame();
         ReferenceFrame cameraZUpFrame = referenceFrames.getSteppingCameraZUpFrame();

         cameraPose.setToZero(cameraFrame);
         cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

         BytePointer compressedDepthPointer = new BytePointer(); // deallocate later
         BytePointer compressedColorPointer = new BytePointer(); // deallocate later

         if (parameters.getPublishDepth() || parameters.getRapidRegionsEnabled() || parameters.getHeightMapEnabled())
         {
            sourceDepthImage = new Mat(realsense.getDepthHeight(),
                                       realsense.getDepthWidth(),
                                       opencv_core.CV_16UC1,
                                       realsense.getDepthFrameData()); // deallocate later
            depth16UC1Image = sourceDepthImage.clone();
         }

         if (parameters.getPublishDepth())
         {
            executorService.submit(() ->
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
                                   });
         }

         if (parameters.getPublishColor())
         {
            colorPoseInDepthFrame.set(realsense.getDepthToColorTranslation(), realsense.getDepthToColorRotation());

            sourceColorImage = new Mat(realsense.getColorHeight(),
                                       realsense.getColorWidth(),
                                       opencv_core.CV_8UC3,
                                       realsense.getColorFrameData()); // deallocate later
            color8UC3Image = sourceColorImage.clone();

            // YUV I420 has 1.5 times the height of the image
            yuvColorImage = new Mat(realsense.getColorHeight() * 1.5, realsense.getColorWidth(), opencv_core.CV_8UC1); // deallocate later

            executorService.submit(() ->
                                   {
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
                                   });
         }

         // TODO: Add spherical region extraction toggling parameter.
         // humanoidPerception.setSphericalRegionsEnabled(parameters.getRapidRegionsEnabled());
         humanoidPerception.setRapidRegionsEnabled(parameters.getRapidRegionsEnabled());
         humanoidPerception.setHeightMapEnabled(parameters.getHeightMapEnabled());
         humanoidPerception.setMappingEnabled(parameters.getSLAMEnabled());

         /* Call all "enabled" setters before this update call.*/
         humanoidPerception.updateTerrain(ros2Helper, depth16UC1Image, cameraFrame, cameraZUpFrame, false);

         if (parameters.getPublishDepth() || parameters.getRapidRegionsEnabled())
         {
            sourceDepthImage.deallocate();
            compressedDepthPointer.deallocate();
         }
      }
   }

   /**
    * Must be called in the shutdown hook from the sensor-specific calling class. Handles Ctrl + C based closing gracefully.
    */
   public void destroy()
   {
      running = false;

      scheduledExecutorService.shutdown();
      executorService.clearTaskQueue();
      executorService.destroy();

      realtimeROS2Node.destroy();
      if (humanoidPerception != null)
      {
         humanoidPerception.destroy();
      }

      openCLManager.destroy();
      if (depthBytedecoImage != null)
      {
         depthBytedecoImage.destroy(openCLManager);
      }

      destroyedNotification.blockingPoll();
   }

   public HumanoidPerceptionModule getHumanoidPerceptionModule()
   {
      return humanoidPerception;
   }
}
