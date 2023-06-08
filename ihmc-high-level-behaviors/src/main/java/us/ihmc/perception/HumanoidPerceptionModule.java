package us.ihmc.perception;

import controller_msgs.msg.dds.RobotConfigurationData;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.headless.LocalizationAndMappingProcess;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.tools.PerceptionFilterTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

public class HumanoidPerceptionModule
{
   private final AtomicReference<FramePlanarRegionsList> frameRegionsReference = new AtomicReference<>();
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));

   private BytedecoImage bytedecoDepthImage;
   private OpenCLManager openCLManager;
   private LocalizationAndMappingProcess localizationAndMappingProcess;
   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private CollidingScanRegionFilter collidingScanRegionFilter;
   private FullHumanoidRobotModel fullRobotModel;
   private PlanarRegionsList regionsInSensorFrame;
   private PlanarRegionsList regionsInWorldFrame;
   private CollisionBoxProvider collisionBoxProvider;
   private FramePlanarRegionsList sensorFrameRegions;

   private RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private RobotConfigurationData robotConfigurationData;

   boolean waitIfNecessary = false; // dangerous if true! need a timeout

   public HumanoidPerceptionModule(OpenCLManager openCLManager)
   {
      this.openCLManager = openCLManager;
   }

   public void subscribeToRobotData(String robotName, ROS2Node ros2Node)
   {
      this.robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
      this.robotConfigurationData = new RobotConfigurationData();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node,
                                                    RobotConfigurationData.class,
                                                    ROS2Tools.getRobotConfigurationDataTopic(robotName),
                                                    s ->
                                                    {
                                                       s.takeNextData(robotConfigurationData, null);
                                                    });
   }

   public void update(ROS2Helper ros2Helper, Mat depthImage, ReferenceFrame cameraFrame, boolean rapidRegionsEnabled, boolean mappingEnabled)
   {
      localizationAndMappingProcess.setEnableLiveMode(mappingEnabled);

      if (rapidRegionsEnabled)
      {
         executorService.submit(() -> {

            if (robotConfigurationData != null && robotConfigurationData.getJointNameHash() != 0)
            {
               robotConfigurationDataBuffer.update(robotConfigurationData);
               long newestTimestamp = robotConfigurationDataBuffer.getNewestTimestamp();
               long selectedTimestamp = robotConfigurationDataBuffer.updateFullRobotModel(waitIfNecessary, newestTimestamp, this.fullRobotModel, null);
            }

            OpenCVTools.convertFloatToShort(depthImage, bytedecoDepthImage.getBytedecoOpenCVMat(),
                                            1000.0,
                                            0.0);

            extractFramePlanarRegionsList(cameraFrame);
            filterFramePlanarRegionsList();

            PerceptionMessageTools.publishFramePlanarRegionsList(this.sensorFrameRegions, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS, ros2Helper);
         });
      }
   }

   public void initializePerspectiveRapidRegionsExtractor(CameraIntrinsics cameraIntrinsics)
   {
      LogTools.info("Initializing Perspective Rapid Regions: {}", cameraIntrinsics);

      this.sensorFrameRegions = new FramePlanarRegionsList();
      this.bytedecoDepthImage = new BytedecoImage(cameraIntrinsics.getWidth(), cameraIntrinsics.getHeight(), opencv_core.CV_16UC1);
      this.bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      this.rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor(openCLManager, cameraIntrinsics.getHeight(), cameraIntrinsics.getWidth(),
                                              cameraIntrinsics.getFx(), cameraIntrinsics.getFy(), cameraIntrinsics.getCx(),
                                              cameraIntrinsics.getCy());

      this.rapidPlanarRegionsExtractor.getDebugger().setEnabled(false);
   }

   public void initializeBodyCollisionFilter(FullHumanoidRobotModel fullRobotModel, CollisionBoxProvider collisionBoxProvider)
   {
      LogTools.info("Initializing Body Collision Filter");
      this.fullRobotModel = fullRobotModel;
      this.collisionBoxProvider = collisionBoxProvider;
      this.collidingScanRegionFilter = PerceptionFilterTools.createHumanoidShinCollisionFilter(fullRobotModel, collisionBoxProvider);
   }

   public void initializeLocalizationAndMappingProcess(ROS2SyncedRobotModel syncedRobot, String robotName, ROS2Node ros2Node, boolean smoothing)
   {
      LogTools.info("Initializing Localization and Mapping Process (Smoothing: {})", smoothing);
      localizationAndMappingProcess = new LocalizationAndMappingProcess(robotName, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS,
              PerceptionAPI.SPHERICAL_RAPID_REGIONS_WITH_POSE, ros2Node, syncedRobot.getReferenceFrames(), () -> {}, smoothing);
   }

   public void extractFramePlanarRegionsList(ReferenceFrame cameraFrame)
   {
      this.rapidPlanarRegionsExtractor.update(bytedecoDepthImage, cameraFrame, this.sensorFrameRegions);
      this.rapidPlanarRegionsExtractor.setProcessing(false);

      this.regionsInSensorFrame = this.sensorFrameRegions.getPlanarRegionsList();
      this.regionsInWorldFrame = this.regionsInSensorFrame.copy();
      this.regionsInWorldFrame.applyTransform(cameraFrame.getTransformToWorldFrame());
   }

   public void filterFramePlanarRegionsList()
   {
      this.fullRobotModel.updateFrames();
      this.collidingScanRegionFilter.update();

      synchronized (sensorFrameRegions)
      {
         PerceptionFilterTools.filterCollidingPlanarRegions(sensorFrameRegions, this.collidingScanRegionFilter);
      }
   }

   public FramePlanarRegionsList getFramePlanarRegionsResult()
   {
      return this.sensorFrameRegions;
   }

   public PlanarRegionsList getRegionsInSensorFrame()
   {
      return this.regionsInSensorFrame;
   }

   public PlanarRegionsList getRegionsInWorldFrame()
   {
      return this.regionsInWorldFrame;
   }

   public BytedecoImage getBytedecoDepthImage()
   {
      return this.bytedecoDepthImage;
   }

   public RapidPlanarRegionsExtractor getRapidRegionsExtractor()
   {
      return this.rapidPlanarRegionsExtractor;
   }

   public void destroy()
   {
      executorService.shutdownNow();

      try {
         boolean result = executorService.awaitTermination(1, TimeUnit.SECONDS);
         Thread.sleep(1000);
      } catch (InterruptedException e) {
         throw new RuntimeException(e);
      }

      rapidPlanarRegionsExtractor.destroy();
      localizationAndMappingProcess.destroy();
   }
}
