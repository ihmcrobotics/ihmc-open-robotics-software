package us.ihmc.perception;

import controller_msgs.msg.dds.RobotConfigurationData;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.activeMapping.ActiveMappingRemoteProcess;
import us.ihmc.behaviors.monteCarloPlanning.Agent;
import us.ihmc.behaviors.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.behaviors.monteCarloPlanning.World;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.headless.LocalizationAndMappingProcess;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.tools.ActiveMappingTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.perception.tools.PerceptionFilterTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;

import java.util.ArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class HumanoidPerceptionModule
{
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));

   private BytedecoImage realsenseDepthImage;
   private Mat gridColor = new Mat();

   private World world;
   private Agent agent;

   private final FramePose3D cameraPose = new FramePose3D();

   private OpenCLManager openCLManager;
   private LocalizationAndMappingProcess localizationAndMappingProcess;
   private ActiveMappingRemoteProcess activeMappingRemoteProcess;
   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private RapidHeightMapExtractor rapidHeightMapExtractor;
   private CollidingScanRegionFilter collidingScanRegionFilter;
   private FullHumanoidRobotModel fullRobotModel;
   private PlanarRegionsList regionsInSensorFrame;
   private PlanarRegionsList regionsInWorldFrame;
   private CollisionBoxProvider collisionBoxProvider;
   private FramePlanarRegionsList sensorFrameRegions;

   private RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private RobotConfigurationData robotConfigurationData;

   private PerceptionConfigurationParameters perceptionConfigurationParameters;

   boolean waitIfNecessary = false; // dangerous if true! need a timeout

   public HumanoidPerceptionModule(OpenCLManager openCLManager)
   {
      this.openCLManager = openCLManager;
   }

   public void initializeRealsenseDepthImage(int height, int width)
   {
      this.realsenseDepthImage = new BytedecoImage(width, height, opencv_core.CV_16UC1);
      this.realsenseDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void subscribeToRobotData(String robotName, ROS2Node ros2Node)
   {
      this.robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
      this.robotConfigurationData = new RobotConfigurationData();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class, ROS2Tools.getRobotConfigurationDataTopic(robotName), s ->
      {
         s.takeNextData(robotConfigurationData, null);
      });
   }

   public void updateTerrain(ROS2Helper ros2Helper, Mat depthImage, ReferenceFrame cameraFrame, ReferenceFrame cameraZUpFrame,
                             boolean rapidRegionsEnabled, boolean mappingEnabled, boolean heightMapEnabled)
   {
      if (localizationAndMappingProcess != null)
         localizationAndMappingProcess.setEnableLiveMode(mappingEnabled);

      if (rapidRegionsEnabled)
      {
         executorService.submit(() ->
                                {
                                    updatePlanarRegions(ros2Helper, depthImage, cameraFrame);
                                });
      }

      if (heightMapEnabled)
      {
         executorService.submit(() ->
                                {
                                   updateRapidHeightMap(ros2Helper, depthImage, cameraFrame, cameraZUpFrame);
                                });
      }
   }

   private void updatePlanarRegions(ROS2Helper ros2Helper, Mat depthImage, ReferenceFrame cameraFrame)
   {
      OpenCVTools.convertFloatToShort(depthImage, realsenseDepthImage.getBytedecoOpenCVMat(), 1000.0, 0.0);
      extractFramePlanarRegionsList(rapidPlanarRegionsExtractor,
                                    realsenseDepthImage,
                                    sensorFrameRegions,
                                    regionsInWorldFrame,
                                    regionsInSensorFrame,
                                    cameraFrame);
      filterFramePlanarRegionsList();
      PerceptionMessageTools.publishFramePlanarRegionsList(sensorFrameRegions,
                                                           PerceptionAPI.PERSPECTIVE_RAPID_REGIONS,
                                                           ros2Helper);
   }

   private void updateRapidHeightMap(ROS2Helper ros2Helper, Mat depthImage, ReferenceFrame cameraFrame, ReferenceFrame cameraZUpFrame)
   {
      RigidBodyTransform sensorToWorld = cameraFrame.getTransformToWorldFrame();
      RigidBodyTransform sensorToGround = cameraFrame.getTransformToDesiredFrame(cameraZUpFrame);

      rapidHeightMapExtractor.update(sensorToWorld, sensorToGround, 0);
   }

   public void updateStructural(ROS2Helper ros2Helper, ArrayList<Point3D> pointCloud, ReferenceFrame sensorFrame, float thresholdHeight, boolean display)
   {
      //this.activeMappingRemoteProcess.getActiveMappingModule().submitRangeScan(pointCloud);

      //Instant acquisitionTime = Instant.now();
      //
      cameraPose.setToZero(sensorFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());
      //
      //           occupancyGrid.put(new Scalar(0));

      extractOccupancyGrid(pointCloud,
                           world.getGrid(),
                           sensorFrame.getTransformToWorldFrame(),
                           thresholdHeight,
                           perceptionConfigurationParameters.getOccupancyGridResolution(),
                           70);


      if (activeMappingRemoteProcess == null)
      {
         int gridX = ActiveMappingTools.getIndexFromCoordinates(sensorFrame.getTransformToWorldFrame().getTranslationX(),
                                                                perceptionConfigurationParameters.getOccupancyGridResolution(),
                                                                70);
         int gridY = ActiveMappingTools.getIndexFromCoordinates(sensorFrame.getTransformToWorldFrame().getTranslationY(),
                                                                perceptionConfigurationParameters.getOccupancyGridResolution(),
                                                                70);

         agent.getPosition().set(gridX, gridY);

         agent.measure(world);

         if (display)
         {
            MonteCarloPlannerTools.plotWorld(world, gridColor);
            MonteCarloPlannerTools.plotAgent(agent, gridColor);
            MonteCarloPlannerTools.plotRangeScan(agent.getScanPoints(), gridColor);

            PerceptionDebugTools.display("Monte Carlo Planner World", gridColor, 1, 1400);
         }
      }
   }

   public void initializePerspectiveRapidRegionsExtractor(CameraIntrinsics cameraIntrinsics)
   {
      LogTools.info("Initializing Perspective Rapid Regions: {}", cameraIntrinsics);

      this.sensorFrameRegions = new FramePlanarRegionsList();

      this.rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor(openCLManager,
                                                                         cameraIntrinsics.getHeight(),
                                                                         cameraIntrinsics.getWidth(),
                                                                         cameraIntrinsics.getFx(),
                                                                         cameraIntrinsics.getFy(),
                                                                         cameraIntrinsics.getCx(),
                                                                         cameraIntrinsics.getCy());

      this.rapidPlanarRegionsExtractor.getDebugger().setEnabled(false);
   }

   public void initializePerspectiveRapidHeightMapExtractor(CameraIntrinsics cameraIntrinsics)
   {
      LogTools.info("Initializing Perspective Rapid Height Map: {}", cameraIntrinsics);

      rapidHeightMapExtractor = new RapidHeightMapExtractor();

      rapidHeightMapExtractor.setDepthIntrinsics(cameraIntrinsics);
      rapidHeightMapExtractor.setHeightMapResolution(3.0f, 0.02f);
      rapidHeightMapExtractor.create(openCLManager, realsenseDepthImage, 1);
   }

   public void initializeOccupancyGrid(int depthHeight, int depthWidth, int gridHeight, int gridWidth)
   {
      if (activeMappingRemoteProcess != null)
      {
         LogTools.warn("Initializing Occupancy Grid from Active Mapping Remote Process");

         this.world = activeMappingRemoteProcess.getActiveMappingModule().getPlanner().getWorld();
         this.agent = activeMappingRemoteProcess.getActiveMappingModule().getPlanner().getAgent();
      }
      else
      {
         LogTools.warn("Initializing Occupancy Grid from Scratch");

         this.world = new World(0, gridHeight, gridWidth);
         this.agent = new Agent(new Point2D());
      }
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
      localizationAndMappingProcess = new LocalizationAndMappingProcess(robotName,
                                                                        PerceptionAPI.PERSPECTIVE_RAPID_REGIONS,
                                                                        PerceptionAPI.SPHERICAL_RAPID_REGIONS_WITH_POSE,
                                                                        ros2Node,
                                                                        syncedRobot.getReferenceFrames(),
                                                                        () ->
                                                                        {
                                                                        },
                                                                        smoothing);
   }

   public void initializeActiveMappingProcess(String robotName, DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot, ROS2Node ros2Node)
   {
      LogTools.info("Initializing Active Mapping Process");
      activeMappingRemoteProcess = new ActiveMappingRemoteProcess(robotName,
                                                                  robotModel,
                                                                  syncedRobot,
                                                                  PerceptionAPI.PERSPECTIVE_RAPID_REGIONS,
                                                                  PerceptionAPI.SPHERICAL_RAPID_REGIONS_WITH_POSE,
                                                                  ros2Node,
                                                                  syncedRobot.getReferenceFrames(),
                                                                  () ->
                                                                  {
                                                                  },
                                                                  true);
   }

   public void extractFramePlanarRegionsList(RapidPlanarRegionsExtractor extractor,
                                             BytedecoImage depthImage,
                                             FramePlanarRegionsList sensorFrameRegions,
                                             PlanarRegionsList worldRegions,
                                             PlanarRegionsList sensorRegions,
                                             ReferenceFrame cameraFrame)
   {
      extractor.update(depthImage, cameraFrame, sensorFrameRegions);
      extractor.setProcessing(false);

      sensorRegions = sensorFrameRegions.getPlanarRegionsList();
      worldRegions = sensorRegions.copy();
      worldRegions.applyTransform(cameraFrame.getTransformToWorldFrame());
   }

   public void extractOccupancyGrid(ArrayList<Point3D> pointCloud,
                                    Mat occupancyGrid,
                                    RigidBodyTransform sensorToWorldTransform,
                                    float thresholdHeight,
                                    float occupancyGridResolution,
                                    int offset)
   {
      for (int i = 0; i < pointCloud.size(); i++)
      {
         Point3D point = pointCloud.get(i);
         //sensorToWorldTransform.transform(point);

         int gridX = ActiveMappingTools.getIndexFromCoordinates(point.getX(), occupancyGridResolution, offset);
         int gridY = ActiveMappingTools.getIndexFromCoordinates(point.getY(), occupancyGridResolution, offset);

         if (point.getZ() > thresholdHeight && gridX >= 0 && gridX < occupancyGrid.cols() && gridY >= 0 && gridY < occupancyGrid.rows())
         {
            occupancyGrid.ptr(gridX, gridY).put((byte) 100);
         }
      }
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

   public BytedecoImage getRealsenseDepthImage()
   {
      return this.realsenseDepthImage;
   }

   public RapidPlanarRegionsExtractor getRapidRegionsExtractor()
   {
      return this.rapidPlanarRegionsExtractor;
   }

   public void destroy()
   {
      executorService.shutdownNow();

      try
      {
         boolean result = executorService.awaitTermination(1, TimeUnit.SECONDS);
         Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }

      if (rapidPlanarRegionsExtractor != null)
         rapidPlanarRegionsExtractor.destroy();

      if (localizationAndMappingProcess != null)
         localizationAndMappingProcess.destroy();

      if (activeMappingRemoteProcess != null)
         activeMappingRemoteProcess.destroy();
   }

   public RapidHeightMapExtractor getRapidHeightMapExtractor()
   {
      return rapidHeightMapExtractor;
   }

   public void setPerceptionConfigurationParameters(PerceptionConfigurationParameters perceptionConfigurationParameters)
   {
      this.perceptionConfigurationParameters = perceptionConfigurationParameters;
   }
}
