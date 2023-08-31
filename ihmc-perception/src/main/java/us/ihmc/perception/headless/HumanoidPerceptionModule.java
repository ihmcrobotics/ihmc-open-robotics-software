package us.ihmc.perception.headless;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
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

import java.time.Instant;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class HumanoidPerceptionModule
{
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final FramePose3D cameraPose = new FramePose3D();

   private BytedecoImage realsenseDepthImage;
   private OpenCLManager openCLManager;
   private LocalizationAndMappingTask localizationAndMappingTask;
   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private RapidHeightMapExtractor rapidHeightMapExtractor;
   private CollidingScanRegionFilter collidingScanRegionFilter;
   private FullHumanoidRobotModel fullRobotModel;
   private PlanarRegionsList regionsInSensorFrame;
   private PlanarRegionsList regionsInWorldFrame;
   private CollisionBoxProvider collisionBoxProvider;
   private FramePlanarRegionsList sensorFrameRegions;

   private final ImageMessage depthImageMessage = new ImageMessage();
   private final BytePointer compressedDepthPointer = new BytePointer();

   private PerceptionConfigurationParameters perceptionConfigurationParameters;

   public HumanoidPerceptionModule(OpenCLManager openCLManager)
   {
      this.openCLManager = openCLManager;
   }

   public void initializeRealsenseDepthImage(int height, int width)
   {
      this.realsenseDepthImage = new BytedecoImage(width, height, opencv_core.CV_16UC1);
      this.realsenseDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void updateTerrain(ROS2Helper ros2Helper, Mat incomingDepth, ReferenceFrame cameraFrame, ReferenceFrame cameraZUpFrame,
                             boolean rapidRegionsEnabled, boolean mappingEnabled, boolean heightMapEnabled, boolean metricDepth)
   {
      if (localizationAndMappingTask != null)
         localizationAndMappingTask.setEnableLiveMode(mappingEnabled);

      if (rapidRegionsEnabled || heightMapEnabled)
      {
         if (metricDepth)
         {
            OpenCVTools.convertFloatToShort(incomingDepth, realsenseDepthImage.getBytedecoOpenCVMat(), 1000.0, 0.0);
         }
         else
         {
            incomingDepth.convertTo(realsenseDepthImage.getBytedecoOpenCVMat(), opencv_core.CV_16UC1, 1, 0);
         }
      }

      if (rapidRegionsEnabled)
      {
         executorService.submit(() ->
         {
            updatePlanarRegions(ros2Helper, cameraFrame);
         });
      }

      if (heightMapEnabled)
      {
         executorService.submit(() ->
         {
            updateRapidHeightMap(ros2Helper, cameraFrame, cameraZUpFrame);
         });
      }
   }

   private void updatePlanarRegions(ROS2Helper ros2Helper, ReferenceFrame cameraFrame)
   {
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

   private void updateRapidHeightMap(ROS2Helper ros2Helper, ReferenceFrame cameraFrame, ReferenceFrame cameraZUpFrame)
   {
      RigidBodyTransform sensorToWorld = cameraFrame.getTransformToWorldFrame();
      RigidBodyTransform sensorToGround = cameraFrame.getTransformToDesiredFrame(cameraZUpFrame);
      RigidBodyTransform groundToWorld = cameraZUpFrame.getTransformToWorldFrame();

      rapidHeightMapExtractor.update(sensorToWorld, sensorToGround, groundToWorld);

      Instant acquisitionTime = Instant.now();

      OpenCVTools.compressImagePNG(rapidHeightMapExtractor.getGlobalHeightMapImage().getBytedecoOpenCVMat(), compressedDepthPointer);
      PerceptionMessageTools.publishCompressedDepthImage(compressedDepthPointer,
                                                         PerceptionAPI.HEIGHT_MAP_GLOBAL,
                                                         depthImageMessage,
                                                         ros2Helper,
                                                         cameraPose,
                                                         acquisitionTime, rapidHeightMapExtractor.getSequenceNumber(),
                                                         rapidHeightMapExtractor.getGlobalHeightMapImage().getImageHeight(),
                                                         rapidHeightMapExtractor.getGlobalHeightMapImage().getImageWidth(),
                                                         10000.0f);
   }

   public void updateStructural(ROS2Helper ros2Helper, List<Point3D> pointCloud, ReferenceFrame sensorFrame, Mat occupancy, float thresholdHeight, boolean display)
   {
      cameraPose.setToZero(sensorFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      extractOccupancyGrid(pointCloud,
                           occupancy,
                           sensorFrame.getTransformToWorldFrame(),
                           thresholdHeight,
                           perceptionConfigurationParameters.getOccupancyGridResolution(),
                           70);

      // TODO: Publish the occupancy grid as ImageMessage using the ROS2Helper.
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

   public void initializeBodyCollisionFilter(FullHumanoidRobotModel fullRobotModel, CollisionBoxProvider collisionBoxProvider)
   {
      LogTools.info("Initializing Body Collision Filter");
      this.fullRobotModel = fullRobotModel;
      this.collisionBoxProvider = collisionBoxProvider;
      this.collidingScanRegionFilter = PerceptionFilterTools.createHumanoidShinCollisionFilter(fullRobotModel, collisionBoxProvider);
   }

   public void initializeLocalizationAndMappingThread(HumanoidReferenceFrames referenceFrames, String robotName, ROS2Node ros2Node, boolean smoothing)
   {
      LogTools.info("Initializing Localization and Mapping Process (Smoothing: {})", smoothing);
      localizationAndMappingTask = new LocalizationAndMappingTask(robotName,
                                                                  PerceptionAPI.PERSPECTIVE_RAPID_REGIONS,
                                                                  PerceptionAPI.SPHERICAL_RAPID_REGIONS_WITH_POSE,
                                                                  ros2Node,
                                                                  referenceFrames,
                                                                  () ->
                                                                        {
                                                                        },
                                                                  smoothing);
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

   public void extractOccupancyGrid(List<Point3D> pointCloud,
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
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }

      if (rapidPlanarRegionsExtractor != null)
         rapidPlanarRegionsExtractor.destroy();

      if (localizationAndMappingTask != null)
         localizationAndMappingTask.destroy();
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
