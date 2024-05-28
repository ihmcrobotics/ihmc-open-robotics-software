package us.ihmc.perception.headless;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.heightMap.RemoteHeightMapUpdater;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.timing.PerceptionStatistics;
import us.ihmc.perception.tools.PerceptionFilterTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.perception.gpuHeightMap.HeightMapTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.time.Instant;
import java.util.List;
import java.util.function.Supplier;

public class HumanoidPerceptionModule
{
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 16);
   private final BytePointer compressedDepthPointer = new BytePointer();
   private final FramePose3D cameraPose = new FramePose3D();
   private final FramePose3D lidarPose = new FramePose3D();
   private final OpenCLManager openCLManager;

   private final ImageMessage croppedHeightMapImageMessage = new ImageMessage();
   private final ImageMessage localHeightMapImageMessage = new ImageMessage();
   private final ImageMessage globalHeightMapImageMessage = new ImageMessage();

   private final BytePointer compressedCroppedHeightMapPointer = new BytePointer();
   private final BytePointer compressedLocalHeightMapPointer = new BytePointer();
   private final BytePointer compressedInternalHeightMapPointer = new BytePointer();

   private RemoteHeightMapUpdater heightMap;
   private PerceptionConfigurationParameters perceptionConfigurationParameters;
   private LocalizationAndMappingTask localizationAndMappingTask;
   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private RapidHeightMapExtractor rapidHeightMapExtractor;
   private CollidingScanRegionFilter collidingScanRegionFilter;
   private FullHumanoidRobotModel fullRobotModel;
   private PlanarRegionsList regionsInSensorFrame;
   private PlanarRegionsList regionsInWorldFrame;
   private CollisionBoxProvider collisionBoxProvider;
   private FramePlanarRegionsList sensorFrameRegions;
   private HeightMapData latestHeightMapData;
   private BytedecoImage realsenseDepthImage;

   private final PerceptionStatistics perceptionStatistics = new PerceptionStatistics();

   private boolean rapidRegionsEnabled = false;
   private boolean sphericalRegionsEnabled = false;
   private boolean heightMapEnabled = false;
   private boolean mappingEnabled = false;
   private boolean occupancyGridEnabled = false;
   public boolean heightMapDataBeingProcessed = false;

   public HumanoidPerceptionModule(OpenCLManager openCLManager)
   {
      this.openCLManager = openCLManager;
   }

   public void initializeRealsenseDepthImage(int height, int width)
   {
      this.realsenseDepthImage = new BytedecoImage(width, height, opencv_core.CV_16UC1);
      this.realsenseDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
   }

   public void initializeHeightMapUpdater(String robotName, Supplier<ReferenceFrame> frameSupplier, RealtimeROS2Node realtimeRos2Node)
   {
      heightMap = new RemoteHeightMapUpdater(robotName, frameSupplier, realtimeRos2Node);
      heightMap.start();
   }

   public void setIsHeightMapDataBeingProcessed(boolean dataBeingProcessed)
   {
      heightMapDataBeingProcessed = dataBeingProcessed;
   }

   public boolean isHeightMapDataBeingProcessed()
   {
      return heightMapDataBeingProcessed;
   }

   public void updateTerrain(ROS2Helper ros2Helper,
                             Mat incomingDepth,
                             ReferenceFrame cameraFrame,
                             ReferenceFrame cameraZUpFrame,
                             boolean metricDepth)
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
            incomingDepth.convertTo(realsenseDepthImage.getBytedecoOpenCVMat(), opencv_core.CV_16UC1);
         }
      }

      executorService.clearTaskQueue();

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
                                   if (!heightMapDataBeingProcessed)
                                   {
                                      if (rapidHeightMapExtractor.getHeightMapParameters().getResetHeightMap())
                                      {
                                         rapidHeightMapExtractor.reset();
                                      }
                                      updateRapidHeightMap(ros2Helper, cameraFrame, cameraZUpFrame);
                                   }

                                   Instant acquisitionTime = Instant.now();
                                   Mat croppedHeightMapImage = rapidHeightMapExtractor.getTerrainMapData().getHeightMap();

                                   if (ros2Helper != null)
                                   {
                                      publishHeightMapImage(ros2Helper,
                                                            croppedHeightMapImage,
                                                            compressedCroppedHeightMapPointer,
                                                            PerceptionAPI.HEIGHT_MAP_CROPPED,
                                                            croppedHeightMapImageMessage,
                                                            acquisitionTime);
                                      //               publishHeightMapImage(ros2Helper, localHeightMapImage, compressedLocalHeightMapPointer, PerceptionAPI.HEIGHT_MAP_LOCAL,
                                      //                                     localHeightMapImageMessage, acquisitionTime);
                                      //               publishHeightMapImage(ros2Helper, globalHeightMapImage, compressedInternalHeightMapPointer, PerceptionAPI.HEIGHT_MAP_GLOBAL,
                                      //                                     globalHeightMapImageMessage, acquisitionTime);
                                   }
                                });
      }
   }

   public void publishHeightMapImage(ROS2Helper ros2Helper,
                                     Mat image,
                                     BytePointer pointer,
                                     ROS2Topic<ImageMessage> topic,
                                     ImageMessage message,
                                     Instant acquisitionTime)
   {
      OpenCVTools.compressImagePNG(image, pointer);
      PerceptionMessageTools.publishCompressedDepthImage(pointer,
                                                         topic,
                                                         message,
                                                         ros2Helper,
                                                         cameraPose,
                                                         acquisitionTime,
                                                         rapidHeightMapExtractor.getSequenceNumber(),
                                                         image.rows(),
                                                         image.cols(),
                                                         (float) RapidHeightMapExtractor.getHeightMapParameters().getHeightScaleFactor());
   }

   public void publishExternalHeightMapImage(ROS2Helper ros2Helper)
   {
      executorService.clearTaskQueue();
      executorService.submit(() ->
        {
           Instant acquisitionTime = Instant.now();
           Mat heightMapImage = rapidHeightMapExtractor.getInternalGlobalHeightMapImage().getBytedecoOpenCVMat();
           OpenCVTools.compressImagePNG(heightMapImage, compressedInternalHeightMapPointer);
           //PerceptionDebugTools.displayDepth("Published Global Height Map", heightMapImage, 1);
           PerceptionMessageTools.publishCompressedDepthImage(compressedInternalHeightMapPointer,
                                                              PerceptionAPI.HEIGHT_MAP_CROPPED,
                                                              croppedHeightMapImageMessage,
                                                              ros2Helper,
                                                              new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                                              rapidHeightMapExtractor.getSensorOrigin(),
                                                                              new Quaternion()),
                                                              acquisitionTime,
                                                              rapidHeightMapExtractor.getSequenceNumber(),
                                                              heightMapImage.rows(),
                                                              heightMapImage.cols(),
                                                              (float) RapidHeightMapExtractor.getHeightMapParameters().getHeightScaleFactor());
        });
   }

   private void updatePlanarRegions(ROS2Helper ros2Helper, ReferenceFrame cameraFrame)
   {
      long begin = System.nanoTime();
      extractFramePlanarRegionsList(rapidPlanarRegionsExtractor, realsenseDepthImage, sensorFrameRegions, cameraFrame);
      filterFramePlanarRegionsList();
      perceptionStatistics.updateTimeToComputeRapidRegions((System.nanoTime() - begin) * 1e-6f);
      PerceptionMessageTools.publishFramePlanarRegionsList(sensorFrameRegions, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS, ros2Helper);
      perceptionStatistics.updateTimeToComputeRapidRegions((System.nanoTime() - begin) * 1e-6f);
   }

   private void updateRapidHeightMap(ROS2Helper ros2Helper, ReferenceFrame cameraFrame, ReferenceFrame cameraZUpFrame)
   {
      RigidBodyTransform sensorToWorld = cameraFrame.getTransformToWorldFrame();
      RigidBodyTransform sensorToGround = cameraFrame.getTransformToDesiredFrame(cameraZUpFrame);
      RigidBodyTransform groundToWorld = cameraZUpFrame.getTransformToWorldFrame();

      cameraPose.setToZero(cameraFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      long begin = System.nanoTime();
      rapidHeightMapExtractor.update(sensorToWorld, sensorToGround, groundToWorld);
      perceptionStatistics.updateTimeToComputeHeightMap((System.nanoTime() - begin) * 1e-6f);
   }

   public void updateStructural(ROS2Helper ros2Helper, List<Point3D> pointCloud, ReferenceFrame sensorFrame, Mat occupancy, float thresholdHeight)
   {
      lidarPose.setToZero(sensorFrame);
      lidarPose.changeFrame(ReferenceFrame.getWorldFrame());

      if (occupancyGridEnabled)
      {
         executorService.clearQueueAndExecute(() ->
                                              {
                                                 extractOccupancyGrid(pointCloud,
                                                                      occupancy,
                                                                      sensorFrame.getTransformToWorldFrame(),
                                                                      thresholdHeight,
                                                                      perceptionConfigurationParameters.getOccupancyGridResolution(),
                                                                      70);
                                              });
      }

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

   public void initializeHeightMapExtractor(HumanoidReferenceFrames referenceFrames, CameraIntrinsics cameraIntrinsics)
   {
      LogTools.info("Rapid Height Map: {}", cameraIntrinsics);
      rapidHeightMapExtractor = new RapidHeightMapExtractor(openCLManager, referenceFrames);
      rapidHeightMapExtractor.setDepthIntrinsics(cameraIntrinsics);
      rapidHeightMapExtractor.create(realsenseDepthImage, 1);
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
                                             ReferenceFrame cameraFrame)
   {
      extractor.update(depthImage, cameraFrame, sensorFrameRegions);
      extractor.setProcessing(false);

      regionsInSensorFrame = sensorFrameRegions.getPlanarRegionsList();
      regionsInWorldFrame = regionsInSensorFrame.copy();
      regionsInWorldFrame.applyTransform(cameraFrame.getTransformToWorldFrame());
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

         int gridX = HeightMapTools.getIndexFromCoordinates(point.getX(), occupancyGridResolution, offset);
         int gridY = HeightMapTools.getIndexFromCoordinates(point.getY(), occupancyGridResolution, offset);

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
      executorService.clearTaskQueue();
      executorService.destroy();

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

   public HeightMapMessage getGlobalHeightMapMessage()
   {
      BytedecoImage heightMapImage = rapidHeightMapExtractor.getInternalGlobalHeightMapImage();
      Mat heightMapMat = heightMapImage.getBytedecoOpenCVMat().clone();
      if (latestHeightMapData == null)
      {
         latestHeightMapData = new HeightMapData((float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(),
                                                 (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters(),
                                                 rapidHeightMapExtractor.getSensorOrigin().getX(),
                                                 rapidHeightMapExtractor.getSensorOrigin().getY());
      }
      PerceptionMessageTools.convertToHeightMapData(heightMapMat,
                                                    latestHeightMapData,
                                                    rapidHeightMapExtractor.getSensorOrigin(),
                                                    (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters(),
                                                    (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters());
      return HeightMapMessageTools.toMessage(latestHeightMapData);
   }

   public HeightMapData getLatestHeightMapData()
   {
      Mat heightMapMat = rapidHeightMapExtractor.getTerrainMapData().getHeightMap();
      if (latestHeightMapData == null)
      {
         latestHeightMapData = new HeightMapData((float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(),
                                                 (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters(),
                                                 rapidHeightMapExtractor.getSensorOrigin().getX(),
                                                 rapidHeightMapExtractor.getSensorOrigin().getY());
      }
      PerceptionMessageTools.convertToHeightMapData(heightMapMat,
                                                    latestHeightMapData,
                                                    rapidHeightMapExtractor.getSensorOrigin(),
                                                    (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters(),
                                                    (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters());
      return latestHeightMapData;
   }

   public void setRapidRegionsEnabled(boolean rapidRegionsEnabled)
   {
      this.rapidRegionsEnabled = rapidRegionsEnabled;
   }

   public void setHeightMapEnabled(boolean heightMapEnabled)
   {
      this.heightMapEnabled = heightMapEnabled;
   }

   public void setMappingEnabled(boolean mappingEnabled)
   {
      this.mappingEnabled = mappingEnabled;
   }

   public void setSphericalRegionsEnabled(boolean sphericalRegionsEnabled)
   {
      this.sphericalRegionsEnabled = sphericalRegionsEnabled;
   }

   public PerceptionStatistics getPerceptionStatistics()
   {
      return perceptionStatistics;
   }
}
