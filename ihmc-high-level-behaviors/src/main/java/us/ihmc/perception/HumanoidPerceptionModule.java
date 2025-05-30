package us.ihmc.perception;

import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.GlobalMapTileMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.timing.PerceptionStatistics;
import us.ihmc.perception.tools.PerceptionFilterTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.perception.globalHeightMap.GlobalHeightMap;
import us.ihmc.perception.globalHeightMap.GlobalMapTile;
import us.ihmc.perception.heightMap.HeightMapMessageTools;
import us.ihmc.perception.heightMap.HeightMapTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.time.Instant;
import java.util.Collection;
import java.util.List;

public class HumanoidPerceptionModule
{
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 16);
   private final FramePose3D lidarPose = new FramePose3D();
   private final OpenCLManager openCLManager;

   private PerceptionConfigurationParameters perceptionConfigurationParameters;
   private LocalizationAndMappingTask localizationAndMappingTask;
   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private CollidingScanRegionFilter collidingScanRegionFilter;
   private FullHumanoidRobotModel fullRobotModel;
   private PlanarRegionsList regionsInSensorFrame;
   private PlanarRegionsList regionsInWorldFrame;
   private CollisionBoxProvider collisionBoxProvider;
   private FramePlanarRegionsList sensorFrameRegions;
   private BytedecoImage realsenseDepthImage;
   private GpuMat deviceDepthImage;

   private final GlobalHeightMap globalHeightMap = new GlobalHeightMap();
   private final PerceptionStatistics perceptionStatistics = new PerceptionStatistics();

   private boolean rapidRegionsEnabled = false;
   private boolean sphericalRegionsEnabled = false;
   private boolean heightMapEnabled = false;
   private boolean mappingEnabled = false;
   private boolean occupancyGridEnabled = false;

   public HumanoidPerceptionModule(OpenCLManager openCLManager)
   {
      this.openCLManager = openCLManager;
   }

   public void updateTerrain(ROS2Helper ros2Helper, Mat incomingDepth, ReferenceFrame cameraFrame, ReferenceFrame cameraZUpFrame, boolean metricDepth)
   {
      if (localizationAndMappingTask != null)
         localizationAndMappingTask.setEnableLiveMode(mappingEnabled);

      executorService.clearTaskQueue();

      if (rapidRegionsEnabled)
      {
         executorService.submit(() ->
                                {
                                   updatePlanarRegions(ros2Helper, cameraFrame);
                                });
      }
   }

   private static void publishGlobalHeightMapTile(ROS2Helper ros2Helper,
                                                  GlobalHeightMap globalHeightMap,
                                                  Instant acquisitionTime,
                                                  ROS2Topic<GlobalMapTileMessage> topic)
   {
      // Get tiles (made out of modified cells) from the global height map class and publish them in a for loop
      Collection<GlobalMapTile> modifiedCells = globalHeightMap.getModifiedMapTiles();
      for (GlobalMapTile tile : modifiedCells)
      {
         GlobalMapTileMessage globalMapTileMessage = new GlobalMapTileMessage();
         packGlobalMapTileMessage(globalMapTileMessage, tile);
         ros2Helper.publish(topic, globalMapTileMessage);
      }
   }

   private static void packGlobalMapTileMessage(GlobalMapTileMessage messageToPack, GlobalMapTile tile)
   {
      messageToPack.setCenterX(tile.getCenterX());
      messageToPack.setCenterY(tile.getCenterY());
      messageToPack.setHashCodeOfTile(tile.hashCode());
      messageToPack.getHeightMap().set(HeightMapMessageTools.toMessage(tile));
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

   public void initializeBodyCollisionFilter(FullHumanoidRobotModel fullRobotModel, CollisionBoxProvider collisionBoxProvider)
   {
      LogTools.info("Initializing Body Collision Filter");
      this.fullRobotModel = fullRobotModel;
      this.collisionBoxProvider = collisionBoxProvider;
      this.collidingScanRegionFilter = PerceptionFilterTools.createHumanoidShinCollisionFilter(fullRobotModel, collisionBoxProvider);
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

   public RapidPlanarRegionsExtractor getRapidRegionsExtractor()
   {
      return this.rapidPlanarRegionsExtractor;
   }

   public void setPerceptionConfigurationParameters(PerceptionConfigurationParameters perceptionConfigurationParameters)
   {
      this.perceptionConfigurationParameters = perceptionConfigurationParameters;
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
