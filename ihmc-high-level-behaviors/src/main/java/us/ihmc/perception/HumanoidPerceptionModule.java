package us.ihmc.perception;

import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.timing.PerceptionStatistics;
import us.ihmc.perception.tools.PerceptionFilterTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public class HumanoidPerceptionModule
{
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 16);
   private final OpenCLManager openCLManager;

   private PerceptionConfigurationParameters perceptionConfigurationParameters;
   private LocalizationAndMappingTask localizationAndMappingTask;
   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
   private CollidingScanRegionFilter collidingScanRegionFilter;
   private PlanarRegionsList regionsInSensorFrame;
   private PlanarRegionsList regionsInWorldFrame;
   private FramePlanarRegionsList sensorFrameRegions;

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

   private void updatePlanarRegions(ROS2Helper ros2Helper, ReferenceFrame cameraFrame)
   {
      long begin = System.nanoTime();
      filterFramePlanarRegionsList();
      perceptionStatistics.updateTimeToComputeRapidRegions((System.nanoTime() - begin) * 1e-6f);
      PerceptionMessageTools.publishFramePlanarRegionsList(sensorFrameRegions, PerceptionAPI.PERSPECTIVE_RAPID_REGIONS, ros2Helper);
      perceptionStatistics.updateTimeToComputeRapidRegions((System.nanoTime() - begin) * 1e-6f);
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

   public void filterFramePlanarRegionsList()
   {
      this.collidingScanRegionFilter.update();

      synchronized (sensorFrameRegions)
      {
         PerceptionFilterTools.filterCollidingPlanarRegions(sensorFrameRegions, this.collidingScanRegionFilter);
      }
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
