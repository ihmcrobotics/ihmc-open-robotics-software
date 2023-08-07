package us.ihmc.perception;

import controller_msgs.msg.dds.RobotConfigurationData;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.activeMapping.ActiveMappingRemoteProcess;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.headless.LocalizationAndMappingProcess;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.perception.tools.PerceptionFilterTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;

import java.time.Instant;
import java.util.ArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static org.bytedeco.opencv.global.opencv_imgproc.COLOR_GRAY2RGB;

public class HumanoidPerceptionModule
{
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));

   private BytedecoImage realsenseDepthImage;
   private BytedecoImage ousterDepthImage;
   private Mat occupancyGrid;
   private Mat gridColor = new Mat();
   private BytePointer compressedOccupancyGrid;
   private ImageMessage occupancyGridMessage;

   private final FramePose3D cameraPose = new FramePose3D();

   private OpenCLManager openCLManager;
   private LocalizationAndMappingProcess localizationAndMappingProcess;
   private ActiveMappingRemoteProcess activeMappingRemoteProcess;
   private RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor;
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

   public void subscribeToRobotData(String robotName, ROS2Node ros2Node)
   {
      this.robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
      this.robotConfigurationData = new RobotConfigurationData();
      ROS2Tools.createCallbackSubscriptionTypeNamed(ros2Node, RobotConfigurationData.class, ROS2Tools.getRobotConfigurationDataTopic(robotName), s ->
      {
         s.takeNextData(robotConfigurationData, null);
      });
   }

   public void updateTerrain(ROS2Helper ros2Helper, Mat depthImage, ReferenceFrame cameraFrame, boolean rapidRegionsEnabled, boolean mappingEnabled)
   {
      if (localizationAndMappingProcess != null)
         localizationAndMappingProcess.setEnableLiveMode(mappingEnabled);

      if (rapidRegionsEnabled)
      {
         executorService.submit(() ->
           {
              if (robotConfigurationData != null && robotConfigurationData.getJointNameHash() != 0)
              {
                 robotConfigurationDataBuffer.update(robotConfigurationData);
                 long newestTimestamp = robotConfigurationDataBuffer.getNewestTimestamp();
                 long selectedTimestamp = robotConfigurationDataBuffer.updateFullRobotModel(waitIfNecessary,
                                                                                            newestTimestamp,
                                                                                            this.fullRobotModel,
                                                                                            null);
              }

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
           });
      }
   }

   public void updateStructural(ROS2Helper ros2Helper, ArrayList<Point3D> pointCloud, ReferenceFrame sensorFrame, float thresholdHeight)
   {
//      executorService.submit(() ->
        {
           Instant acquisitionTime = Instant.now();

           cameraPose.setToZero(sensorFrame);
           cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

           occupancyGrid.put(new Scalar(0));
           extractOccupancyGrid(pointCloud, occupancyGrid, sensorFrame.getTransformToWorldFrame(), thresholdHeight,
                                perceptionConfigurationParameters.getOccupancyGridResolution());



           opencv_imgproc.cvtColor(occupancyGrid, gridColor, COLOR_GRAY2RGB);
           PerceptionDebugTools.display("Occupancy Grid", gridColor, 1, 1400);

//           OpenCVTools.compressImagePNG(occupancyGrid, compressedOccupancyGrid);
//           CameraModel.PINHOLE.packMessageFormat(occupancyGridMessage);
//           PerceptionMessageTools.publishCompressedDepthImage(compressedOccupancyGrid,
//                                                              PerceptionAPI.OCCUPANCY_GRID_MESSAGE,
//                                                              occupancyGridMessage,
//                                                              ros2Helper,
//                                                              cameraPose,
//                                                              acquisitionTime,
//                                                              0,
//                                                              ousterDepthImage.getImageHeight(),
//                                                              ousterDepthImage.getImageWidth(),
//                                                              (float) 0.05f);
        }
//        );
   }

   public void initializePerspectiveRapidRegionsExtractor(CameraIntrinsics cameraIntrinsics)
   {
      LogTools.info("Initializing Perspective Rapid Regions: {}", cameraIntrinsics);

      this.sensorFrameRegions = new FramePlanarRegionsList();
      this.realsenseDepthImage = new BytedecoImage(cameraIntrinsics.getWidth(), cameraIntrinsics.getHeight(), opencv_core.CV_16UC1);
      this.realsenseDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      this.rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor(openCLManager,
                                                                         cameraIntrinsics.getHeight(),
                                                                         cameraIntrinsics.getWidth(),
                                                                         cameraIntrinsics.getFx(),
                                                                         cameraIntrinsics.getFy(),
                                                                         cameraIntrinsics.getCx(),
                                                                         cameraIntrinsics.getCy());

      this.rapidPlanarRegionsExtractor.getDebugger().setEnabled(false);
   }

   public void initializeOccupancyGrid(int depthHeight, int depthWidth, int gridHeight, int gridWidth)
   {
      LogTools.info("Initializing Occupancy Grid");
      this.ousterDepthImage = new BytedecoImage(depthWidth, depthHeight, opencv_core.CV_16UC1);
      this.occupancyGrid = new Mat(gridHeight, gridWidth, opencv_core.CV_8UC1, new Scalar(0));
      compressedOccupancyGrid = new BytePointer(); // deallocate later
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
                                                                        ros2Node, syncedRobot.getReferenceFrames(), () -> {}, smoothing);
   }

   public void initializeActiveMappingProcess(String robotName, DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobot, ROS2Node ros2Node)
   {
      LogTools.info("Initializing Active Mapping Process");
      activeMappingRemoteProcess = new ActiveMappingRemoteProcess(robotName, robotModel, syncedRobot,
                                                                  PerceptionAPI.PERSPECTIVE_RAPID_REGIONS,
                                                                  PerceptionAPI.SPHERICAL_RAPID_REGIONS_WITH_POSE,
                                                                  ros2Node, syncedRobot.getReferenceFrames(), () -> {}, true);
   }

   public void extractFramePlanarRegionsList(RapidPlanarRegionsExtractor extractor, BytedecoImage depthImage, FramePlanarRegionsList sensorFrameRegions,
                                             PlanarRegionsList worldRegions, PlanarRegionsList sensorRegions, ReferenceFrame cameraFrame)
   {
      extractor.update(depthImage, cameraFrame, sensorFrameRegions);
      extractor.setProcessing(false);

      sensorRegions = sensorFrameRegions.getPlanarRegionsList();
      worldRegions = sensorRegions.copy();
      worldRegions.applyTransform(cameraFrame.getTransformToWorldFrame());
   }

   public void extractOccupancyGrid(ArrayList<Point3D> pointCloud, Mat occupancyGrid, RigidBodyTransform sensorToWorldTransform, float thresholdHeight,
                                    float occupancyGridResolution)
   {
      for (int i = 0; i<pointCloud.size(); i++)
      {
         Point3D point = pointCloud.get(i);
         //sensorToWorldTransform.transform(point);

         int gridX = (int) (point.getX() * occupancyGridResolution + 70);
         int gridY = (int) (point.getY() * occupancyGridResolution + 70);

         if (point.getZ() > thresholdHeight && gridX >= 0 && gridX < occupancyGrid.cols() && gridY >= 0 && gridY < occupancyGrid.rows())
         {
            occupancyGrid.ptr(gridY, gridX).putInt(100);
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

   public void setPerceptionConfigurationParameters(PerceptionConfigurationParameters perceptionConfigurationParameters)
   {
      this.perceptionConfigurationParameters = perceptionConfigurationParameters;
   }
}
