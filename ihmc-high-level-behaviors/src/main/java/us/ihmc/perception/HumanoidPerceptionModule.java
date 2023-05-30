package us.ihmc.perception;

import controller_msgs.msg.dds.RobotConfigurationData;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.filters.CollidingScanRegionFilter;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractor;
import us.ihmc.perception.tools.PerceptionFilterTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;

public class HumanoidPerceptionModule
{
   private final PlanarRegionMappingHandler mapHandler = new PlanarRegionMappingHandler();

   private BytedecoImage bytedecoDepthImage;
   private OpenCLManager openCLManager;
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

   public void update(Mat depthImage, ReferenceFrame cameraFrame, boolean enabled)
   {
      if (enabled)
      {
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

         if (mapHandler.isEnabled())
         {
            mapHandler.updateMapWithNewRegions(this.sensorFrameRegions);
         }
      }
   }

   public void initializePerspectiveRapidRegionsExtractor(CameraIntrinsics cameraIntrinsics)
   {
      LogTools.info("Initializing Perspective Rapid Regions: {}", cameraIntrinsics);

      this.sensorFrameRegions = new FramePlanarRegionsList();
      this.bytedecoDepthImage = new BytedecoImage(cameraIntrinsics.getWidth(), cameraIntrinsics.getHeight(), opencv_core.CV_16UC1);
      this.bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      this.rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor();
      this.rapidPlanarRegionsExtractor.create(openCLManager, cameraIntrinsics.getHeight(), cameraIntrinsics.getWidth(),
                                              cameraIntrinsics.getFx(), cameraIntrinsics.getFy(), cameraIntrinsics.getCx(),
                                              cameraIntrinsics.getCy());

      this.rapidPlanarRegionsExtractor.getDebugger().setEnabled(false);
   }

   public void initializeBodyCollisionFilter(FullHumanoidRobotModel fullRobotModel, CollisionBoxProvider collisionBoxProvider)
   {
      this.fullRobotModel = fullRobotModel;
      this.collisionBoxProvider = collisionBoxProvider;
      this.collidingScanRegionFilter = PerceptionFilterTools.createHumanoidShinCollisionFilter(fullRobotModel, collisionBoxProvider);
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

   public PlanarRegionMappingHandler getMapHandler()
   {
      return this.mapHandler;
   }
}
