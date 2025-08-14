package us.ihmc.rdx.perception;

import com.google.common.util.concurrent.ThreadFactoryBuilder;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.footstepPlanning.SnappingTerrainManager;
import us.ihmc.footstepPlanning.steppableRegions.SteppableRegionsManager;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.perception.ROS2ImageSensors;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapManager;
import us.ihmc.perception.heightMap.HeightMapParameters;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedImageSensor;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ZEDImageSensor;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

public class RDXSteppableRegionCalculatorDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ROS2Node ros2Node;
   private final BlockingQueue<RawImage> rawImageCollection;
   private final RapidHeightMapManager rapidHeightMapManager;
   private final RDXSimulatedImageSensor zedImageSensor;
   private final SnappingTerrainManager snappingTerrainManager;
   private final SteppableRegionsManager steppableRegionsManager;

   public RDXSteppableRegionCalculatorDemo()
   {
      ros2Node = new ROS2NodeBuilder().build("steppable_regions_demo");

      // World frame reference
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      // Experimental camera frame: +1m forward (x), +1m up (z)
      ReferenceFrame experimentalCameraFrame = new ReferenceFrame("experimentalCameraFrame", worldFrame, false, true)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setIdentity();
            transformToParent.getTranslation().set(1.0, 0.0, 1.0);
         }
      };

      // Height map center: same x/y as camera, z=0
      ReferenceFrame heightMapCenter = new ReferenceFrame("heightMapCenter", worldFrame, false, true)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setIdentity();
            transformToParent.getTranslation().set(1.0, 0.0, 0.0);
         }
      };

      // Left foot frame: same x/y as camera, z=0, +0.12m in y
      ReferenceFrame leftFootFrame = new ReferenceFrame("leftFootFrame", worldFrame, false, true)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setIdentity();
            transformToParent.getTranslation().set(1.0, 0.12, 0.0);
         }
      };

      // Right foot frame: same x/y as camera, z=0, -0.12m in y
      ReferenceFrame rightFootFrame = new ReferenceFrame("rightFootFrame", worldFrame, false, true)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setIdentity();
            transformToParent.getTranslation().set(1.0, -0.12, 0.0);
         }
      };

      // Important: Update transforms after creation
      experimentalCameraFrame.update();
      heightMapCenter.update();
      leftFootFrame.update();
      rightFootFrame.update();

      ROS2ImageSensors ros2ImageSensors = new ROS2ImageSensors(ros2Node);
      zedImageSensor = RDXSimulatedSensorFactory.createZED2iImageSensor();
      ros2ImageSensors.addZEDSensor(zedImageSensor, experimentalCameraFrame);
      rawImageCollection = new LinkedBlockingQueue<>();
      ros2ImageSensors.registerImageQueueForZED(rawImageCollection, ZEDImageSensor.DEPTH_IMAGE_KEY);

      HeightMapParameters heightMapParameters = new HeightMapParameters();
      ControllerFootstepQueueMonitor controllerFootstepQueueMonitor = new ControllerFootstepQueueMonitor(ros2Node, "what");
      rapidHeightMapManager = new RapidHeightMapManager(ros2Node,
                                                        leftFootFrame,
                                                        rightFootFrame,
                                                        heightMapCenter,
                                                        controllerFootstepQueueMonitor,
                                                        heightMapParameters);

      snappingTerrainManager = new SnappingTerrainManager(ros2Node, heightMapParameters);
      steppableRegionsManager = new SteppableRegionsManager(ros2Node);

      ThreadFactory threadFactory = new ThreadFactoryBuilder().setNameFormat("STEPPABLE_REGIONS_UPDATE").build();
      ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1, threadFactory);
      scheduler.scheduleAtFixedRate(this::updateAlgorithms, 500, 100, TimeUnit.MILLISECONDS);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(3.0, 1.0, 2.5);

            zedImageSensor.create(baseUI.getPrimaryScene());
            zedImageSensor.run(true);
         }

         @Override
         public void render()
         {
            zedImageSensor.render();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            zedImageSensor.close();
            rapidHeightMapManager.destroy();
            snappingTerrainManager.close();
            steppableRegionsManager.destroy();
            ros2Node.destroy();
            baseUI.dispose();
         }
      });
   }

   public void updateAlgorithms()
   {
      try
      {
         RawImage depthImage = rawImageCollection.take();

         RigidBodyTransformReadOnly transformToWorld = depthImage.getTransformToWorld();
         ReferenceFrame cameraFrameInWorld = new FixedReferenceFrame("FrameInWorld", ReferenceFrame.getWorldFrame(), transformToWorld);
         ZUpFrame cameraZUpFrameInWorld = new ZUpFrame(cameraFrameInWorld, "ZUpFrameInWorld");
         // Need to update this due to how its implemented, other the transform to world will be all zeros
         cameraZUpFrameInWorld.update();

         GpuMat latestDepthImage = depthImage.getGpuImageMat();
         CameraIntrinsics depthIntrinsicsCopy = depthImage.getIntrinsicsCopy();
         rapidHeightMapManager.updateAndPublish(latestDepthImage, depthIntrinsicsCopy, cameraFrameInWorld, cameraZUpFrameInWorld);
         snappingTerrainManager.updateAndPublish(rapidHeightMapManager.getLatestHeightMapData());

         // This is what we are trying to test, the rest is just to get here...
         steppableRegionsManager.update(snappingTerrainManager.getTerrainMapData());
      }
      catch (InterruptedException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static void main(String[] args)
   {
      new RDXSteppableRegionCalculatorDemo();
   }
}
