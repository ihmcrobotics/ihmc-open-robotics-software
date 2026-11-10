package us.ihmc.behaviors.activeMapping;

import com.google.common.util.concurrent.ThreadFactoryBuilder;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2TunedRigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.perception.GpuMappingThread;
import us.ihmc.perception.ROS2ImageSensors;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractionThread;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Runs the perception side of active mapping in-process, for simulation UIs: the GPU height map and terrain map from
 * the "Stepping Camera" depth images, rapid planar regions from the "Experimental Camera" depth images, the stepping
 * camera transform tuning, and the ROS 2 sync of the mapping parameters (see RDXPerceptionParametersPanel).
 */
public class ActiveMappingProcess
{
   public static final String STEPPING_CAMERA_NAME = "Stepping Camera";
   public static final String EXPERIMENTAL_CAMERA_NAME = "Experimental Camera";

   private final ActiveMappingParameterToolBox activeMappingParameterToolBox;
   private final RapidPlanarRegionsExtractionThread rapidPlanarRegionsExtractionThread;
   private final GpuMappingThread gpuMappingThread;
   private final ScheduledExecutorService syncedRobotScheduler;
   private final ScheduledExecutorService sensorTransformScheduler;
   private final ScheduledExecutorService parameterScheduler;

   public ActiveMappingProcess(DRCRobotModel robotModel,
                               RobotCollisionModel robotCollisionModel,
                               ROS2Node ros2Node,
                               ROS2ImageSensors ros2ImageSensors,
                               ROS2SyncedRobotModel ros2SyncedRobot)
   {
      ros2SyncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);
      ControllerFootstepQueueMonitor controllerFootstepQueueMonitor = new ControllerFootstepQueueMonitor(ros2Node, robotModel.getSimpleRobotName());

      activeMappingParameterToolBox = new ActiveMappingParameterToolBox(ros2Node);

      // Lets the stepping camera transform be tuned from the UI, needed for calibrating the sensor
      ROS2TunedRigidBodyTransform steppingCameraTunableTransform
            = ROS2TunedRigidBodyTransform.toBeTuned(ros2Node,
                                                    PerceptionAPI.STEPPING_CAMERA_TO_PARENT_TUNING,
                                                    robotModel.getSensorInformation().getSteppingCameraTransform());

      BlockingQueue<RawImage> steppingCameraDepthImages = new LinkedBlockingQueue<>(ImageSensor.DEFAULT_IMAGE_QUEUE_CAPACITY);
      BlockingQueue<RawImage> experimentalCameraDepthImages = new LinkedBlockingQueue<>(ImageSensor.DEFAULT_IMAGE_QUEUE_CAPACITY);
      if (ros2ImageSensors.getSensor(STEPPING_CAMERA_NAME) != null)
         ros2ImageSensors.getSensor(STEPPING_CAMERA_NAME).registerImageQueue(steppingCameraDepthImages, ZEDImageSensor.DEPTH_IMAGE_KEY);
      if (ros2ImageSensors.getSensor(EXPERIMENTAL_CAMERA_NAME) != null)
         ros2ImageSensors.getSensor(EXPERIMENTAL_CAMERA_NAME).registerImageQueue(experimentalCameraDepthImages, ZEDImageSensor.DEPTH_IMAGE_KEY);

      gpuMappingThread = new GpuMappingThread(ros2Node,
                                              ros2SyncedRobot,
                                              robotCollisionModel,
                                              steppingCameraDepthImages,
                                              controllerFootstepQueueMonitor,
                                              activeMappingParameterToolBox);
      rapidPlanarRegionsExtractionThread = new RapidPlanarRegionsExtractionThread(ros2Node, experimentalCameraDepthImages);

      gpuMappingThread.startRepeating();
      rapidPlanarRegionsExtractionThread.startRepeating();

      // Named threads so they are easy to identify when profiling
      syncedRobotScheduler = Executors.newScheduledThreadPool(1, new ThreadFactoryBuilder().setNameFormat("SyncedRobotThread").build());
      syncedRobotScheduler.scheduleAtFixedRate(ros2SyncedRobot::update, 100, 10, TimeUnit.MILLISECONDS);

      sensorTransformScheduler = Executors.newScheduledThreadPool(1, new ThreadFactoryBuilder().setNameFormat("SensorTunableTransformUpdateThread").build());
      sensorTransformScheduler.scheduleAtFixedRate(steppingCameraTunableTransform::update, 0, 33, TimeUnit.MILLISECONDS);

      parameterScheduler = Executors.newScheduledThreadPool(1, new ThreadFactoryBuilder().setNameFormat("HeightMapParametersThread").build());
      parameterScheduler.scheduleWithFixedDelay(activeMappingParameterToolBox::update, 500, 100, TimeUnit.MILLISECONDS);
   }

   public void destroy()
   {
      parameterScheduler.shutdownNow();
      sensorTransformScheduler.shutdownNow();
      syncedRobotScheduler.shutdownNow();
      gpuMappingThread.blockingKill();
      rapidPlanarRegionsExtractionThread.kill();
   }
}
