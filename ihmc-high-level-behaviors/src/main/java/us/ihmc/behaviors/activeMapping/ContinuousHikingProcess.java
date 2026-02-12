package us.ihmc.behaviors.activeMapping;

import com.google.common.util.concurrent.ThreadFactoryBuilder;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2TunedRigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.perception.GpuMappingThread;
import us.ihmc.perception.ROS2ImageSensors;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.rapidRegions.RapidPlanarRegionsExtractionThread;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

public class ContinuousHikingProcess
{
   public static final String CONTINUOUS_HIKING_THREAD = "ContinuousHikingThread";
   public static final String SYNCED_ROBOT_THREAD = "SyncedRobotThread";
   public static final String SENSOR_TUNABLE_TRANSFORM_UPDATE_THREAD = "SensorTunableTransformUpdateThread";

   private final ActiveMappingParameterToolBox activeMappingParameterToolBox;
   private final ContinuousPlanningStateMachine continuousPlanningStateMachine;
   private final RapidPlanarRegionsExtractionThread rapidPlanarRegionsExtractionThread;
   private final GpuMappingThread gpuMappingThread;

   public ContinuousHikingProcess(DRCRobotModel robotModel,
                                  RobotCollisionModel robotCollisionModel,
                                  ROS2Node ros2Node,
                                  ROS2ImageSensors ros2ImageSensors,
                                  ROS2SyncedRobotModel ros2SyncedRobot)
   {
      // Create a bunch of overhead for the ROS2 communication and the robot
      ros2SyncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);
      ControllerFootstepQueueMonitor controllerFootstepQueueMonitor = new ControllerFootstepQueueMonitor(ros2Node, robotModel.getSimpleRobotName());

      // This is all the parameters grouped into one place, so we can pass things around easier
      activeMappingParameterToolBox = new ActiveMappingParameterToolBox(ros2Node, robotModel, "ForContinuousWalking");

      // This allows the sensor to be tuned via the user interface, and the effect shows on hardware, needed for calibrating the sensor
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      ROS2TunedRigidBodyTransform realsenseTunableTransform = ROS2TunedRigidBodyTransform.toBeTuned(ros2Helper,
                                                                                                    PerceptionAPI.STEPPING_CAMERA_TO_PARENT_TUNING,
                                                                                                    ros2SyncedRobot.getRobotModel()
                                                                                                                   .getSensorInformation()
                                                                                                                   .getSteppingCameraTransform());

      // This is for the height map, it expects the queue of images that we get from the sensors
      BlockingQueue<RawImage> rawImageCollectionSteppingCamera = new LinkedBlockingQueue<>(ImageSensor.DEFAULT_IMAGE_QUEUE_CAPACITY);
      BlockingQueue<RawImage> rawImageCollectionHeadCamera = new LinkedBlockingQueue<>(ImageSensor.DEFAULT_IMAGE_QUEUE_CAPACITY);
      if (ros2ImageSensors.getSensor("Stepping Camera") != null)
         ros2ImageSensors.getSensor("Stepping Camera").registerImageQueue(rawImageCollectionSteppingCamera, ZEDImageSensor.DEPTH_IMAGE_KEY);
      if (ros2ImageSensors.getSensor("Experimental Camera") != null)
         ros2ImageSensors.getSensor("Experimental Camera").registerImageQueue(rawImageCollectionHeadCamera, ZEDImageSensor.DEPTH_IMAGE_KEY);

      // Class's that perform the real work of the process... the good stuff
      {
         gpuMappingThread = new GpuMappingThread(ros2Node,
                                                 ros2SyncedRobot,
                                                 robotCollisionModel,
                                                 rawImageCollectionSteppingCamera,
                                                 controllerFootstepQueueMonitor,
                                                 activeMappingParameterToolBox);

         rapidPlanarRegionsExtractionThread = new RapidPlanarRegionsExtractionThread(ros2Node, rawImageCollectionHeadCamera);

         continuousPlanningStateMachine = new ContinuousPlanningStateMachine(robotModel,
                                                                             ros2Node,
                                                                             ros2SyncedRobot,
                                                                             ros2SyncedRobot.getReferenceFrames(),
                                                                             controllerFootstepQueueMonitor,
                                                                             activeMappingParameterToolBox);
      }

      // Custom thread getting started
      gpuMappingThread.startRepeating();
      rapidPlanarRegionsExtractionThread.startRepeating();

      // We create ThreadFactory's here so that when profiling the thread, we have user-friendly names to identify the threads with
      ThreadFactory threadFactorySyncedRobot = new ThreadFactoryBuilder().setNameFormat(SYNCED_ROBOT_THREAD).build();
      ScheduledExecutorService schedulerSyncedRobot = Executors.newScheduledThreadPool(1, threadFactorySyncedRobot);
      schedulerSyncedRobot.scheduleAtFixedRate(ros2SyncedRobot::update, 100, 10, TimeUnit.MILLISECONDS);

      ThreadFactory threadFactory = new ThreadFactoryBuilder().setNameFormat(SENSOR_TUNABLE_TRANSFORM_UPDATE_THREAD).build();
      ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1, threadFactory);
      scheduler.scheduleAtFixedRate(realsenseTunableTransform::update, 0, 33, TimeUnit.MILLISECONDS);

      ThreadFactory threadFactoryContinuousHiking = new ThreadFactoryBuilder().setNameFormat(CONTINUOUS_HIKING_THREAD).build();
      ScheduledExecutorService schedulerContinuousHiking = Executors.newScheduledThreadPool(1, threadFactoryContinuousHiking);
      schedulerContinuousHiking.scheduleWithFixedDelay(this::update, 500, 100, TimeUnit.MILLISECONDS);
   }

   public void update()
   {
      activeMappingParameterToolBox.update();

      // Update environment
      continuousPlanningStateMachine.setTerrainMapData(gpuMappingThread.getlatestTerrainMapData());
   }

   public void destroy()
   {
      gpuMappingThread.blockingKill();
      rapidPlanarRegionsExtractionThread.kill();
      continuousPlanningStateMachine.destroy();
   }
}
