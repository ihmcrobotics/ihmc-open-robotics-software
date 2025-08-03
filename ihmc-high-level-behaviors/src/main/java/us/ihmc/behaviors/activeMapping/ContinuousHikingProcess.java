package us.ihmc.behaviors.activeMapping;

import com.google.common.util.concurrent.ThreadFactoryBuilder;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2TunedRigidBodyTransform;
import us.ihmc.footstepPlanning.SnappingTerrainManager;
import us.ihmc.footstepPlanning.graphSearch.EnvironmentHandler;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.perception.steppableRegions.SteppableRegionsManager;
import us.ihmc.perception.ROS2ImageSensors;
import us.ihmc.perception.RapidHeightMapThread;
import us.ihmc.perception.RawImage;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.realsense.RealSenseImageSensor;
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

   private final EnvironmentHandler environmentHandler = new EnvironmentHandler();
   private final ActiveMappingParameterToolBox activeMappingParameterToolBox;
   private final ContinuousPlannerSchedulingTask continuousPlannerSchedulingTask;
   private final SnappingTerrainManager snappingTerrainManager;
   private final RapidHeightMapThread rapidHeightMapThread;
   private final SteppableRegionsManager steppableRegionsManager;

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
      BlockingQueue<RawImage> rawImageCollection = new LinkedBlockingQueue<>(ImageSensor.DEFAULT_IMAGE_QUEUE_CAPACITY);
      ros2ImageSensors.registerImageQueueForRealsense(rawImageCollection, RealSenseImageSensor.DEPTH_IMAGE_KEY);
      ros2ImageSensors.registerImageQueueForZED(rawImageCollection, ZEDImageSensor.DEPTH_IMAGE_KEY);

      // Class's that perform the real work of the process... the good stuff
      {
         rapidHeightMapThread = new RapidHeightMapThread(ros2Node,
                                                         ros2SyncedRobot,
                                                         robotCollisionModel,
                                                         rawImageCollection,
                                                         controllerFootstepQueueMonitor,
                                                         activeMappingParameterToolBox.getHeightMapParameters(),
                                                         activeMappingParameterToolBox.getDepthImageFilteringParameters());

         snappingTerrainManager = new SnappingTerrainManager(ros2Node, activeMappingParameterToolBox.getHeightMapParameters());
         steppableRegionsManager = new SteppableRegionsManager(ros2Node);
         continuousPlannerSchedulingTask = new ContinuousPlannerSchedulingTask(robotModel,
                                                                               ros2Node,
                                                                               ros2SyncedRobot,
                                                                               ros2SyncedRobot.getReferenceFrames(),
                                                                               controllerFootstepQueueMonitor,
                                                                               activeMappingParameterToolBox);
      }

      // Custom thread getting started
      rapidHeightMapThread.startRepeating();

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
      environmentHandler.setHeightMapData(rapidHeightMapThread.getLatestHeightMapData());
      snappingTerrainManager.updateAndPublish(environmentHandler.getHeightMapData());
      environmentHandler.setTerrainMapData(snappingTerrainManager.getTerrainMapData());
      steppableRegionsManager.update(environmentHandler.getTerrainMapData());
      continuousPlannerSchedulingTask.setLatestEnvironmentHandler(environmentHandler);
   }

   public void destroy()
   {
      rapidHeightMapThread.blockingKill();
      continuousPlannerSchedulingTask.destroy();
      snappingTerrainManager.close();
      steppableRegionsManager.destroy();
   }
}
