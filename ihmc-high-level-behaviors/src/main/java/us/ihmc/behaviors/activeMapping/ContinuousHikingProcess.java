package us.ihmc.behaviors.activeMapping;

import com.google.common.util.concurrent.ThreadFactoryBuilder;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.perception.StandAloneRealsenseProcess;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

public class ContinuousHikingProcess
{
   public static final String CONTINUOUS_HIKING_THREAD = "ContinuousHikingThread";
   public static final String SYNCED_ROBOT_THREAD = "SyncedRobotThread";

   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private final ContinuousPlannerSchedulingTask continuousPlannerSchedulingTask;

   private final StandAloneRealsenseProcess standAloneRealsenseProcess;

   public ContinuousHikingProcess(DRCRobotModel robotModel, RobotCollisionModel robotCollisionModel)
   {
      ROS2Node ros2Node = new ROS2NodeBuilder().build("nadia_terrain_perception_node");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      ROS2SyncedRobotModel syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);

      // We create a ThreadFactory here so that when profiling the thread, there is a user-friendly name to identify it with
      ThreadFactory threadFactorySyncedRobot = new ThreadFactoryBuilder().setNameFormat(SYNCED_ROBOT_THREAD).build();
      ScheduledExecutorService schedulerSyncedRobot = Executors.newScheduledThreadPool(1, threadFactorySyncedRobot);
      schedulerSyncedRobot.scheduleAtFixedRate(syncedRobot::update, 100, 10, TimeUnit.MILLISECONDS);

      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Node);
      ActiveMappingParameterToolBox activeMappingParameterToolBox = new ActiveMappingParameterToolBox(ros2PropertySetGroup, robotModel, "ForContinuousWalking");

      ContinuousHikingLogger continuousHikingLogger = new ContinuousHikingLogger();
      ControllerFootstepQueueMonitor controllerFootstepQueueMonitor = new ControllerFootstepQueueMonitor(ros2Node, robotModel.getSimpleRobotName());

      standAloneRealsenseProcess = new StandAloneRealsenseProcess(ros2Node,
                                                                  ros2Helper,
                                                                  syncedRobot,
                                                                  robotCollisionModel,
                                                                  activeMappingParameterToolBox.getHeightMapParameters(),
                                                                  controllerFootstepQueueMonitor);

      continuousPlannerSchedulingTask = new ContinuousPlannerSchedulingTask(robotModel,
                                                                            ros2Node,
                                                                            syncedRobot,
                                                                            syncedRobot.getReferenceFrames(),
                                                                            controllerFootstepQueueMonitor,
                                                                            continuousHikingLogger,
                                                                            activeMappingParameterToolBox);

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));

      // We create a ThreadFactory here so that when profiling the thread, there is a user-friendly name to identify it with
      ThreadFactory threadFactoryContinuousHiking = new ThreadFactoryBuilder().setNameFormat(CONTINUOUS_HIKING_THREAD).build();
      ScheduledExecutorService schedulerContinuousHiking = Executors.newScheduledThreadPool(1, threadFactoryContinuousHiking);
      schedulerContinuousHiking.scheduleWithFixedDelay(this::update, 500, 100, TimeUnit.MILLISECONDS);
   }

   public void update()
   {
      ros2PropertySetGroup.update();

      if (standAloneRealsenseProcess.getHeightMapManager() == null)
      {
         return;
      }

      if (standAloneRealsenseProcess.getLatestHeightMapData() != null && standAloneRealsenseProcess.getLatestTerrainMapData() != null)
      {
         continuousPlannerSchedulingTask.setLatestHeightMapData(standAloneRealsenseProcess.getLatestHeightMapData());
         continuousPlannerSchedulingTask.setTerrainMapData(standAloneRealsenseProcess.getLatestTerrainMapData());
      }
   }

   public void destroy()
   {
      continuousPlannerSchedulingTask.destroy();
      standAloneRealsenseProcess.destroy();
   }
}
