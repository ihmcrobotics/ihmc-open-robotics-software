package us.ihmc.behaviors.continuousWalking;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.activeMapping.ContinuousPlanningParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.HumanoidActivePerceptionModule;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.headless.TerrainPerceptionProcessWithDriver;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class PerceptionBasedContinuousWalking
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2Helper ros2Helper;
   private ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private TerrainPerceptionProcessWithDriver perceptionTask;
   private HumanoidActivePerceptionModule activePerceptionModule;
   private final ContinuousPlanningParameters continuousPlanningParameters = new ContinuousPlanningParameters();

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   public PerceptionBasedContinuousWalking(DRCRobotModel robotModel, String realsenseSerialNumber)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "nadia_terrain_perception_node");
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      ros2Helper = new ROS2Helper(ros2Node);
      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);
      perceptionTask = new TerrainPerceptionProcessWithDriver(realsenseSerialNumber,
                                                              robotModel.getSimpleRobotName(),
                                                              robotModel.getCollisionBoxProvider(),
                                                              robotModel.createFullRobotModel(),
                                                              RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ,
                                                              ros2PropertySetGroup,
                                                              ros2Helper,
                                                              PerceptionAPI.D455_DEPTH_IMAGE,
                                                              PerceptionAPI.D455_COLOR_IMAGE,
                                                              PerceptionAPI.PERSPECTIVE_RAPID_REGIONS,
                                                              syncedRobot.getReferenceFrames(),
                                                              syncedRobot::update);

      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.FOOTSTEP_PLANNING_PARAMETERS, activePerceptionModule.getContinuousMappingRemoteThread().getContinuousPlanner().getFootstepPlannerParameters());
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.CONTINUOUS_PLANNING_PARAMETERS, continuousPlanningParameters);
      activePerceptionModule = new HumanoidActivePerceptionModule(perceptionTask.getConfigurationParameters(), continuousPlanningParameters);
      activePerceptionModule.initializeContinuousElevationMappingTask(robotModel, ros2Node, syncedRobot.getReferenceFrames());

      perceptionTask.run();

      // Add initial delay to get things going in the right order
      executorService.scheduleAtFixedRate(this::update, 500, 100, TimeUnit.MILLISECONDS);

      ThreadTools.sleepForever();
   }

   public void update()
   {
      ros2PropertySetGroup.update();

      if (perceptionTask.getHumanoidPerceptionModule().getLatestHeightMapData() != null)
      {
         perceptionTask.getHumanoidPerceptionModule().setIsHeightMapDataBeingProcessed(true);
         activePerceptionModule.getContinuousMappingRemoteThread().setLatestHeightMapData(perceptionTask.getHumanoidPerceptionModule().getLatestHeightMapData());
         perceptionTask.getHumanoidPerceptionModule().setIsHeightMapDataBeingProcessed(false);
      }
   }
}
