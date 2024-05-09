package us.ihmc.behaviors.activeMapping;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.HumanoidActivePerceptionModule;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.headless.TerrainPerceptionProcessWithDriver;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class PerceptionBasedContinuousHiking
{
   private static final ContinuousPlanner.PlanningMode PLANNING_MODE = ContinuousPlanner.PlanningMode.FAST_HIKING;

   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private final TerrainPerceptionProcessWithDriver perceptionTask;
   private final HumanoidActivePerceptionModule activePerceptionModule;
   private final ContinuousHikingParameters continuousHikingParameters = new ContinuousHikingParameters();

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   public PerceptionBasedContinuousHiking(DRCRobotModel robotModel, String realsenseSerialNumber)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "nadia_terrain_perception_node");
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(robotModel.createFullRobotModel());

      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);

      perceptionTask = new TerrainPerceptionProcessWithDriver(realsenseSerialNumber,
                                                              robotModel.getSimpleRobotName(),
                                                              robotModel.getCollisionBoxProvider(),
                                                              robotModel.createFullRobotModel(),
                                                              RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ,
                                                              ros2PropertySetGroup, ros2Helper,
                                                              PerceptionAPI.D455_DEPTH_IMAGE,
                                                              PerceptionAPI.D455_COLOR_IMAGE,
                                                              PerceptionAPI.PERSPECTIVE_RAPID_REGIONS,
                                                              referenceFrames);

      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(HighLevelStateChangeStatusMessage.class, robotModel.getSimpleRobotName()),
                                      this::highLevelStateChangeStatusReceived);

      activePerceptionModule = new HumanoidActivePerceptionModule(perceptionTask.getConfigurationParameters());
      activePerceptionModule.initializeContinuousHikingSchedulingTask(robotModel,
                                                                      ros2Helper,
                                                                      ros2Node,
                                                                      referenceFrames, continuousHikingParameters,
                                                                      PLANNING_MODE);

      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.CONTINUOUS_WALKING_PARAMETERS, continuousHikingParameters);
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.FOOTSTEP_PLANNING_PARAMETERS, activePerceptionModule.getContinuousPlannerSchedulingTask().getContinuousPlanner().getFootstepPlannerParameters());
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.SWING_PLANNING_PARAMETERS, activePerceptionModule.getContinuousPlannerSchedulingTask().getContinuousPlanner().getSwingPlannerParameters());
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.MONTE_CARLO_PLANNER_PARAMETERS, activePerceptionModule.getContinuousPlannerSchedulingTask().getContinuousPlanner().getMonteCarloFootstepPlannerParameters());

      perceptionTask.run();

      // Add initial delay to get things going in the right order
      executorService.scheduleAtFixedRate(this::update, 500, 100, TimeUnit.MILLISECONDS);

      ThreadTools.sleepForever();
   }

   private void highLevelStateChangeStatusReceived(HighLevelStateChangeStatusMessage message)
   {
      HighLevelControllerName endState = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());

      if (continuousHikingParameters.getDefaultOperatingMode())
      {
         continuousHikingParameters.setDisableUpdatingHeightMap(endState == HighLevelControllerName.FREEZE_STATE);
         RapidHeightMapExtractor.getHeightMapParameters().setResetHeightMap(endState == HighLevelControllerName.STAND_TRANSITION_STATE);
      }
   }

   public void update()
   {
      if (continuousHikingParameters.getDisableUpdatingHeightMap())
         return;

      ros2PropertySetGroup.update();

      if (perceptionTask.getHumanoidPerceptionModule().getRapidRegionsExtractor() == null)
         return;

      if (perceptionTask.getHumanoidPerceptionModule().getLatestHeightMapData() != null)
      {
         perceptionTask.getHumanoidPerceptionModule().setIsHeightMapDataBeingProcessed(true);
         activePerceptionModule.getContinuousPlannerSchedulingTask().setLatestHeightMapData(perceptionTask.getHumanoidPerceptionModule().getLatestHeightMapData());
         activePerceptionModule.getContinuousPlannerSchedulingTask().setTerrainMapData(perceptionTask.getHumanoidPerceptionModule().getRapidHeightMapExtractor()
                                                                                                     .getTerrainMapData());
         perceptionTask.getHumanoidPerceptionModule().setIsHeightMapDataBeingProcessed(false);
      }
   }
}
