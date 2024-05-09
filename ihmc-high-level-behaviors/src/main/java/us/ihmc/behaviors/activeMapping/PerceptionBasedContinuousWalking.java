package us.ihmc.behaviors.activeMapping;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner.PlanningMode;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.HumanoidActivePerceptionModule;
import us.ihmc.perception.headless.TerrainPerceptionProcessWithDriver;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.tools.thread.ExecutorServiceTools.ExceptionHandling;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class PerceptionBasedContinuousWalking
{
   private PlanningMode mode = PlanningMode.FAST_HIKING;

   private final ROS2Helper ros2Helper;
   private ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private TerrainPerceptionProcessWithDriver perceptionTask;
   private HumanoidActivePerceptionModule activePerceptionModule;
   private final ContinuousWalkingParameters continuousPlanningParameters = new ContinuousWalkingParameters();

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1, getClass(), ExceptionHandling.CATCH_AND_REPORT);

   public PerceptionBasedContinuousWalking(DRCRobotModel robotModel, String realsenseSerialNumber)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "nadia_terrain_perception_node");
      ros2Helper = new ROS2Helper(ros2Node);

      // Sets up the referenceFrames correctly for the process
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      robotModel.getDefaultRobotInitialSetup(0.0, 0.0).initializeFullRobotModel(fullRobotModel);
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel, robotModel.getSensorInformation());

      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
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
                                                              referenceFrames);

      activePerceptionModule = new HumanoidActivePerceptionModule(perceptionTask.getConfigurationParameters(), continuousPlanningParameters);
      activePerceptionModule.initializeContinuousPlannerSchedulingTask(robotModel, ros2Node, referenceFrames, mode);

      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.CONTINUOUS_WALKING_PARAMETERS, continuousPlanningParameters);
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.FOOTSTEP_PLANNING_PARAMETERS, activePerceptionModule.getContinuousPlannerSchedulingTask().getContinuousPlanner().getFootstepPlannerParameters());
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.SWING_PLANNING_PARAMETERS, activePerceptionModule.getContinuousPlannerSchedulingTask().getContinuousPlanner().getSwingPlannerParameters());
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.MONTE_CARLO_PLANNER_PARAMETERS, activePerceptionModule.getContinuousPlannerSchedulingTask().getContinuousPlanner().getMonteCarloFootstepPlannerParameters());

      perceptionTask.run();

      // Add initial delay to get things going in the right order
      executorService.scheduleAtFixedRate(this::update, 500, 100, TimeUnit.MILLISECONDS);

      ThreadTools.sleepForever();
   }

   public void update()
   {
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
