package us.ihmc.behaviors.activeMapping;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.perception.headless.TerrainPerceptionProcessWithDriver;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class PerceptionBasedContinuousHiking
{
   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private final TerrainPerceptionProcessWithDriver perceptionTask;
   private final ContinuousPlannerSchedulingTask continuousPlannerSchedulingTask;

   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   public PerceptionBasedContinuousHiking(DRCRobotModel robotModel, String realsenseSerialNumber)
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "nadia_terrain_perception_node");
      ROS2SyncedRobotModel syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2Node);
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);

      // Add Continuous Hiking Parameters to be between the UI and this process
      ContinuousHikingParameters continuousHikingParameters = new ContinuousHikingParameters();
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.CONTINUOUS_HIKING_PARAMETERS, continuousHikingParameters);

      // Add Monte Carlo Footstep Planner Parameters to be between the UI and this process
      MonteCarloFootstepPlannerParameters monteCarloPlannerParameters = new MonteCarloFootstepPlannerParameters();
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.MONTE_CARLO_PLANNER_PARAMETERS, monteCarloPlannerParameters);

      // Add A* Footstep Planner Parameters to be between the UI and this process
      DefaultFootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters("ForContinuousWalking");
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.FOOTSTEP_PLANNING_PARAMETERS, footstepPlannerParameters);

      // Add Swing Planner Parameters to be synced between the UI and this process
      SwingPlannerParametersBasics swingPlannerParameters = robotModel.getSwingPlannerParameters();
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.SWING_PLANNING_PARAMETERS, swingPlannerParameters);

      perceptionTask = new TerrainPerceptionProcessWithDriver(realsenseSerialNumber,
                                                              robotModel.getSimpleRobotName(),
                                                              robotModel.getCollisionBoxProvider(),
                                                              robotModel.createFullRobotModel(),
                                                              RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ,
                                                              ros2PropertySetGroup,
                                                              ros2Helper,
                                                              PerceptionAPI.D455_DEPTH_IMAGE,
                                                              PerceptionAPI.D455_COLOR_IMAGE,
                                                              syncedRobot.getReferenceFrames(),
                                                              syncedRobot::update);

      continuousPlannerSchedulingTask = new ContinuousPlannerSchedulingTask(robotModel,
                                                                            ros2Node,
                                                                            syncedRobot.getReferenceFrames(),
                                                                            continuousHikingParameters,
                                                                            monteCarloPlannerParameters,
                                                                            footstepPlannerParameters,
                                                                            swingPlannerParameters);

      perceptionTask.run();

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));

      // Add initial delay to get things going in the right order
      executorService.scheduleAtFixedRate(this::update, 500, 100, TimeUnit.MILLISECONDS);
   }

   public void update()
   {
      ros2PropertySetGroup.update();

      if (perceptionTask.getHumanoidPerceptionModule().getRapidRegionsExtractor() == null)
         return;

      if (perceptionTask.getHumanoidPerceptionModule().getLatestHeightMapData() != null)
      {
         perceptionTask.getHumanoidPerceptionModule().setIsHeightMapDataBeingProcessed(true);
         continuousPlannerSchedulingTask.setLatestHeightMapData(perceptionTask.getHumanoidPerceptionModule().getLatestHeightMapData());
         continuousPlannerSchedulingTask.setTerrainMapData(perceptionTask.getHumanoidPerceptionModule().getRapidHeightMapExtractor().getTerrainMapData());
         perceptionTask.getHumanoidPerceptionModule().setIsHeightMapDataBeingProcessed(false);
      }
   }

   public void destroy()
   {
      if (continuousPlannerSchedulingTask != null)
         continuousPlannerSchedulingTask.destroy();
   }
}
