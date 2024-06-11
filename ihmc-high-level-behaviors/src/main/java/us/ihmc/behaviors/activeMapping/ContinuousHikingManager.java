package us.ihmc.behaviors.activeMapping;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.tools.thread.RestartableThrottledThread;

/**
 * This class takes care of stuff to run continuous hiking in the {@link us.ihmc.PerceptionAndAutonomyProcess}.
 */
public class ContinuousHikingManager
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ContinuousPlannerSchedulingTask continuousHiking;
   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private final RestartableThrottledThread parameterUpdateThread;
   private HeightMapData latestHeightMapData;

   public ContinuousHikingManager(ROS2Node ros2Node, DRCRobotModel robotModel)
   {
      ROS2ControllerHelper ros2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      syncedRobot = new ROS2SyncedRobotModel(robotModel, ros2ControllerHelper.getROS2NodeInterface());
      syncedRobot.initializeToDefaultRobotInitialSetup(0.0, 0.0, 0.0, 0.0);

      ContinuousHikingParameters continuousPlanningParameters = new ContinuousHikingParameters();

      continuousHiking = new ContinuousPlannerSchedulingTask(robotModel,
                                                             ros2Node,
                                                             syncedRobot.getReferenceFrames(),
                                                             continuousPlanningParameters,
                                                             ContinuousPlanner.PlanningMode.FAST_HIKING);

      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2ControllerHelper);
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.CONTINUOUS_WALKING_PARAMETERS,
                                                     continuousPlanningParameters);
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.FOOTSTEP_PLANNING_PARAMETERS,
                                                     continuousHiking.getContinuousPlanner().getFootstepPlannerParameters());
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.SWING_PLANNING_PARAMETERS,
                                                     continuousHiking.getContinuousPlanner().getSwingPlannerParameters());
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.MONTE_CARLO_PLANNER_PARAMETERS,
                                                     continuousHiking.getContinuousPlanner().getMonteCarloFootstepPlannerParameters());

      parameterUpdateThread = new RestartableThrottledThread("ContinuousHikingStoredPropertySetUpdater", 1.0, ros2PropertySetGroup::update);
      parameterUpdateThread.start();
   }

   /** This triggers a thread safe deep copy */
   public void setHeightMapDataAsync(RapidHeightMapExtractor heightMapExtractor)
   {
      latestHeightMapData = RapidHeightMapExtractor.packHeightMapData(heightMapExtractor, latestHeightMapData);
      continuousHiking.setLatestHeightMapData(latestHeightMapData);
      continuousHiking.setTerrainMapData(heightMapExtractor.getTerrainMapData());
   }

   public void destroy()
   {
      parameterUpdateThread.stop();
      continuousHiking.destroy();
      syncedRobot.destroy();
   }
}
