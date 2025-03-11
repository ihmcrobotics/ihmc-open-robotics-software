package us.ihmc.behaviors.activeMapping;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.communication.ContinuousHikingAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapManager;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

public class ActiveMappingParameterToolBox
{
   private final ContinuousHikingParameters continuousHikingParameters;
   private final MonteCarloFootstepPlannerParameters monteCarloPlannerParameters;
   private final DefaultFootstepPlannerParametersBasics footstepPlannerParameters;
   private final SwingPlannerParametersBasics swingPlannerParameters;
   private final HeightMapParameters heightMapParameters;

   public ActiveMappingParameterToolBox(ROS2StoredPropertySetGroup ros2PropertySetGroup, DRCRobotModel robotModel, String taskPurpose)
   {
      continuousHikingParameters = new ContinuousHikingParameters();
      monteCarloPlannerParameters = new MonteCarloFootstepPlannerParameters();
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters(taskPurpose);
      swingPlannerParameters = robotModel.getSwingPlannerParameters(taskPurpose);
      heightMapParameters = RapidHeightMapManager.getHeightMapParameters();

      // Add Parameters to be synced between the UI and this process
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousHikingAPI.CONTINUOUS_HIKING_PARAMETERS, continuousHikingParameters);
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousHikingAPI.MONTE_CARLO_PLANNER_PARAMETERS, monteCarloPlannerParameters);
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousHikingAPI.FOOTSTEP_PLANNING_PARAMETERS, footstepPlannerParameters);
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousHikingAPI.SWING_PLANNING_PARAMETERS, swingPlannerParameters);
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.HEIGHT_MAP_PARAMETERS, heightMapParameters);
   }

   public ContinuousHikingParameters getContinuousHikingParameters()
   {
      return continuousHikingParameters;
   }

   public MonteCarloFootstepPlannerParameters getMonteCarloPlannerParameters()
   {
      return monteCarloPlannerParameters;
   }

   public DefaultFootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return footstepPlannerParameters;
   }

   public SwingPlannerParametersBasics getSwingPlannerParameters()
   {
      return swingPlannerParameters;
   }

   public HeightMapParameters getHeightMapParameters()
   {
      return heightMapParameters;
   }
}
