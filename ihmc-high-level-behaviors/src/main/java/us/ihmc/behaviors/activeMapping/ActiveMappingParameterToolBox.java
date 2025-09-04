package us.ihmc.behaviors.activeMapping;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.communication.ContinuousHikingAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.filters.DepthImageFilteringParameters;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.perception.heightMap.HeightMapParameters;

public class ActiveMappingParameterToolBox
{
   private final ContinuousHikingParameters continuousHikingParameters;
   private final MonteCarloFootstepPlannerParameters monteCarloPlannerParameters;
   private final DefaultFootstepPlannerParametersBasics footstepPlannerParameters;
   private final SwingPlannerParametersBasics swingPlannerParameters;
   private final HeightMapParameters heightMapParameters;
   private final ROS2StoredPropertySetGroup ros2StoredPropertySetGroup;
   private final DepthImageFilteringParameters depthImageFilteringParameters;
   private final SteppableRegionCalculatorParameters steppableRegionCalculatorParameters;

   public ActiveMappingParameterToolBox(ROS2Node ros2Node, DRCRobotModel robotModel, String taskPurpose)
   {
      ros2StoredPropertySetGroup = new ROS2StoredPropertySetGroup(ros2Node);

      continuousHikingParameters = new ContinuousHikingParameters();
      monteCarloPlannerParameters = new MonteCarloFootstepPlannerParameters();
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters(taskPurpose);
      swingPlannerParameters = robotModel.getSwingPlannerParameters(taskPurpose);
      heightMapParameters = new HeightMapParameters();
      depthImageFilteringParameters = new DepthImageFilteringParameters();
      steppableRegionCalculatorParameters = new SteppableRegionCalculatorParameters();

      // Add Parameters to be synced between the UI and this process
      ros2StoredPropertySetGroup.registerStoredPropertySet(ContinuousHikingAPI.CONTINUOUS_HIKING_PARAMETERS, continuousHikingParameters);
      ros2StoredPropertySetGroup.registerStoredPropertySet(ContinuousHikingAPI.MONTE_CARLO_PLANNER_PARAMETERS, monteCarloPlannerParameters);
      ros2StoredPropertySetGroup.registerStoredPropertySet(ContinuousHikingAPI.FOOTSTEP_PLANNING_PARAMETERS, footstepPlannerParameters);
      ros2StoredPropertySetGroup.registerStoredPropertySet(ContinuousHikingAPI.SWING_PLANNING_PARAMETERS, swingPlannerParameters);
      ros2StoredPropertySetGroup.registerStoredPropertySet(PerceptionComms.HEIGHT_MAP_PARAMETERS, heightMapParameters);
      ros2StoredPropertySetGroup.registerStoredPropertySet(ContinuousHikingAPI.DEPTH_IMAGE_FILTERING_PARAMETERS, depthImageFilteringParameters);
      ros2StoredPropertySetGroup.registerStoredPropertySet(ContinuousHikingAPI.STEPPABLE_REGION_CALCULATOR_PARAMETERS, steppableRegionCalculatorParameters);
   }

   public void update()
   {
      ros2StoredPropertySetGroup.update();
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

   public DepthImageFilteringParameters getDepthImageFilteringParameters()
   {
      return depthImageFilteringParameters;
   }

   public SteppableRegionCalculatorParameters getSteppableRegionCalculatorParameters()
   {
      return steppableRegionCalculatorParameters;
   }
}
