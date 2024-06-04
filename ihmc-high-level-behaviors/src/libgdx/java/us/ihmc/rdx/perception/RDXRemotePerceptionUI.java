package us.ihmc.rdx.perception;

import us.ihmc.avatar.colorVision.BlackflyComms;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.mapping.PlanarRegionMappingParameters;
import us.ihmc.perception.parameters.IntrinsicCameraMatrixProperties;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.rapidRegions.RapidRegionsExtractorParameters;
import us.ihmc.perception.sensorHead.BlackflyLensProperties;
import us.ihmc.perception.sensorHead.SensorHeadParameters;
import us.ihmc.perception.steppableRegions.SteppableRegionCalculatorParameters;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySetGroup;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

public class RDXRemotePerceptionUI
{
   private RDXPanel panel;

   private final SteppableRegionCalculatorParameters steppableRegionCalculatorParameters = new SteppableRegionCalculatorParameters();
   private final PerceptionConfigurationParameters perceptionConfigurationParameters = new PerceptionConfigurationParameters();
   private final ContinuousHikingParameters continuousHikingParameters = new ContinuousHikingParameters();
   private final HeightMapParameters heightMapParameters = new HeightMapParameters("GPU");

   private SwingPlannerParametersBasics swingPlannerParameters;
   private FootstepPlannerParametersBasics footstepPlannerParameters;

   private final MonteCarloFootstepPlannerParameters monteCarloFootstepPlannerParameters = new MonteCarloFootstepPlannerParameters();
   private final RapidRegionsExtractorParameters rapidRegionsExtractorParameters = new RapidRegionsExtractorParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PlanarRegionMappingParameters regionMappingParameters = new PlanarRegionMappingParameters();

   private final RapidRegionsExtractorParameters sphericalRegionExtractorParameters = new RapidRegionsExtractorParameters("Spherical");
   private final PolygonizerParameters sphericalPolygonizerParameters = new PolygonizerParameters("ForSphericalRapidRegions");
   private final ConcaveHullFactoryParameters sphericalConcaveHullFactoryParameters = new ConcaveHullFactoryParameters("ForSphericalRapidRegions");
   private final PlanarRegionMappingParameters sphericalRegionMappingParameters = new PlanarRegionMappingParameters("Spherical");

   private IntrinsicCameraMatrixProperties ousterFisheyeColoringIntrinsics;

   private final ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets;

   public RDXRemotePerceptionUI(ROS2Helper ros2Helper, RDXPanel panel)
   {
      this(ros2Helper);
      this.panel = panel;
   }

   public RDXRemotePerceptionUI(ROS2Helper ros2Helper)
   {
      remotePropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Helper);

      remotePropertySets.registerRemotePropertySet(perceptionConfigurationParameters, PerceptionComms.PERCEPTION_CONFIGURATION_PARAMETERS);
      remotePropertySets.registerRemotePropertySet(continuousHikingParameters, ContinuousWalkingAPI.CONTINUOUS_WALKING_PARAMETERS);
      remotePropertySets.registerRemotePropertySet(heightMapParameters, PerceptionComms.HEIGHT_MAP_PARAMETERS);
      remotePropertySets.registerRemotePropertySet(monteCarloFootstepPlannerParameters, ContinuousWalkingAPI.MONTE_CARLO_PLANNER_PARAMETERS);

      registerRapidRegionsParameters();
   }

   public void registerRapidRegionsParameters()
   {
      remotePropertySets.registerRemotePropertySet(rapidRegionsExtractorParameters, PerceptionComms.PERSPECTIVE_RAPID_REGION_PARAMETERS);
      remotePropertySets.registerRemotePropertySet(polygonizerParameters, PerceptionComms.PERSPECTIVE_POLYGONIZER_PARAMETERS);
      remotePropertySets.registerRemotePropertySet(concaveHullFactoryParameters, PerceptionComms.PERSPECTIVE_CONVEX_HULL_FACTORY_PARAMETERS);
      remotePropertySets.registerRemotePropertySet(regionMappingParameters, PerceptionComms.PERSPECTIVE_PLANAR_REGION_MAPPING_PARAMETERS);

      remotePropertySets.registerRemotePropertySet(sphericalRegionExtractorParameters, PerceptionComms.SPHERICAL_RAPID_REGION_PARAMETERS);
      remotePropertySets.registerRemotePropertySet(sphericalPolygonizerParameters, PerceptionComms.SPHERICAL_POLYGONIZER_PARAMETERS);
      remotePropertySets.registerRemotePropertySet(sphericalConcaveHullFactoryParameters, PerceptionComms.SPHERICAL_CONVEX_HULL_FACTORY_PARAMETERS);
      remotePropertySets.registerRemotePropertySet(sphericalRegionMappingParameters, PerceptionComms.SPHERICAL_PLANAR_REGION_MAPPING_PARAMETERS);
   }

   public void setBlackflyLensProperties(BlackflyLensProperties blackflyLensCombo)
   {
      ousterFisheyeColoringIntrinsics = SensorHeadParameters.loadOusterFisheyeColoringIntrinsicsOnRobot(blackflyLensCombo);
      remotePropertySets.registerRemotePropertySet(ousterFisheyeColoringIntrinsics, BlackflyComms.OUSTER_FISHEYE_COLORING_INTRINSICS);
   }

   public void renderImGuiWidgets()
   {
      remotePropertySets.renderImGuiWidgets();
   }

   public void setPropertyChanged()
   {
      remotePropertySets.setPropertyChanged();
   }

   public void destroy()
   {
   }

   public PerceptionConfigurationParameters getPerceptionConfigurationParameters()
   {
      return perceptionConfigurationParameters;
   }

   public ContinuousHikingParameters getContinuousPlanningParameters()
   {
      return continuousHikingParameters;
   }

   public SwingPlannerParametersBasics getSwingPlannerParameters()
   {
      return swingPlannerParameters;
   }

   public HeightMapParameters getHeightMapParameters()
   {
      return heightMapParameters;
   }

   public FootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return footstepPlannerParameters;
   }

   public void setFootstepPlannerParameters(FootstepPlannerParametersBasics parameters)
   {
      this.footstepPlannerParameters = parameters;
      remotePropertySets.registerRemotePropertySet(footstepPlannerParameters, ContinuousWalkingAPI.FOOTSTEP_PLANNING_PARAMETERS);
   }

   public void setSwingPlannerParameters(SwingPlannerParametersBasics parameters)
   {
      this.swingPlannerParameters = parameters;
      remotePropertySets.registerRemotePropertySet(swingPlannerParameters, ContinuousWalkingAPI.SWING_PLANNING_PARAMETERS);
   }

   public MonteCarloFootstepPlannerParameters getMonteCarloFootstepPlannerParameters()
   {
      return monteCarloFootstepPlannerParameters;
   }

   public SteppableRegionCalculatorParameters getSteppableRegionCalculatorParameters()
   {
      return steppableRegionCalculatorParameters;
   }
}
