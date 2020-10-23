package us.ihmc.atlas.behaviors;

import java.util.function.Supplier;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.jfxvisualizer.AtlasFootstepPlannerUI;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationParameters;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionPublisher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.tools.SimulatedREAModule;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIRegistry;
import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.javafx.JavaFXMissingTools;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.parameterTuner.remote.ParameterTuner;
import us.ihmc.pathPlanning.PlannerTestEnvironments;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.tools.processManagement.JavaProcessSpawner;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

/**
 * Runs self contained behavior demo.
 */
public class AtlasBehaviorUIDemo
{
   private static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;
   private static final boolean USE_KINEMATIC_SIMULATION = true;
   private static final boolean CREATE_YO_VARIABLE_SERVER = false;
   private static final boolean LAUNCH_PARAMETER_TUNER = false;
   private static final boolean LAUNCH_FOOTSTEP_PLANNER_UI = false;

   // functions to prevent constructing all environments every time
   private static final Supplier<PlanarRegionsList> FLAT_GROUND = () -> PlanarRegionsList.flatGround(10.0);
   private static final Supplier<PlanarRegionsList> UP_DOWN_OPEN_HOUSE = BehaviorPlanarRegionEnvironments::createUpDownOpenHouseRegions;
   private static final Supplier<PlanarRegionsList> UP_DOWN_TWO_HIGH_FLAT_IN_BETWEEN = BehaviorPlanarRegionEnvironments::createUpDownTwoHighWithFlatBetween;
   private static final Supplier<PlanarRegionsList> UP_DOWN_FOUR_HIGH_WITH_FLAT_CENTER = BehaviorPlanarRegionEnvironments::createUpDownFourHighWithFlatCenter;
   private static final Supplier<PlanarRegionsList> STAIRS = BehaviorPlanarRegionEnvironments::createStairs;
   private static final Supplier<PlanarRegionsList> SLAM_REAL_DATA = BehaviorPlanarRegionEnvironments::realDataFromAtlasSLAMDataset20190710;
   private static final Supplier<PlanarRegionsList> CORRIDOR = PlannerTestEnvironments::getTrickCorridor;

   private static final Supplier<PlanarRegionsList> ENVIRONMENT = CORRIDOR;

   // Increase to 10 when you want the sims to run a little faster and don't need all of the YoVariable data.
   private final int recordFrequencySpeedup = 10;

   public AtlasBehaviorUIDemo()
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      if (ENVIRONMENT != FLAT_GROUND)
      {
         ThreadTools.startAThread(this::simulatedREAModule, "SimulatedREAModule");
         ThreadTools.startAThread(this::bipedalSupportRegionPublisher, "BipedalSupportRegionPublisher");
      }

      ThreadTools.startAThread(this::simulation, "Simulation");
      ThreadTools.startAThread(this::footstepPlanningToolbox, "FootstepPlanningToolbox");
      if (LAUNCH_PARAMETER_TUNER) ThreadTools.startAThread(this::parameterTuner, "ParameterTuner");
      if (LAUNCH_FOOTSTEP_PLANNER_UI) ThreadTools.startAThread(this::footstepPlannerUI, "FootstepPlannerUI");

      BehaviorUIRegistry behaviorRegistry = BehaviorUIRegistry.DEFAULT_BEHAVIORS;

      LogTools.info("Creating behavior module");
      BehaviorModule.createInterprocess(behaviorRegistry, createRobotModel());

      LogTools.info("Creating behavior user interface");
      BehaviorUI.createInterprocess(behaviorRegistry, createRobotModel(), "localhost");
   }

   private void simulation()
   {
      if (USE_KINEMATIC_SIMULATION)
      {
         LogTools.info("Creating kinematics simulation");
         HumanoidKinematicsSimulationParameters kinematicsSimulationParameters = new HumanoidKinematicsSimulationParameters();
         kinematicsSimulationParameters.setPubSubImplementation(PubSubImplementation.FAST_RTPS);
         kinematicsSimulationParameters.setCreateYoVariableServer(CREATE_YO_VARIABLE_SERVER);
         HumanoidKinematicsSimulation.create(createRobotModel(), kinematicsSimulationParameters);
      }
      else
      {
         LogTools.info("Creating dynamics simulation");
         AtlasDynamicsSimulation.createForManualTest(createRobotModel(),
                                                     new PlanarRegionsListDefinedEnvironment(ENVIRONMENT.get(), 0.02, false),
                                                     recordFrequencySpeedup, 10).simulate();
      }
   }

   private void simulatedREAModule()
   {
      LogTools.info("Creating planar region publisher");
      new SimulatedREAModule(ENVIRONMENT.get(), createRobotModel(), PubSubImplementation.FAST_RTPS).start();
   }

   private void bipedalSupportRegionPublisher()
   {
      LogTools.info("Creating bipedal support region publisher");
      new BipedalSupportPlanarRegionPublisher(createRobotModel(), PubSubImplementation.FAST_RTPS).start();
   }

   private void footstepPlanningToolbox()
   {
      LogTools.info("Starting footstep toolbox");
      FootstepPlanningModuleLauncher.createModule(createRobotModel(), PubSubImplementation.FAST_RTPS);
   }

   private void parameterTuner()
   {
      LogTools.info("Spawning parameter tuner");
      new JavaProcessSpawner(true).spawn(ParameterTuner.class); // NPE if ParameterTuner started in same process, so spawn it
   }

   private void footstepPlannerUI()
   {
      LogTools.info("Launching remote footstep planner UI");
      AtlasFootstepPlannerUI atlasFootstepPlannerUI = new AtlasFootstepPlannerUI();
      JavaFXMissingTools.runApplication(atlasFootstepPlannerUI);
   }

   private AtlasRobotModel createRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      return new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false, simulationContactPoints);
   }

   public static void main(String[] args)
   {
      new AtlasBehaviorUIDemo();
   }
}
