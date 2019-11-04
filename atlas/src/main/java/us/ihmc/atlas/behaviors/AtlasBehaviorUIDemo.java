package us.ihmc.atlas.behaviors;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionPublisher;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.tools.FakeREAModule;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.simulation.BehaviorPlanarRegionEnvironments;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.parameterTuner.remote.ParameterTuner;
import us.ihmc.pathPlanning.PlannerTestEnvironments;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.tools.processManagement.JavaProcessSpawner;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

import java.util.function.Supplier;

/**
 * Runs self contained behavior demo.
 */
public class AtlasBehaviorUIDemo extends Application
{
   private static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;
   private static final boolean USE_KINEMATIC_SIMULATION = true;
   private static final boolean CREATE_YO_VARIABLE_SERVER = false;
   private static final boolean LAUNCH_PARAMETER_TUNER = false;

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

   private BehaviorUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      if (ENVIRONMENT != FLAT_GROUND)
      {
         new Thread(() -> {
            LogTools.info("Creating planar region publisher");
            new FakeREAModule(ENVIRONMENT.get(), createRobotModel()).start();
         }).start();

         new Thread(() -> {
            LogTools.info("Creating bipedal support region publisher");
            new BipedalSupportPlanarRegionPublisher(createRobotModel()).start();
         }).start();
      }

      new Thread(() -> {
         LogTools.info("Creating simulation");
         if (USE_KINEMATIC_SIMULATION)
         {
            HumanoidKinematicsSimulation.createForManualTest(createRobotModel(), CREATE_YO_VARIABLE_SERVER);
         }
         else
         {
            AtlasBehaviorSimulation.createForManualTest(createRobotModel(),
                                                        new PlanarRegionsListDefinedEnvironment(ENVIRONMENT.get(), 0.02, false),
                                                        recordFrequencySpeedup).simulate();
         }
      }).start();

      new Thread(() -> {
         LogTools.info("Creating footstep toolbox");
         new MultiStageFootstepPlanningModule(createRobotModel(), null, false, DomainFactory.PubSubImplementation.FAST_RTPS);
      }).start();

      new Thread(() -> {
         LogTools.info("Creating behavior backpack");
         BehaviorModule.createForBackpack(createRobotModel());
      }).start();

      if (LAUNCH_PARAMETER_TUNER)
      {
         new Thread(() ->
         {
            LogTools.info("Spawning parameter tuner");
            new JavaProcessSpawner(true).spawn(ParameterTuner.class); // NPE if ParameterTuner started in same process, so spawn it
         }).start();
      }

      LogTools.info("Creating behavior user interface");
      AtlasRobotModel robotModel = createRobotModel();
      Messager behaviorMessager = RemoteBehaviorInterface.createForUI("localhost");
      ui = new BehaviorUI(primaryStage, behaviorMessager, robotModel, PubSubImplementation.FAST_RTPS);
      ui.show();
   }

   private AtlasRobotModel createRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      return new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false, simulationContactPoints);

   }

   @Override
   public void stop() throws Exception
   {
      super.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
