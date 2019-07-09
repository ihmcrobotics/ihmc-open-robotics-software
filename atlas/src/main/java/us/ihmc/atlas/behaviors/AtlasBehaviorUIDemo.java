package us.ihmc.atlas.behaviors;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.avatar.kinematicsSimulation.AvatarKinematicsSimulation;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionPublisher;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.tools.FakeREAModule;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.simulation.PatrolSimulationRegionFields;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

/**
 * Runs self contained behavior demo.
 */
public class AtlasBehaviorUIDemo extends Application
{
   private static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;
   private static final boolean USE_FLAT_GROUND = false;
   private static final boolean USE_KINEMATIC_SIMULATION = true;
   private static final int RECORD_TICKS_PER_CONTROL_TICKS = 10;

   private BehaviorUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      if (!USE_FLAT_GROUND)
      {
         new Thread(() -> {
            LogTools.info("Creating planar region publisher");
            new FakeREAModule(createPlanarRegions());
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
            AvatarKinematicsSimulation.createForManualTest(createRobotModel(), true);
         }
         else
         {
            CommonAvatarEnvironmentInterface environment = USE_FLAT_GROUND ? new FlatGroundEnvironment()
                  : new PlanarRegionsListDefinedEnvironment(createPlanarRegions(), 0.02, false);
            SimulationConstructionSet scs = AtlasBehaviorSimulation.createForManualTest(createRobotModel(), environment, RECORD_TICKS_PER_CONTROL_TICKS);

            scs.simulate();
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

      //      new Thread(() -> {
      //         LogTools.info("Spawning parameter tuner");
      //         new JavaProcessSpawner(true).spawn(ParameterTuner.class); // NPE if ParameterTuner started in same process, so spawn it
      //      }).start();

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

   private PlanarRegionsList createPlanarRegions()
   {
      //      return PatrolSimulationRegionFields.createUpDownOpenHouseRegions();
      //      return PatrolSimulationRegionFields.createUpDownTwoHighWithFlatBetween();
      return PatrolSimulationRegionFields.createUpDownFourHighWithFlatCenter();
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
