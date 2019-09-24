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
import us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAM;
import us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAMParameters;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.simulation.PatrolSimulationRegionFields;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.parameterTuner.remote.ParameterTuner;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.tools.io.WorkspacePathTools;
import us.ihmc.tools.processManagement.JavaProcessSpawner;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

import java.nio.file.Path;

/**
 * Runs self contained behavior demo.
 */
public class AtlasBehaviorUIDemo extends Application
{
   private static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;
   private static final boolean USE_KINEMATIC_SIMULATION = true;
   public static final boolean CREATE_YO_VARIABLE_SERVER = false;
   private static final boolean LAUNCH_PARAMETER_TUNER = false;

   private enum Environment
   {
      FLAT_GROUND,
      UP_DOWN_OPEN_HOUSE,
      UP_DOWN_TWO_HIGH_FLAT_IN_BETWEEN,
      UP_DOWN_FOUR_HIGH_WITH_FLAT_CENTER,
      SLAM_REAL_DATA,
      STAIRS
   }
   private static final Environment ENVIRONMENT = Environment.SLAM_REAL_DATA;


   // Increase to 10 when you want the sims to run a little faster and don't need all of the YoVariable data.
   private final int recordFrequencySpeedup = 10;

   private BehaviorUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      if (ENVIRONMENT != Environment.FLAT_GROUND)
      {
         new Thread(() -> {
            LogTools.info("Creating planar region publisher");
            new FakeREAModule(createPlanarRegions(), createRobotModel()).start();
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
            AvatarKinematicsSimulation.createForManualTest(createRobotModel(), CREATE_YO_VARIABLE_SERVER);
         }
         else
         {
            AtlasBehaviorSimulation.createForManualTest(createRobotModel(), generateEnvironment(), recordFrequencySpeedup).simulate();
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

   private CommonAvatarEnvironmentInterface generateEnvironment()
   {
      return new PlanarRegionsListDefinedEnvironment(createPlanarRegions(), 0.02, false);
   }

   private PlanarRegionsList createPlanarRegions()
   {
      switch (ENVIRONMENT)
      {
         case UP_DOWN_OPEN_HOUSE:
            return PatrolSimulationRegionFields.createUpDownOpenHouseRegions();
         case UP_DOWN_TWO_HIGH_FLAT_IN_BETWEEN:
            return PatrolSimulationRegionFields.createUpDownTwoHighWithFlatBetween();
         case UP_DOWN_FOUR_HIGH_WITH_FLAT_CENTER:
            return PatrolSimulationRegionFields.createUpDownFourHighWithFlatCenter();
         case STAIRS:
            return PatrolSimulationRegionFields.createStairs();
         case SLAM_REAL_DATA:
            return slamDataset();
         case FLAT_GROUND:
         default:
            return PlanarRegionsList.flatGround(10.0);
      }
   }

   private PlanarRegionsList slamDataset()
   {
      PlanarRegionsList map = PlanarRegionsList.flatGround(10.0);
      PlanarRegionSLAMParameters parameters = new PlanarRegionSLAMParameters();
      map = PlanarRegionSLAM.slam(map, loadDataSet("20190710_174025_PlanarRegion"), parameters).getMergedMap();
      map = PlanarRegionSLAM.slam(map, loadDataSet("IntentionallyDrifted"), parameters).getMergedMap();
      map = PlanarRegionSLAM.slam(map, loadDataSet("20190710_174422_PlanarRegion"), parameters).getMergedMap();
      return map;
   }

   private PlanarRegionsList loadDataSet(String dataSetName)
   {
      Path openRobotics = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
      Path path = openRobotics.resolve("robot-environment-awareness/Data/PlanarRegion/20190710_SLAM_PlanarRegionFittingExamples/").resolve(dataSetName);
      return PlanarRegionFileTools.importPlanarRegionData(path.toFile());
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
