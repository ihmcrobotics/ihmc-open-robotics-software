package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.behaviors.scsSensorSimulation.SCSLidarAndCameraSimulator;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.environments.RealisticLabTerrainBuilder;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepBehaviorUI;
import us.ihmc.humanoidBehaviors.ui.simulation.EnvironmentInitialSetup;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.rtps.impl.fastRTPS.FastRTPSDomain;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.tools.processManagement.JavaProcessManager;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

import java.util.ArrayList;
import java.util.Random;

import static us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments.TRIPLE_PLATFORM_HEIGHT;

public class AtlasLookAndStepBehaviorDemo
{
   private static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;

   private static boolean LOG_TO_FILE = Boolean.parseBoolean(System.getProperty("log.to.file"));
   private static boolean CREATE_YOVARIABLE_SERVER = Boolean.parseBoolean(System.getProperty("create.yovariable.server"));
   private static boolean USE_DYNAMICS_SIMULATION = Boolean.parseBoolean(System.getProperty("use.dynamics.simulation"));
   private static boolean RUN_LIDAR_AND_CAMERA_SIMULATION = Boolean.parseBoolean(System.getProperty("run.lidar.and.camera.simulation"));
   private static boolean USE_INTERPROCESS_ROS2 = Boolean.parseBoolean(System.getProperty("use.interprocess.ros2"));
   private static boolean USE_INTERPROCESS_KRYO = Boolean.parseBoolean(System.getProperty("use.interprocess.kryo"));
   private static boolean RUN_REALSENSE_SLAM = Boolean.parseBoolean(System.getProperty("run.realsense.slam", "true"));
   private static boolean RUN_LIDAR_REA = Boolean.parseBoolean(System.getProperty("run.lidar.rea", "true"));
   private static boolean SHOW_REALSENSE_SLAM_UIS = Boolean.parseBoolean(System.getProperty("show.realsense.slam.uis"));
   private static boolean USE_ADDITIONAL_CONTACT_POINTS = Boolean.parseBoolean(System.getProperty("use.additional.contact.points"));
   private static int ENVIRONMENT = Integer.parseInt(System.getProperty("environment", "-1"));

   private final CommunicationMode communicationModeROS2 = USE_INTERPROCESS_ROS2 ? CommunicationMode.INTERPROCESS : CommunicationMode.INTRAPROCESS;
   private final CommunicationMode communicationModeKryo = USE_INTERPROCESS_KRYO ? CommunicationMode.INTERPROCESS : CommunicationMode.INTRAPROCESS;
   private final Runnable simulation = USE_DYNAMICS_SIMULATION ? this::dynamicsSimulation : this::kinematicSimulation;

   private final ArrayList<EnvironmentInitialSetup> environmentInitialSetups = new ArrayList<>();

   {
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockAngledStepsUpAndDown,
                                                               0.0, 0.0, 0.0, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockAngledStepsUpAndDown,
                                                               0.0, Math.PI, 6.0, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockAngledStepsUpAndDown,
                                                               TRIPLE_PLATFORM_HEIGHT, Math.PI, 2.8, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockAngledStepsUpAndDown,
                                                               TRIPLE_PLATFORM_HEIGHT, 0.0,2.8, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockStepsUpAndDown,
                                                               0.0, 0.0, 0.0, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockStepsUpAndDown,
                                                               0.0, Math.PI, 6.0, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockStepsUpAndDown,
                                                               TRIPLE_PLATFORM_HEIGHT, Math.PI, 2.8, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockStepsUpAndDown,
                                                               TRIPLE_PLATFORM_HEIGHT, 0.0, 2.8, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateRealisticEasierStartingBlockRegions,
                                                               RealisticLabTerrainBuilder.PALLET_HEIGHT * 2.0, 0.0, 0.0, 0.0));
      environmentInitialSetups.add(new EnvironmentInitialSetup(BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockStepsUpAndDown,
                                                               0.0, 0.0, 0.0, 0.0));

   }
   private final Random random = new Random();
   private final EnvironmentInitialSetup environmentInitialSetup
         = environmentInitialSetups.get(ENVIRONMENT > 0 ? ENVIRONMENT : random.nextInt(environmentInitialSetups.size()));

   private BehaviorUI behaviorUI;
   private final BehaviorModule behaviorModule;
   private AtlasPerceptionSimulation perceptionStack;
   private SCSLidarAndCameraSimulator lidarAndCameraSimulator;
   private AtlasDynamicsSimulation dynamicSimulation;
   private HumanoidKinematicsSimulation kinematicsSimulation;

   public AtlasLookAndStepBehaviorDemo()
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      ThreadTools.startAsDaemon(() -> perceptionStack = new AtlasPerceptionSimulation(communicationModeROS2,
                                                                                      environmentInitialSetup.getPlanarRegionsSupplier().get(),
                                                                                      RUN_REALSENSE_SLAM,
                                                                                      SHOW_REALSENSE_SLAM_UIS,
                                                                                      RUN_LIDAR_REA,
                                                                                      createRobotModel()),
                                "PerceptionStack");
      ThreadTools.startAsDaemon(simulation, "Simulation");

      if (RUN_LIDAR_AND_CAMERA_SIMULATION)
         ThreadTools.startAsDaemon(this::lidarAndCameraSimulator, "LidarAndCamera");

      BehaviorUIRegistry behaviorRegistry = BehaviorUIRegistry.of(LookAndStepBehaviorUI.DEFINITION);

      behaviorModule = new BehaviorModule(behaviorRegistry, createRobotModel(), communicationModeROS2, communicationModeKryo);

      LogTools.info("Creating behavior user interface");
      Messager behaviorMessager;
      if (communicationModeKryo == CommunicationMode.INTERPROCESS)
      {
         behaviorMessager = RemoteBehaviorInterface.createForUI(behaviorRegistry, "localhost");
      }
      else
      {
         behaviorMessager = behaviorModule.getMessager();
      }
      behaviorUI = new BehaviorUI(behaviorRegistry, behaviorMessager, createRobotModel(), communicationModeROS2.getPubSubImplementation());
      behaviorUI.addOnCloseRequestListener(() -> ThreadTools.startAThread(() -> {
         destroy();
         Runtime.getRuntime().exit(0);
      }, "DestroyViaUI"));

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "DestroyViaKill"));
   }

   private void lidarAndCameraSimulator()
   {
      lidarAndCameraSimulator = new SCSLidarAndCameraSimulator(communicationModeROS2.getPubSubImplementation(),
                                                               createCommonAvatarEnvironment(),
                                                               createRobotModel());
   }

   private void dynamicsSimulation()
   {
      LogTools.info("Creating dynamics simulation");
      int recordFrequencySpeedup = 50; // Increase to 10 when you want the sims to run a little faster and don't need all of the YoVariable data.
      int dataBufferSize = 10; // Reduce memory footprint; in this demo we only care about dynamics output
      dynamicSimulation = AtlasDynamicsSimulation.create(createRobotModel(),
                                                         createCommonAvatarEnvironment(),
                                                         environmentInitialSetup.getGroundZ(),
                                                         environmentInitialSetup.getInitialX(),
                                                         environmentInitialSetup.getInitialY(),
                                                         environmentInitialSetup.getInitialYaw(),
                                                         communicationModeROS2.getPubSubImplementation(),
                                                         recordFrequencySpeedup,
                                                         dataBufferSize,
                                                         LOG_TO_FILE);
      dynamicSimulation.simulate();
   }

   private void kinematicSimulation()
   {
      LogTools.info("Creating kinematics simulation");
      HumanoidKinematicsSimulationParameters kinematicsSimulationParameters = new HumanoidKinematicsSimulationParameters();
      kinematicsSimulationParameters.setPubSubImplementation(communicationModeROS2.getPubSubImplementation());
      kinematicsSimulationParameters.setLogToFile(LOG_TO_FILE);
      kinematicsSimulationParameters.setCreateYoVariableServer(CREATE_YOVARIABLE_SERVER);
      kinematicsSimulationParameters.setInitialGroundHeight(environmentInitialSetup.getGroundZ());
      kinematicsSimulationParameters.setInitialRobotYaw(environmentInitialSetup.getInitialYaw());
      kinematicsSimulationParameters.setInitialRobotX(environmentInitialSetup.getInitialX());
      kinematicsSimulationParameters.setInitialRobotY(environmentInitialSetup.getInitialY());
      kinematicsSimulation = AtlasKinematicSimulation.create(createRobotModel(), kinematicsSimulationParameters);
   }

   private CommonAvatarEnvironmentInterface createCommonAvatarEnvironment()
   {
      String environmentName = PlanarRegionsListDefinedEnvironment.class.getSimpleName();
      YoAppearanceTexture cinderBlockTexture = new YoAppearanceTexture("sampleMeshes/cinderblock.png");
      return new PlanarRegionsListDefinedEnvironment(environmentName,
                                                     environmentInitialSetup.getPlanarRegionsSupplier().get(),
                                                     cinderBlockTexture,
                                                     0.02,
                                                     false);
   }

   private AtlasRobotModel createRobotModel()
   {
      if (USE_ADDITIONAL_CONTACT_POINTS)
      {
         FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
         return new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false, simulationContactPoints);
      }
      else
      {
         return new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false);
      }
   }

   private boolean destroyed = false;

   private void destroy()
   {
      if (!destroyed)
      {
         LogTools.info("Shutting down");
         behaviorUI.closeMessager();
         behaviorModule.destroy();
         perceptionStack.destroy();
         if (RUN_LIDAR_AND_CAMERA_SIMULATION)
            lidarAndCameraSimulator.destroy();
         if (USE_DYNAMICS_SIMULATION)
            dynamicSimulation.destroy();
         else
            kinematicsSimulation.destroy();

         DomainFactory.getDomain(communicationModeROS2.getPubSubImplementation()).stopAll();

         destroyed = true;
      }
   }

   public static void main(String[] args)
   {
      if (LOG_TO_FILE)
      {
         JavaProcessManager.teeToLogFile(AtlasLookAndStepBehaviorDemo.class);
      }
      else
      {
         new AtlasLookAndStepBehaviorDemo();
      }
   }
}
