package us.ihmc.atlas.behaviors.tools;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.behaviors.AtlasDynamicsSimulation;
import us.ihmc.atlas.behaviors.AtlasKinematicSimulation;
import us.ihmc.atlas.behaviors.scsSensorSimulation.SCSLidarAndCameraSimulator;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationParameters;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTexture;
import us.ihmc.humanoidBehaviors.ui.simulation.EnvironmentInitialSetup;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.tools.processManagement.JavaProcessManager;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

import java.util.ArrayList;
import java.util.Random;

public class AtlasSimulationBasics
{
   protected static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   protected static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;

   protected static boolean LOG_TO_FILE = Boolean.parseBoolean(System.getProperty("log.to.file"));
   protected static boolean CREATE_YOVARIABLE_SERVER = Boolean.parseBoolean(System.getProperty("create.yovariable.server"));
   protected static boolean USE_DYNAMICS_SIMULATION = Boolean.parseBoolean(System.getProperty("use.dynamics.simulation"));
   protected static boolean RUN_LIDAR_AND_CAMERA_SIMULATION = Boolean.parseBoolean(System.getProperty("run.lidar.and.camera.simulation"));
   protected boolean CREATE_MORE_FOOT_CONTACT_POINTS = Boolean.parseBoolean(System.getProperty("create.more.foot.contact.points"));
   protected boolean CREATE_HAND_CONTACT_POINTS = Boolean.parseBoolean(System.getProperty("create.hand.contact.points"));
   protected int numberOfContactPointsX = 8;
   protected int numberOfContactPointsY = 3;
   private static boolean USE_INTERPROCESS_ROS2 = Boolean.parseBoolean(System.getProperty("use.interprocess.ros2"));
   private static boolean USE_INTERPROCESS_KRYO = Boolean.parseBoolean(System.getProperty("use.interprocess.kryo"));

   protected static final CommunicationMode COMMUNICATION_MODE_ROS2 = USE_INTERPROCESS_ROS2 ? CommunicationMode.INTERPROCESS : CommunicationMode.INTRAPROCESS;
   protected static final CommunicationMode COMMUNICATION_MODE_KRYO = USE_INTERPROCESS_KRYO ? CommunicationMode.INTERPROCESS : CommunicationMode.INTRAPROCESS;

   protected final Runnable simulation = USE_DYNAMICS_SIMULATION ? this::dynamicsSimulation : this::kinematicSimulation;

   private static int ENVIRONMENT = Integer.parseInt(System.getProperty("environment", "-1"));
   protected final ArrayList<EnvironmentInitialSetup> environmentInitialSetups = new ArrayList<>();
   protected EnvironmentInitialSetup environmentInitialSetup;

   protected AtlasDynamicsSimulation dynamicSimulation;
   protected HumanoidKinematicsSimulation kinematicsSimulation;

   protected boolean destroyed = false;
   protected SCSLidarAndCameraSimulator lidarAndCameraSimulator;

   protected void selectEnvironment()
   {
      Random random = new Random();
      environmentInitialSetup = environmentInitialSetups.get(ENVIRONMENT >= 0 ? ENVIRONMENT : random.nextInt(environmentInitialSetups.size()));
   }

   protected void dynamicsSimulation()
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
                                                         COMMUNICATION_MODE_ROS2.getPubSubImplementation(),
                                                         recordFrequencySpeedup,
                                                         dataBufferSize,
                                                         LOG_TO_FILE);
      dynamicSimulation.simulate();
   }

   protected void kinematicSimulation()
   {
      LogTools.info("Creating kinematics simulation");
      HumanoidKinematicsSimulationParameters kinematicsSimulationParameters = new HumanoidKinematicsSimulationParameters();
      kinematicsSimulationParameters.setPubSubImplementation(COMMUNICATION_MODE_ROS2.getPubSubImplementation());
      kinematicsSimulationParameters.setLogToFile(LOG_TO_FILE);
      kinematicsSimulationParameters.setCreateYoVariableServer(CREATE_YOVARIABLE_SERVER);
      kinematicsSimulationParameters.setInitialGroundHeight(environmentInitialSetup.getGroundZ());
      kinematicsSimulationParameters.setInitialRobotYaw(environmentInitialSetup.getInitialYaw());
      kinematicsSimulationParameters.setInitialRobotX(environmentInitialSetup.getInitialX());
      kinematicsSimulationParameters.setInitialRobotY(environmentInitialSetup.getInitialY());
      kinematicsSimulation = AtlasKinematicSimulation.create(createRobotModel(), kinematicsSimulationParameters);
   }

   protected CommonAvatarEnvironmentInterface createCommonAvatarEnvironment()
   {
      if (environmentInitialSetup.hasCommonAvatarEnvironmentInterface())
      {
         return environmentInitialSetup.getCommonAvatarEnvironmentInterface();
      }
      else
      {
         String environmentName = PlanarRegionsListDefinedEnvironment.class.getSimpleName();
         YoAppearanceTexture cinderBlockTexture = new YoAppearanceTexture("sampleMeshes/cinderblock.png");
         return new PlanarRegionsListDefinedEnvironment(environmentName,
                                                        environmentInitialSetup.getPlanarRegionsSupplier().get(),
                                                        cinderBlockTexture,
                                                        0.02,
                                                        false);
      }
   }

   protected AtlasRobotModel createRobotModel()
   {
      if (CREATE_MORE_FOOT_CONTACT_POINTS)
      {
         FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values,
                                                                                                        numberOfContactPointsX,
                                                                                                        numberOfContactPointsY,
                                                                                                        true,
                                                                                                        true);
         return new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false, simulationContactPoints, CREATE_HAND_CONTACT_POINTS);
      }
      else
      {
         return new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false, CREATE_HAND_CONTACT_POINTS);
      }
   }

   protected void lidarAndCameraSimulator()
   {
      lidarAndCameraSimulator = new SCSLidarAndCameraSimulator(COMMUNICATION_MODE_ROS2.getPubSubImplementation(),
                                                               createCommonAvatarEnvironment(),
                                                               createRobotModel());
   }

   protected boolean destroy()
   {
      if (!destroyed)
      {
         LogTools.info("Shutting down");
         if (USE_DYNAMICS_SIMULATION)
            dynamicSimulation.destroy();
         else
            kinematicsSimulation.destroy();

         if (lidarAndCameraSimulator != null)
            lidarAndCameraSimulator.destroy();

         DomainFactory.getDomain(COMMUNICATION_MODE_ROS2.getPubSubImplementation()).stopAll();

         destroyed = true;
         return true;
      }

      return false;
   }

   public static void runOrLogToFile(Class<?> clazz)
   {
      if (LOG_TO_FILE)
      {
         JavaProcessManager.teeToLogFile(clazz);
      }
      else
      {
         ExceptionTools.handle(clazz::newInstance, DefaultExceptionHandler.RUNTIME_EXCEPTION);
      }
   }
}
