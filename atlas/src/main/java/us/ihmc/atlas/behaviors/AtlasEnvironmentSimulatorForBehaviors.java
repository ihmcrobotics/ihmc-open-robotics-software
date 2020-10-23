package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationParameters;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.tools.SimulatedREAModule;
import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

import java.util.function.Supplier;

public class AtlasEnvironmentSimulatorForBehaviors
{
   private static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;
   private PubSubImplementation pubSubMode = PubSubImplementation.FAST_RTPS;

   private static boolean LOG_TO_FILE = Boolean.parseBoolean(System.getProperty("log.to.file"));
   private static boolean CREATE_YOVARIABLE_SERVER = Boolean.parseBoolean(System.getProperty("create.yovariable.server"));

   private static final Supplier<PlanarRegionsList> ENVIRONMENT = BehaviorPlanarRegionEnvironments::createUpDownTwoHighWithFlatBetween;

   public AtlasEnvironmentSimulatorForBehaviors()
   {
      ThreadTools.startAThread(this::reaModule, "REAModule");
      ThreadTools.startAThread(this::kinematicSimulation, "KinematicsSimulation");
      ThreadTools.startAThread(this::footstepPlanningToolbox, "FootstepPlanningToolbox");
   }

   private void footstepPlanningToolbox()
   {
      LogTools.info("Starting footstep toolbox");
      FootstepPlanningModuleLauncher.createModule(createRobotModel(), DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   private void reaModule()
   {
      LogTools.info("Creating simulated REA module");
      SimulatedREAModule simulatedREAModule = new SimulatedREAModule(ENVIRONMENT.get(), createRobotModel(), pubSubMode);
      simulatedREAModule.start();
   }

   private void kinematicSimulation()
   {
      LogTools.info("Creating simulation");
      HumanoidKinematicsSimulationParameters kinematicsSimulationParameters = new HumanoidKinematicsSimulationParameters();
      kinematicsSimulationParameters.setPubSubImplementation(pubSubMode);
      kinematicsSimulationParameters.setLogToFile(LOG_TO_FILE);
      kinematicsSimulationParameters.setCreateYoVariableServer(CREATE_YOVARIABLE_SERVER);
      AtlasKinematicSimulation.create(createRobotModel(), kinematicsSimulationParameters);
   }

   private AtlasRobotModel createRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      return new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false, simulationContactPoints);
   }

   public static void main(String[] args)
   {
      new AtlasEnvironmentSimulatorForBehaviors();
   }
}
