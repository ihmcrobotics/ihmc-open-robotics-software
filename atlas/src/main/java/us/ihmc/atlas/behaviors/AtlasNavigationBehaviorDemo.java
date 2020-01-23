package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulationParameters;
import us.ihmc.commons.thread.Notification;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.tools.PlanarRegionsMappingModule;
import us.ihmc.humanoidBehaviors.tools.SimulatedREAModule;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.PlannerTestEnvironments;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

import java.util.function.Supplier;

public class AtlasNavigationBehaviorDemo
{
   private static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;
   private PubSubImplementation pubSubMode = PubSubImplementation.INTRAPROCESS;

   private static boolean LOG_TO_FILE = Boolean.parseBoolean(System.getProperty("log.to.file"));
   private static boolean CREATE_YOVARIABLE_SERVER = Boolean.parseBoolean(System.getProperty("create.yovariable.server"));

   private static final Supplier<PlanarRegionsList> TRICKY_CORRIDOR = PlannerTestEnvironments::getTrickCorridorWidened;
   private static final Supplier<PlanarRegionsList> MAZE_CORRIDOR = PlannerTestEnvironments::getMazeCorridor;

   private static final Supplier<PlanarRegionsList> ENVIRONMENT = MAZE_CORRIDOR;

   private Notification slamUpdated;

   public AtlasNavigationBehaviorDemo()
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      new Thread(() -> {
         LogTools.info("Creating simulated REA module");
         SimulatedREAModule simulatedREAModule = new SimulatedREAModule(ENVIRONMENT.get(), createRobotModel(), pubSubMode);
         simulatedREAModule.start();
      }).start();

      new Thread(() ->
      {
         LogTools.info("Creating planar regions mapping module");
         PlanarRegionsMappingModule planarRegionsMappingModule = new PlanarRegionsMappingModule(pubSubMode);
         slamUpdated = planarRegionsMappingModule.getSlamUpdated();
      }).start();

      new Thread(() ->
      {
         LogTools.info("Creating simulation");
         HumanoidKinematicsSimulationParameters kinematicsSimulationParameters = new HumanoidKinematicsSimulationParameters();
         kinematicsSimulationParameters.setPubSubImplementation(pubSubMode);
         kinematicsSimulationParameters.setLogToFile(LOG_TO_FILE);
         kinematicsSimulationParameters.setCreateYoVariableServer(CREATE_YOVARIABLE_SERVER);
         AtlasKinematicSimulation.create(createRobotModel(), kinematicsSimulationParameters);
      }).start();

      new Thread(() ->
      {
         LogTools.info("Creating behavior module");
         BehaviorModule.createInterprocess(createRobotModel());
      }).start();

      LogTools.info("Creating behavior user interface");
      Messager behaviorMessager = RemoteBehaviorInterface.createForUI("localhost");
      AtlasRobotModel robotModel = createRobotModel();
      new BehaviorUI(behaviorMessager, robotModel, pubSubMode);
   }

   private AtlasRobotModel createRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      return new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false, simulationContactPoints);
   }

   public static void main(String[] args)
   {
      new AtlasNavigationBehaviorDemo();
   }
}
