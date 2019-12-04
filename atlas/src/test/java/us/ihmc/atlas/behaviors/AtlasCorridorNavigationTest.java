package us.ihmc.atlas.behaviors;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.avatar.kinematicsSimulation.HumanoidKinematicsSimulation;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.tools.SimulatedREAModule;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pathPlanning.PlannerTestEnvironments;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

public class AtlasCorridorNavigationTest
{
   private static boolean VISUALIZE = Boolean.parseBoolean(System.getProperty("visualize")); // To visualize, pass -Dvisualize=true

   @BeforeAll
   static public void beforeAll()
   {
      if (VISUALIZE) JavaFXApplicationCreator.createAJavaFXApplication();
   }

   @AfterEach
   public void afterEach()
   {
      if (VISUALIZE) ThreadTools.sleepForever();
   }

   @Test
   public void testAtlasMakesItToGoalInTrickyCorridor()
   {
      new Thread(() -> {
         LogTools.info("Creating simulate REA module");
         new SimulatedREAModule(PlannerTestEnvironments.getTrickCorridor(), createRobotModel()).start();
      }).start();

      new Thread(() -> {
         LogTools.info("Creating simulation");
         boolean createYoVariableServer = false;
         HumanoidKinematicsSimulation.createForAutomatedTest(createRobotModel(), createYoVariableServer);
      }).start();

      new Thread(() -> {
         LogTools.info("Creating footstep toolbox");
         new MultiStageFootstepPlanningModule(createRobotModel(), null, false, DomainFactory.PubSubImplementation.INTRAPROCESS);
      }).start();

      SharedMemoryMessager messager = new SharedMemoryMessager(BehaviorModule.MessagerAPI);
      ExceptionTools.handle(() -> messager.startMessager(), DefaultExceptionHandler.RUNTIME_EXCEPTION);

      new Thread(() -> {
         LogTools.info("Creating behavior backpack");
         BehaviorModule.createForTest(createRobotModel(), messager);
      }).start();
   }

   private AtlasRobotModel createRobotModel()
   {
      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      return new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false, simulationContactPoints);
   }
}
