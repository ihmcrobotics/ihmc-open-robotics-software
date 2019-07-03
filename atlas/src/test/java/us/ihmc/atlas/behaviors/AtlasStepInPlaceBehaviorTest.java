package us.ihmc.atlas.behaviors;

import org.junit.jupiter.api.*;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.StepInPlaceBehavior;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

@SuppressWarnings("unused")
@Tag("humanoid-behaviors")
public class AtlasStepInPlaceBehaviorTest
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private AtlasBehaviorTestYoVariables variables;
   private GoalOrientedTestConductor conductor;
   private AvatarSimulation avatarSimulation;
   private AtlasRobotModel robotModel;

   @Test
   public void testStepInPlaceBehavior()
   {
      SharedMemoryMessager messager = new SharedMemoryMessager(BehaviorModule.getBehaviorAPI());
      ExceptionTools.handle(() -> messager.startMessager(), DefaultExceptionHandler.RUNTIME_EXCEPTION);

      LogTools.info("Creating behavior module");
      BehaviorModule.createForTest(robotModel, messager);

      LogTools.info("Creating behavior messager");

      LogTools.info("Set stepping true");
      messager.submitMessage(StepInPlaceBehavior.API.Stepping, true);

      AtlasTestScripts.takeSteps(conductor, variables, 4, 6.0);

      messager.submitMessage(StepInPlaceBehavior.API.Stepping, false);
      messager.submitMessage(StepInPlaceBehavior.API.Abort, true);

      AtlasTestScripts.wait(conductor, variables, 3.0);

      AtlasTestScripts.holdDoubleSupport(conductor, variables, 3.0, 6.0);
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      conductor.concludeTesting();
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @BeforeEach
   public void setUp()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      SimulationConstructionSet scs = AtlasBehaviorSimulation.createForAutomatedTest(robotModel, new FlatGroundEnvironment());
      variables = new AtlasBehaviorTestYoVariables(scs);
      conductor = new GoalOrientedTestConductor(scs, simulationTestingParameters);
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      AtlasTestScripts.standUp(conductor, variables);
   }

   @AfterAll
   public static void printMemoryUsageAfterClass()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(AtlasStepInPlaceBehaviorTest.class + " after class.");
   }
}
