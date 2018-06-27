package us.ihmc.quadrupedRobotics.controller.position;

import org.junit.After;
import org.junit.Before;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.Random;

public abstract class QuadrupedPositionRandomWalkingTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;

   @Before
   public void setup()
   {
      try
      {
         MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

         QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(WholeBodyControllerCoreMode.INVERSE_KINEMATICS);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedTestYoVariables(conductor.getScs());
         stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   @After
   public void tearDown()
   {
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private double randomValidVelocity(Random random)
   {
      double velocity = random.nextDouble() * 2.0 - 1.0;
      return velocity * 0.8;
   }

   private double randomValidYawRate(Random random)
   {
      return random.nextDouble() * 1.0 - 0.5;
   }

   private double randomSimulationDuration(Random random)
   {
      return random.nextDouble() * 2.0 + 0.25;
   }

   public void testExtremeRandomWalking() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      stepTeleopManager.requestXGait();

      Random random = new Random(1447L);
      double runningDuration = variables.getYoTime().getDoubleValue();

      for (int i = 0; i < 10; i++)
      {
         runningDuration += randomSimulationDuration(random);
         stepTeleopManager.setDesiredVelocity(randomValidVelocity(random) * 2.0, 0.0, randomValidYawRate(random) * 2.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();
      }
   }

   public void testWalkingRandomly() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      Random random = new Random(1547L);
      double runningDuration = variables.getYoTime().getDoubleValue();

      for (int i = 0; i < 10; i++)
      {
         runningDuration += randomSimulationDuration(random);
         stepTeleopManager.setDesiredVelocity(randomValidVelocity(random), 0.0, randomValidYawRate(random));
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();

         runningDuration += 1.0;
         stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();
      }
   }

   public void testWalkingAtRandomSpeedsWithStops() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      Random random = new Random(2456L);
      double runningDuration = variables.getYoTime().getDoubleValue();

      stepTeleopManager.requestXGait();

      for (int i = 0; i < 6; i++)
      {
         double randomSimulationDuration = randomSimulationDuration(random);
         double randomValidVelocity = randomValidVelocity(random);
         runningDuration += randomSimulationDuration;
         stepTeleopManager.setDesiredVelocity(randomValidVelocity, 0.0, 0.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();

         runningDuration += 1.0;
         stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();
      }
   }

   public void testWalkingRandomVelocitiesStoppingAndTurning() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      stepTeleopManager.requestXGait();

      Random random = new Random(1557L);
      double runningDuration = variables.getYoTime().getDoubleValue();

      for (int i = 0; i < 6; i++)
      {
         double randomSimulationDuration = randomSimulationDuration(random);
         double randomValidVelocity = randomValidVelocity(random);
         runningDuration += randomSimulationDuration;
         stepTeleopManager.setDesiredVelocity(randomValidVelocity, 0.0, 0.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();

         runningDuration += 1.0;
         stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();

         randomSimulationDuration = randomSimulationDuration(random);
         double randomValidYawRate = randomValidYawRate(random);
         runningDuration += randomSimulationDuration;
         stepTeleopManager.setDesiredVelocity(0.0, 0.0, randomValidYawRate);
         conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), runningDuration));
         conductor.simulate();
      }
   }
}
