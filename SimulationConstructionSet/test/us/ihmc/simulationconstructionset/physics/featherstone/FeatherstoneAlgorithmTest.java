package us.ihmc.simulationconstructionset.physics.featherstone;

import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

import static junit.framework.TestCase.fail;

/**
 * Tests simulation against closed-form dynamics
 */
public class FeatherstoneAlgorithmTest
{
   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   @Test
   public void testSinglePendulumAgainstLagrangianCalculation()
   {
      double epsilon = 1e-5;
      SinglePendulumRobot pendulumRobot = new SinglePendulumRobot("pendulum", 1.2, -0.4);
      testAgainstLagrangianCalculation(pendulumRobot, epsilon);
   }

   @Test
   public void testDoublePendulumAgainstLagrangianCalculation()
   {
      double epsilon = 1e-4;
      DoublePendulumRobot pendulumRobot = new DoublePendulumRobot("doublePendulum", 1.2, -0.4, -0.2, 0.5);
      testAgainstLagrangianCalculation(pendulumRobot, epsilon);
   }

   private void testAgainstLagrangianCalculation(RobotWithClosedFormDynamics robotWithClosedFormDynamics, double epsilon)
   {
      robotWithClosedFormDynamics.setController(new DynamicsChecker(robotWithClosedFormDynamics, epsilon));

      SimulationConstructionSet scs = new SimulationConstructionSet(robotWithClosedFormDynamics, simulationTestingParameters);
      scs.startOnAThread();
      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 45.0);

      try
      {
         blockingSimulationRunner.simulateAndBlock(15.0);
      }
      catch(Exception e)
      {
         fail();
      }
   }

   private class DynamicsChecker implements RobotController
   {
      private final YoVariableRegistry registry;
      private final RobotWithClosedFormDynamics robotWithClosedFormDynamics;
      private final double epsilon;
      private int numberOfTicksToWait = 2;

      public DynamicsChecker(RobotWithClosedFormDynamics robotWithClosedFormDynamics, double epsilon)
      {
         registry = new YoVariableRegistry(robotWithClosedFormDynamics.getName() + "Registry");
         this.robotWithClosedFormDynamics = robotWithClosedFormDynamics;
         this.epsilon = epsilon;
      }

      @Override
      public void initialize()
      {

      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return robotWithClosedFormDynamics.getName();
      }

      @Override
      public String getDescription()
      {
         return getName();
      }

      @Override
      public void doControl()
      {
         if(numberOfTicksToWait == 0)
            robotWithClosedFormDynamics.assertStateIsCloseToLagrangianCalculation(epsilon);
         else
            numberOfTicksToWait--;
      }
   }
}
