package us.ihmc.simulationconstructionset.physics.featherstone;

import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.thread.ThreadTools;

/**
 * Tests simulation against closed-form dynamics
 */
public class FeatherstoneAlgorithmTest
{
   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   @Test
   public void testPendulumAgainstLagrangianCalculation()
   {
      double epsilon = 1e-5;

      SimplePendulumRobot pendulumRobot = new SimplePendulumRobot("pendulum", 1.2, -0.4, 0.1);
      pendulumRobot.setController(new DynamicsChecker(pendulumRobot, epsilon));

      SimulationConstructionSet scs = new SimulationConstructionSet(pendulumRobot, simulationTestingParameters);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   private class DynamicsChecker implements RobotController
   {
      private final YoVariableRegistry registry;
      private final RobotWithClosedFormDynamics robotWithClosedFormDynamics;
      private final double epsilon;

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
         robotWithClosedFormDynamics.assertStateIsCloseToLagrangianCalculation(epsilon);
      }
   }
}
