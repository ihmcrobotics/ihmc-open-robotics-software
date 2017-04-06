package us.ihmc.simulationconstructionset.physics.featherstone;

import org.junit.Test;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.thread.ThreadTools;

public class PinJointPhysicsTest
{
   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   @Test
   public void testPendulumAgainstLagrangianCalculation()
   {
      double epsilon = 1e-6;

      SimplePendulumRobot pendulumRobot = new SimplePendulumRobot("pendulum", 1.2, -0.4);
      pendulumRobot.setController(new SimulationChecker(pendulumRobot, epsilon));

      SimulationConstructionSet scs = new SimulationConstructionSet(pendulumRobot, simulationTestingParameters);
      scs.startOnAThread();

      ThreadTools.sleepForever();
   }

   private class SimulationChecker implements RobotController
   {
      private final YoVariableRegistry registry;
      private final RobotWithExplicitDynamics robotWithExplicitDynamics;
      private final double epsilon;

      public SimulationChecker(RobotWithExplicitDynamics robotWithExplicitDynamics, double epsilon)
      {
         registry = new YoVariableRegistry(robotWithExplicitDynamics.getName() + "Registry");
         this.robotWithExplicitDynamics = robotWithExplicitDynamics;
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
         return robotWithExplicitDynamics.getName();
      }

      @Override
      public String getDescription()
      {
         return getName();
      }

      @Override
      public void doControl()
      {
         robotWithExplicitDynamics.assertStateIsCloseToLagrangianCalculation(epsilon);
      }
   }
}
