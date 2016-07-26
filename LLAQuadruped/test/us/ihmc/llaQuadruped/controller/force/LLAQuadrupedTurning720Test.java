package us.ihmc.llaQuadruped.controller.force;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.llaQuadruped.LLAQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedXGaitTurning720Test;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = TestPlanTarget.InDevelopment)
public class LLAQuadrupedTurning720Test extends QuadrupedXGaitTurning720Test
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new LLAQuadrupedTestFactory();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 80.0)
   @Test(timeout = 200000)
   public void rotate720InPlaceRight() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.rotate720InPlaceRight();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 80.0)
   @Test(timeout = 200000)
   public void rotate720InPlaceLeft() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.rotate720InPlaceLeft();
   }
}
