package us.ihmc.exampleSimulations.genericQuadruped.controller.position;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.position.QuadrupedPositionCrawlTurningVelocityTest;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

import java.io.IOException;

@Disabled
public class GenericQuadrupedPositionCrawlTurningVelocityTest extends QuadrupedPositionCrawlTurningVelocityTest
{
   @Override
   public QuadrupedTestFactory createQuadrupedTestFactory()
   {
      return new GenericQuadrupedTestFactory();
   }
   
   @Override
   @Test
   public void testTurnInPlaceRegularSpeed() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testTurnInPlaceRegularSpeed();
   }
   
   //"Turn in place slowly still fails due to CoM shifting outside support polygon. Need to fix it..."
   @Override
   @Test
   public void testTurnInPlaceSlowly() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testTurnInPlaceSlowly();
   }
   
   @Override
   @Test
   public void testWalkingBackwardStoppingAndTurning() throws SimulationExceededMaximumTimeException, ControllerFailureException, IOException
   {
      super.testWalkingBackwardStoppingAndTurning();
   }
}
