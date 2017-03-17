package us.ihmc.avatar.controllerAPI;

import org.junit.Test;

import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;

public abstract class EndToEndHandLoadBearingTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   @ContinuousIntegrationTest(estimatedDuration = 25.0)
   @Test(timeout = 50000)
   public void testUsingHand() throws Exception
   {

   }
}
