package us.ihmc.simulationconstructionset.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

//import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.simulationconstructionset.SimulationConstructionSetTest.class,
   us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationDispatcherClientTest.class
})

public class SimulationConstructionSetManualTestSuite
{
   public static void main(String[] args)
   {
      //new JUnitTestSuiteRunner(SimulationConstructionSetManualTestSuite.class);
   }
}

