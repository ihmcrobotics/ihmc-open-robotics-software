package us.ihmc.simulationconstructionset.utilities;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.simulationconstructionset.SimulationConstructionSetFestTest.class
})

public class SimulationConstructionSetBambooFestTestSuiteNightly
{
   public static void main(String[] args)
   {
      JUnitTestSuiteConstructor.generateTestSuite(SimulationConstructionSetBambooFestTestSuiteNightly.class);
   }
}
