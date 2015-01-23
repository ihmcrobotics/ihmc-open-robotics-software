package us.ihmc.simulationconstructionset.utilities;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.JUnitTestSuiteGenerator;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.simulationconstructionset.SimulationConstructionSetFestTest.class
})

public class SimulationConstructionSetBambooFestTestSuiteNightly
{
   public static void main(String[] args)
   {
      JUnitTestSuiteGenerator.generateTestSuite(SimulationConstructionSetBambooFestTestSuiteNightly.class);
   }
}
