package us.ihmc.simulationconstructionset.utilities;

import org.junit.runner.*;
import org.junit.runners.*;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

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
