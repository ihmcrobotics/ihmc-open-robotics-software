package us.ihmc.utilities;

import org.junit.runner.*;
import org.junit.runners.*;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.simulationconstructionset.SimulationConstructionSetTestFest.class
})

public class SimulationConstructionSetBambooFestTestSuite
{
   public static void main(String[] args)
   {
      JUnitTestSuiteConstructor.generateTestSuite(SimulationConstructionSetBambooFestTestSuite.class);
   }
}
