package us.ihmc.simulationconstructionset.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.simulationconstructionset.LinkGraphicsTest.class,
   us.ihmc.simulationconstructionset.SimulationConstructionSetMemoryReclamationTest.class,
   us.ihmc.simulationconstructionset.SimulationConstructionSetSetupTest.class,
   us.ihmc.simulationconstructionset.SimulationConstructionSetUsingDirectCallsTest.class
})

public class SimulationConstructionSetAUITestSuite
{
   public static void main(String[] args)
   {
      new JUnitTestSuiteRunner(SimulationConstructionSetAUITestSuite.class);
   }
}

