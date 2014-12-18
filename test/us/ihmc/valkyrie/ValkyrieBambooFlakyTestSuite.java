package us.ihmc.valkyrie;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.valkyrie.simulation.ValkyriePosePlaybackDemoTest.class
})

public class ValkyrieBambooFlakyTestSuite
{
   public static void main(String[] args)
   {
      JUnitTestSuiteConstructor.generateTestSuite(ValkyrieBambooFlakyTestSuite.class);
   }
}