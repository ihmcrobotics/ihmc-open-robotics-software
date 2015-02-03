package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

//import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.atlas.roughTerrainWalking.AtlasFootExplorationTest.class
})

public class AtlasFootExplorationTestSuite
{
   public static void main(String[] args)
   {
      //new JUnitTestSuiteRunner(AtlasFootExplorationTestSuite.class);
   }
}

