package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.runner.BambooTestSuiteRunner;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.atlas.behaviorTests.AtlasChestOrientationBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasComHeightBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasTurnValveBehaviorTest.class
})

public class AtlasCVideoTestSuite
{
   public static void main(String[] args)
   {
      new BambooTestSuiteRunner(AtlasCVideoTestSuite.class);
   }
}

