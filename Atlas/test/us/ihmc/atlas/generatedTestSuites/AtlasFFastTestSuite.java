package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.atlas.behaviorTests.AtlasHandPoseListBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasHeadOrientationBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasHighLevelStateBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasLookAtBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasPelvisPoseBehaviorTest.class
})

public class AtlasFFastTestSuite
{
   public static void main(String[] args)
   {
      new JUnitTestSuiteRunner(AtlasFFastTestSuite.class);
   }
}

