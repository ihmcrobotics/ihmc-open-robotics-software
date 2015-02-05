package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.atlas.AtlasScriptingTest.class,
   us.ihmc.atlas.behaviorTests.AtlasBehaviorDispatcherTest.class,
   us.ihmc.atlas.behaviorTests.AtlasChestOrientationBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasComHeightBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasDetectCollisionUsingWristSensorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasFingerStateBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasFootPoseBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasFootstepListBehaviorTest.class
})

public class AtlasCFastTestSuite
{
   public static void main(String[] args)
   {
      new JUnitTestSuiteRunner(AtlasCFastTestSuite.class);
   }
}

