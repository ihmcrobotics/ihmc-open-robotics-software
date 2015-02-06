package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.runner.BambooTestSuiteRunner;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.atlas.AtlasScriptingTest.class,
   us.ihmc.atlas.behaviorTests.AtlasFootstepListBehaviorTest.class,
   us.ihmc.atlas.behaviorTests.AtlasGraspPieceOfDebrisBehaviorTest.class
})

public class AtlasDFastTestSuite
{
   public static void main(String[] args)
   {
      new BambooTestSuiteRunner(AtlasDFastTestSuite.class);
   }
}

