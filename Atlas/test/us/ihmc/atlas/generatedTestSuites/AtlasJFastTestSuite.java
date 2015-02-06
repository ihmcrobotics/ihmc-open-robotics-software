package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.code.unitTesting.runner.JUnitTestSuiteRunner;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.atlas.calib.KinematicCalibrationHeadLoopResidualTest.class,
   us.ihmc.atlas.controllers.AtlasFootstepGeneratorTest.class,
   us.ihmc.atlas.controllers.responses.AtlasHandPoseStatusTest.class
})

public class AtlasJFastTestSuite
{
   public static void main(String[] args)
   {
      new JUnitTestSuiteRunner(AtlasJFastTestSuite.class);
   }
}

