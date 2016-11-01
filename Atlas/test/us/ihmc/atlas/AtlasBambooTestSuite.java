package us.ihmc.atlas;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.atlas.posePlayback.AtlasPlaybackPoseSequenceTest.class,
   us.ihmc.atlas.utilities.kinematics.AtlasNumericalInverseKinematicsCalculatorWithRobotTest.class
})

public class AtlasBambooTestSuite
{
   public static void main(String[] args)
   {
//      JUnitTestSuiteGenerator.generateTestSuite(AtlasBambooTestSuite.class);
   }
}
