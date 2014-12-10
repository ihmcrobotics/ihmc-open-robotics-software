package us.ihmc.atlas;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses
({
   us.ihmc.atlas.AtlasMultiContactTest.class,
   us.ihmc.atlas.controllers.AtlasFootstepGeneratorTest.class,
   us.ihmc.atlas.momentumBasedControl.AtlasOptimizationMomentumControlModuleTest.class,
   us.ihmc.atlas.networking.AtlasNetworkingCommandReaderTest.class,
   us.ihmc.atlas.networkProcessor.depthData.AtlasDepthDataProcessorTest.class,
   us.ihmc.atlas.networkProcessor.downlinkSerializers.AtlasJointConfigurationDataSerializerTest.class,
   us.ihmc.atlas.posePlayback.AtlasPlaybackPoseSequenceTest.class,
   us.ihmc.atlas.utilities.kinematics.AtlasNumericalInverseKinematicsCalculatorWithRobotTest.class
})

public class AtlasBambooTestSuite
{
   public static void main(String[] args)
   {
      JUnitTestSuiteConstructor.generateTestSuite(AtlasBambooTestSuite.class);
   }
}
