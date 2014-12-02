package us.ihmc.atlas;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;

import us.ihmc.atlas.controllers.AtlasFootstepGeneratorTest;
import us.ihmc.atlas.momentumBasedControl.AtlasOptimizationMomentumControlModuleTest;
import us.ihmc.atlas.networkProcessor.downlinkSerializers.AtlasJointConfigurationDataSerializerTest;
import us.ihmc.atlas.networking.AtlasNetworkingCommandReaderTest;
import us.ihmc.atlas.posePlayback.AtlasPlaybackPoseSequenceTest;
import us.ihmc.atlas.utilities.kinematics.AtlasNumericalInverseKinematicsCalculatorWithRobotTest;
import us.ihmc.utilities.test.JUnitTestSuiteConstructor;

@RunWith(Suite.class)
@Suite.SuiteClasses(
{
   AtlasFootstepGeneratorTest.class,
   AtlasOptimizationMomentumControlModuleTest.class,
   AtlasNetworkingCommandReaderTest.class,
   AtlasNumericalInverseKinematicsCalculatorWithRobotTest.class,
   AtlasMultiContactTest.class,
   AtlasFlatGroundWalkingTest.class,
   AtlasJointConfigurationDataSerializerTest.class,
   AtlasPlaybackPoseSequenceTest.class,
   us.ihmc.atlas.networkProcessor.depthData.AtlasDepthDataProcessorTest.class,
   
})
public class AtlasBambooTestSuite
{
   public static void main(String[] args)
   {
      String packageName = "us.ihmc.atlas";
      System.out.println(JUnitTestSuiteConstructor.createTestSuite("AtlasBambooTestSuite", packageName));
   }
}
