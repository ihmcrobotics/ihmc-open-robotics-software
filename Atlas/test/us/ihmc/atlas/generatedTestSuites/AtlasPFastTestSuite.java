package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.FAST)
@SuiteClasses
({
   us.ihmc.atlas.roughTerrainWalking.AtlasSwingTrajectoryTest.class,
   us.ihmc.atlas.utilities.kinematics.AtlasNumericalInverseKinematicsCalculatorWithRobotTest.class
})

public class AtlasPFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
