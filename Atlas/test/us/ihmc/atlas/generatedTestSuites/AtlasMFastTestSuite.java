package us.ihmc.atlas.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.testing.TestPlanSuite;
import us.ihmc.tools.testing.TestPlanSuite.TestSuiteTarget;
import us.ihmc.tools.testing.TestPlanTarget;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(TestPlanSuite.class)
@TestSuiteTarget(TestPlanTarget.Fast)
@SuiteClasses
({
   us.ihmc.atlas.calib.KinematicCalibrationHeadLoopResidualTest.class,
   us.ihmc.atlas.AtlasMultiContactTest.class,
   us.ihmc.atlas.momentumBasedControl.AtlasOptimizationMomentumControlModuleTest.class,
   us.ihmc.atlas.utilities.kinematics.AtlasNumericalInverseKinematicsCalculatorWithRobotTest.class,
   us.ihmc.atlas.commonWalkingControlModules.sensors.AtlasProvidedMassMatrixToolRigidBodyTest.class
})

public class AtlasMFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
