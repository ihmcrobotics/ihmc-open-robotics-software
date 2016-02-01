package us.ihmc.commonWalkingControlModules.generatedTestSuites;

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
   us.ihmc.commonWalkingControlModules.calibration.virtualChain.VirtualChainBuilderTest.class,
   us.ihmc.commonWalkingControlModules.calibration.virtualChain.VirtualChainConstructorFromARobotTest.class,
   us.ihmc.commonWalkingControlModules.captureRegion.CaptureRegionCalculatorTest.class,
   us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculatorTest.class,
   us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolverTest.class,
   us.ihmc.commonWalkingControlModules.controlModules.GeometricVirtualToePointCalculatorLogisticParametersTest.class,
   us.ihmc.commonWalkingControlModules.controlModules.GeometricVirtualToePointCalculatorTest.class,
   us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization.ConstrainedQPSolverTest.class,
   us.ihmc.commonWalkingControlModules.controlModules.TeeterTotterLegStrengthCalculatorTest.class,
   us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootPosePacketTransformerTest.class,
   us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.FootstepDataTansformerTest.class,
   us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.HandPosePacketTransformerTest.class,
   us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.PelvisPosePacketTransformerTest.class,
   us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects.TorusPosePacketTransformerTest.class,
   us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorToolsTest.class,
   us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepTest.class,
   us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepPathCoordinatorTest.class,
   us.ihmc.commonWalkingControlModules.desiredFootStep.RangeOfStep2dTest.class,
   us.ihmc.commonWalkingControlModules.desiredFootStep.RangeOfStep3dTest.class,
   us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScriptTest.class,
   us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states.FastCutForceControlHelperTest.class,
   us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.DoubleSupportICPEquationsTest.class,
   us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.NewInstantaneousCapturePointPlannerTest.class,
   us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.NewInstantaneousCapturePointPlannerWithSmootherTest.class,
   us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.SmartCMPProjectorTest.class,
   us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointToolsTest.class,
   us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.NewDoubleSupportICPComputerTest.class,
   us.ihmc.commonWalkingControlModules.kinematics.DampedLeastSquaresJacobianSolverTest.class,
   us.ihmc.commonWalkingControlModules.momentumBasedController.CentroidalMomentumBenchmarkTest.class,
   us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumOptimizerOldTest.class,
   us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumSolverTest.class,
   us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OptimizationMomentumControlModuleTest.class,
   us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.SingularValueExplorationAndExamplesTest.class,
   us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.TypicalMotionConstraintsTest.class,
   us.ihmc.commonWalkingControlModules.terrain.VaryingStairGroundProfileTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.CirclePoseTrajectoryGeneratorTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.CoMHeightTimeDerivativesSmootherTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.Constrained5thOrderPolyForSwingTrajectoryTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.EndPointConstrainedCubicTrajectoryTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.LookAheadCoMHeightTrajectoryGeneratorTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.PushRecoveryTrajectoryGeneratorTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.SmoothenedConstantCoPICPTrajectoryGeneratorTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.SplineBasedCoMHeightTrajectoryGeneratorTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.StraightLinePoseTrajectoryGeneratorTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.StraightUpThenParabolicCartesianTrajectoryGeneratorTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.SwingHeightTrajectoryCalculatorTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.TakeoffLandingCartesianTrajectoryGeneratorTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.ThreePointDoubleSplines1DTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.ThreePointDoubleSplines2DTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointPositionTrajectoryGeneratorTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.VelocityConstrainedPoseTrajectoryGeneratorTest.class,
   us.ihmc.commonWalkingControlModules.trajectories.ZeroToOneParabolicVelocityTrajectoryGeneratorTest.class,
   us.ihmc.commonWalkingControlModules.wrenchDistribution.ContactPointWrenchMatrixCalculatorTest.class,
   us.ihmc.commonWalkingControlModules.wrenchDistribution.CylinderAndPlaneContactForceOptimizerMatrixCalculatorTest.class,
   us.ihmc.commonWalkingControlModules.wrenchDistribution.GroundReactionWrenchDistributorTest.class,
   us.ihmc.commonWalkingControlModules.wrenchDistribution.PlaneContactWrenchMatrixCalculatorTest.class
})

public class CommonWalkingControlModulesAFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
