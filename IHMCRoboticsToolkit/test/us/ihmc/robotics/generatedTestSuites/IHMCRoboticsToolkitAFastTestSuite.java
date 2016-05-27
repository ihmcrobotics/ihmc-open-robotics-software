package us.ihmc.robotics.generatedTestSuites;

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
   us.ihmc.robotics.lidar.AbstractLidarScanTest.class,
   us.ihmc.robotics.optimization.ActiveSearchQuadraticProgramOptimizerTest.class,
   us.ihmc.robotics.math.filters.AlphaBetaFilteredYoVariableTest.class,
   us.ihmc.robotics.math.filters.AlphaFilteredWrappingYoVariableTest.class,
   us.ihmc.robotics.math.filters.AlphaFilteredYoFrameQuaternionTest.class,
   us.ihmc.robotics.math.filters.AlphaFilteredYoVariableTest.class,
   us.ihmc.robotics.math.filters.AlphaFusedYoVariableTest.class,
   us.ihmc.robotics.geometry.AngleToolsTest.class,
   us.ihmc.robotics.kinematics.AverageQuaternionCalculatorTest.class,
   us.ihmc.robotics.math.filters.BacklashCompensatingVelocityYoVariableTest.class,
   us.ihmc.robotics.math.filters.BacklashProcessingYoVariableTest.class,
   us.ihmc.robotics.geometry.shapes.BestFitPlaneCalculatorTest.class,
   us.ihmc.robotics.math.filters.BetaFilteredYoVariableTest.class,
   us.ihmc.robotics.linearDynamicSystems.BodeUnitsConverterTest.class,
   us.ihmc.robotics.dataStructures.variable.BooleanYoVariableTest.class,
   us.ihmc.robotics.geometry.BoundingBox2dTest.class,
   us.ihmc.robotics.geometry.BoundingBox3dTest.class,
   us.ihmc.robotics.geometry.shapes.Box3dTest.class,
   us.ihmc.robotics.time.CallFrequencyCalculatorTest.class,
   us.ihmc.robotics.hierarchicalKinematics.CapsuleCollisionCheckTest.class,
   us.ihmc.robotics.geometry.CapsuleTest.class,
   us.ihmc.robotics.screwTheory.CenterOfMassAccelerationCalculatorTest.class,
   us.ihmc.robotics.screwTheory.CenterOfMassJacobianTest.class,
   us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrameTest.class,
   us.ihmc.robotics.math.trajectories.CirclePositionTrajectoryGeneratorTest.class,
   us.ihmc.robotics.hierarchicalKinematics.CollisionAvoidanceConstraintTest.class,
   us.ihmc.robotics.linearDynamicSystems.ComplexConjugateModeTest.class,
   us.ihmc.robotics.linearDynamicSystems.ComplexMatrixTest.class,
   us.ihmc.robotics.dataStructures.ComplexNumberTest.class,
   us.ihmc.robotics.screwTheory.CompositeRigidBodyMassMatrixCalculatorTest.class,
   us.ihmc.robotics.math.trajectories.ConstantAccelerationTrajectoryGeneratorTest.class,
   us.ihmc.robotics.trajectories.providers.ConstantDoubleProviderTest.class,
   us.ihmc.robotics.math.trajectories.ConstantForceTrajectoryGeneratorTest.class,
   us.ihmc.robotics.math.trajectories.ConstantOrientationTrajectoryGeneratorTest.class,
   us.ihmc.robotics.math.trajectories.ConstantPoseTrajectoryGeneratorTest.class,
   us.ihmc.robotics.trajectories.providers.ConstantPositionProviderTest.class,
   us.ihmc.robotics.math.trajectories.ConstantPositionTrajectoryGeneratorTest.class,
   us.ihmc.robotics.math.trajectories.ConstantVelocityTrajectoryGeneratorTest.class,
   us.ihmc.robotics.screwTheory.ConvectiveTermCalculatorTest.class,
   us.ihmc.robotics.geometry.ConvexHullCalculator2dTest.class,
   us.ihmc.robotics.geometry.ConvexPolygon2dTest.class,
   us.ihmc.robotics.geometry.ConvexPolygonShrinkerTest.class,
   us.ihmc.robotics.geometry.ConvexPolygonToolsTest.class,
   us.ihmc.robotics.statistics.CovarianceDerivationTest.class,
   us.ihmc.robotics.trajectories.providers.CurrentPositionProviderTest.class,
   us.ihmc.robotics.geometry.shapes.Cylinder3dTest.class,
   us.ihmc.robotics.controllers.CylindricalPDGainsTest.class,
   us.ihmc.robotics.kinematics.DdoglegInverseKinematicsCalculatorTest.class,
   us.ihmc.robotics.math.filters.DeadzoneYoVariableTest.class,
   us.ihmc.robotics.math.filters.DelayedBooleanYoVariableTest.class,
   us.ihmc.robotics.math.filters.DelayedDoubleYoVariableTest.class,
   us.ihmc.robotics.math.filters.DeltaLimitedYoVariableTest.class,
   us.ihmc.robotics.screwTheory.DesiredJointAccelerationCalculatorTest.class,
   us.ihmc.robotics.screwTheory.DifferentialIDMassMatrixCalculatorTest.class,
   us.ihmc.robotics.numericalMethods.DifferentiatorTest.class,
   us.ihmc.robotics.geometry.DirectionTest.class,
   us.ihmc.robotics.dataStructures.DoubleHashHeightMapTest.class,
   us.ihmc.robotics.dataStructures.variable.DoubleYoVariableTest.class,
   us.ihmc.robotics.linearDynamicSystems.EigenvalueDecomposerTest.class,
   us.ihmc.robotics.geometry.shapes.Ellipsoid3dTest.class,
   us.ihmc.robotics.dataStructures.variable.EnumYoVariableTest.class,
   us.ihmc.robotics.optimization.EqualityConstraintEnforcerTest.class,
   us.ihmc.robotics.time.ExecutionTimerTest.class,
   us.ihmc.robotics.math.filters.FilteredDiscreteVelocityYoVariableTest.class,
   us.ihmc.robotics.math.filters.FilteredVelocityYoVariableTest.class,
   us.ihmc.robotics.math.filters.FirstOrderFilteredYoVariableTest.class,
   us.ihmc.robotics.hierarchicalKinematics.ForwardKinematicSolverTest.class,
   us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorsComparisonTest.class,
   us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorTest.class,
   us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorWithDerivativesTest.class,
   us.ihmc.robotics.screwTheory.FourBarKinematicLoopTest.class,
   us.ihmc.robotics.geometry.shapes.FrameBox3dTest.class,
   us.ihmc.robotics.geometry.FrameConvexPolygon2dTest.class,
   us.ihmc.robotics.math.trajectories.waypoints.FrameEuclideanTrajectoryPointTest.class,
   us.ihmc.robotics.geometry.FrameLine2dTest.class,
   us.ihmc.robotics.geometry.FrameLineTest.class,
   us.ihmc.robotics.geometry.FrameMatrix3DTest.class,
   us.ihmc.robotics.geometry.FrameOrientationTest.class,
   us.ihmc.robotics.geometry.shapes.FramePlane3dTest.class,
   us.ihmc.robotics.geometry.FramePoint2dTest.class,
   us.ihmc.robotics.geometry.FramePointTest.class,
   us.ihmc.robotics.geometry.FramePoseTest.class,
   us.ihmc.robotics.math.trajectories.waypoints.FrameSE3TrajectoryPointTest.class,
   us.ihmc.robotics.math.trajectories.waypoints.FrameSO3TrajectoryPointTest.class,
   us.ihmc.robotics.geometry.FrameVector2dTest.class,
   us.ihmc.robotics.geometry.FrameVectorTest.class,
   us.ihmc.robotics.controllers.GainCalculatorTest.class,
   us.ihmc.robotics.screwTheory.GenericCRC32Test.class,
   us.ihmc.robotics.screwTheory.GeometricJacobianTest.class,
   us.ihmc.robotics.geometry.GeometryToolsTest.class,
   us.ihmc.robotics.geometry.GeoregressionConversionToolsTest.class,
   us.ihmc.robotics.math.filters.GlitchFilteredBooleanYoVariableTest.class,
   us.ihmc.robotics.filters.GlitchFilterForDataSetTest.class,
   us.ihmc.robotics.time.GlobalTimerTest.class
})

public class IHMCRoboticsToolkitAFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
