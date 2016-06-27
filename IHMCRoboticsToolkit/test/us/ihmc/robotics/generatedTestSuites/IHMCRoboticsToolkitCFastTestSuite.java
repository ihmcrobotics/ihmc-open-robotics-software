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
   us.ihmc.robotics.math.trajectories.waypoints.SimpleEuclideanTrajectoryPointTest.class,
   us.ihmc.robotics.math.trajectories.waypoints.SimpleSE3TrajectoryPointTest.class,
   us.ihmc.robotics.math.trajectories.waypoints.SimpleSO3TrajectoryPointTest.class,
   us.ihmc.robotics.math.trajectories.waypoints.YoFrameEuclideanTrajectoryPointTest.class,
   us.ihmc.robotics.math.trajectories.waypoints.YoFrameSE3TrajectoryPointTest.class,
   us.ihmc.robotics.math.trajectories.waypoints.YoFrameSO3TrajectoryPointTest.class,
   us.ihmc.robotics.math.trajectories.WrapperForMultiplePositionTrajectoryGeneratorsTest.class,
   us.ihmc.robotics.math.trajectories.WrapperForPositionAndOrientationTrajectoryGeneratorsTest.class,
   us.ihmc.robotics.math.trajectories.YoConcatenatedSplinesTest.class,
   us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectoryTest.class,
   us.ihmc.robotics.math.trajectories.YoParabolicTrajectoryGeneratorTest.class,
   us.ihmc.robotics.math.trajectories.YoSpline3DTest.class,
   us.ihmc.robotics.math.YoRMSCalculatorTest.class,
   us.ihmc.robotics.math.YoSignalDerivativeTest.class,
   us.ihmc.robotics.MathToolsTest.class,
   us.ihmc.robotics.numericalMethods.DifferentiatorTest.class,
   us.ihmc.robotics.numericalMethods.NewtonRaphsonMethodTest.class,
   us.ihmc.robotics.numericalMethods.QuarticEquationSolverTest.class,
   us.ihmc.robotics.numericalMethods.QuarticRootFinderTest.class,
   us.ihmc.robotics.optimization.ActiveSearchQuadraticProgramOptimizerTest.class,
   us.ihmc.robotics.optimization.EqualityConstraintEnforcerTest.class,
   us.ihmc.robotics.quadTree.QuadTreeForGroundTest.class,
   us.ihmc.robotics.random.RandomToolsTest.class,
   us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrameTest.class,
   us.ihmc.robotics.referenceFrames.Pose2dReferenceFrameTest.class,
   us.ihmc.robotics.referenceFrames.PoseReferenceFrameTest.class,
   us.ihmc.robotics.referenceFrames.ReferenceFrameTest.class,
   us.ihmc.robotics.robotSide.QuadrantDependentListTest.class,
   us.ihmc.robotics.robotSide.RecyclingQuadrantDependentListTest.class,
   us.ihmc.robotics.robotSide.RobotQuadrantTest.class,
   us.ihmc.robotics.robotSide.RobotSideTest.class,
   us.ihmc.robotics.robotSide.SideDependentListTest.class,
   us.ihmc.robotics.screwTheory.CenterOfMassAccelerationCalculatorTest.class,
   us.ihmc.robotics.screwTheory.CenterOfMassJacobianTest.class,
   us.ihmc.robotics.screwTheory.CompositeRigidBodyMassMatrixCalculatorTest.class,
   us.ihmc.robotics.screwTheory.ConvectiveTermCalculatorTest.class,
   us.ihmc.robotics.screwTheory.DesiredJointAccelerationCalculatorTest.class,
   us.ihmc.robotics.screwTheory.DifferentialIDMassMatrixCalculatorTest.class,
   us.ihmc.robotics.screwTheory.FourBarKinematicLoopTest.class,
   us.ihmc.robotics.screwTheory.GenericCRC32Test.class,
   us.ihmc.robotics.screwTheory.GeometricJacobianTest.class,
   us.ihmc.robotics.screwTheory.MomentumCalculatorTest.class,
   us.ihmc.robotics.screwTheory.MomentumTest.class,
   us.ihmc.robotics.screwTheory.PassiveRevoluteJointTest.class,
   us.ihmc.robotics.screwTheory.PointJacobianTest.class,
   us.ihmc.robotics.screwTheory.RigidBodyInertiaTest.class,
   us.ihmc.robotics.screwTheory.ScrewToolsTest.class,
   us.ihmc.robotics.screwTheory.SpatialAccelerationVectorTest.class,
   us.ihmc.robotics.screwTheory.SpatialMotionVectorTest.class,
   us.ihmc.robotics.screwTheory.ThreeDoFAngularAccelerationCalculatorTest.class,
   us.ihmc.robotics.screwTheory.TotalMassCalculatorTest.class,
   us.ihmc.robotics.screwTheory.TwistTest.class,
   us.ihmc.robotics.screwTheory.WrenchTest.class,
   us.ihmc.robotics.stateMachines.StateChangeRecorderTest.class,
   us.ihmc.robotics.stateMachines.StateMachineTest.class,
   us.ihmc.robotics.statistics.CovarianceDerivationTest.class,
   us.ihmc.robotics.statistics.OnePassMeanAndStandardDeviationTest.class,
   us.ihmc.robotics.statistics.PermutationTest.class,
   us.ihmc.robotics.time.CallFrequencyCalculatorTest.class,
   us.ihmc.robotics.time.ExecutionTimerTest.class,
   us.ihmc.robotics.time.GlobalTimerTest.class,
   us.ihmc.robotics.time.TimeToolsTest.class,
   us.ihmc.robotics.trajectories.LinearInterpolatorTest.class,
   us.ihmc.robotics.trajectories.MinimumJerkTrajectoryTest.class,
   us.ihmc.robotics.trajectories.ParametricSplineTrajectorySolverTest.class,
   us.ihmc.robotics.trajectories.PolynomialSplineTest.class,
   us.ihmc.robotics.trajectories.providers.ConstantDoubleProviderTest.class,
   us.ihmc.robotics.trajectories.providers.ConstantPositionProviderTest.class,
   us.ihmc.robotics.trajectories.providers.CurrentPositionProviderTest.class,
   us.ihmc.robotics.trajectories.TrajectoryPointOptimizerTest.class,
   us.ihmc.robotics.trajectories.TrapezoidalVelocityTrajectoryTest.class,
   us.ihmc.robotics.trajectories.WaypointMotionGeneratorTest.class
})

public class IHMCRoboticsToolkitCFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
