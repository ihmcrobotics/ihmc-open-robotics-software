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
   us.ihmc.robotics.math.trajectories.HermiteCurveBasedOrientationTrajectoryGeneratorTest.class,
   us.ihmc.robotics.geometry.HumanEvaluationLine2dTest.class,
   us.ihmc.robotics.hyperCubeTree.HyperCubeNodeTest.class,
   us.ihmc.robotics.hyperCubeTree.HyperCubeTreeTest.class,
   us.ihmc.robotics.math.filters.HysteresisFilteredYoVariableTest.class,
   us.ihmc.robotics.geometry.InertiaToolsTest.class,
   us.ihmc.robotics.geometry.InPlaceConvexHullCalculator2dTest.class,
   us.ihmc.robotics.dataStructures.variable.IntegerYoVariableTest.class,
   us.ihmc.robotics.math.trajectories.IntermediateWaypointVelocityGeneratorTest.class,
   us.ihmc.robotics.geometry.IterativeBestFitPlaneCalculatorTest.class,
   us.ihmc.robotics.geometry.LeastSquaresZPlaneFitterTest.class,
   us.ihmc.robotics.lidar.LidarScanParametersTest.class,
   us.ihmc.robotics.lidar.LidarScanTest.class,
   us.ihmc.robotics.geometry.Line2dTest.class,
   us.ihmc.robotics.linearDynamicSystems.LinearDynamicSystemTest.class,
   us.ihmc.robotics.trajectories.LinearInterpolatorTest.class,
   us.ihmc.robotics.functionApproximation.LinearMappingTest.class,
   us.ihmc.robotics.functionApproximation.LinearRegressionTest.class,
   us.ihmc.robotics.geometry.LineSegment2dTest.class,
   us.ihmc.robotics.geometry.LineSegment3dTest.class,
   us.ihmc.robotics.dataStructures.variable.LongYoVariableTest.class,
   us.ihmc.robotics.linearDynamicSystems.MassSpringDamperTest.class,
   us.ihmc.robotics.MathToolsTest.class,
   us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculatorTest.class,
   us.ihmc.robotics.linearAlgebra.MatrixOfCofactorsCalculatorInefficientTest.class,
   us.ihmc.robotics.linearAlgebra.MatrixStatisticsTest.class,
   us.ihmc.robotics.linearAlgebra.MatrixToolsTest.class,
   us.ihmc.robotics.math.MatrixYoVariableConversionToolsTest.class,
   us.ihmc.robotics.trajectories.MinimumJerkTrajectoryTest.class,
   us.ihmc.robotics.screwTheory.MomentumCalculatorTest.class,
   us.ihmc.robotics.screwTheory.MomentumTest.class,
   us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsOrientationTrajectoryGeneratorTest.class,
   us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsPositionTrajectoryGeneratorTest.class,
   us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGeneratorTest.class,
   us.ihmc.robotics.dataStructures.registry.NameSpaceTest.class,
   us.ihmc.robotics.numericalMethods.NewtonRaphsonMethodTest.class,
   us.ihmc.robotics.math.corruptors.NoisyDoubleYoVariableTest.class,
   us.ihmc.robotics.math.corruptors.NoisyYoRotationMatrixTest.class,
   us.ihmc.robotics.linearAlgebra.NullspaceCalculatorTest.class,
   us.ihmc.robotics.kinematics.NumericalInverseKinematicsCalculatorTest.class,
   us.ihmc.robotics.hyperCubeTree.OctreeTest.class,
   us.ihmc.robotics.hyperCubeTree.OneDimensionalBoundsTest.class,
   us.ihmc.robotics.math.trajectories.OneDoFJointQuinticTrajectoryGeneratorTest.class,
   us.ihmc.robotics.statistics.OnePassMeanAndStandardDeviationTest.class,
   us.ihmc.robotics.functionApproximation.OnlineLinearRegressionTest.class,
   us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculatorTest.class,
   us.ihmc.robotics.math.trajectories.OrientationInterpolationTrajectoryGeneratorTest.class,
   us.ihmc.robotics.trajectories.ParametricSplineTrajectorySolverTest.class,
   us.ihmc.robotics.screwTheory.PassiveRevoluteJointTest.class,
   us.ihmc.robotics.controllers.PDControllerTest.class,
   us.ihmc.robotics.statistics.PermutationTest.class,
   us.ihmc.robotics.controllers.PIDControllerTest.class,
   us.ihmc.robotics.geometry.shapes.Plane3dTest.class,
   us.ihmc.robotics.screwTheory.PointJacobianTest.class,
   us.ihmc.robotics.geometry.PointToLineUnProjectorTest.class,
   us.ihmc.robotics.linearDynamicSystems.PolynomialMatrixTest.class,
   us.ihmc.robotics.trajectories.PolynomialSplineTest.class,
   us.ihmc.robotics.dataStructures.PolynomialTest.class,
   us.ihmc.robotics.referenceFrames.Pose2dReferenceFrameTest.class,
   us.ihmc.robotics.referenceFrames.PoseReferenceFrameTest.class,
   us.ihmc.robotics.math.trajectories.PositionTrajectorySmootherTest.class,
   us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3DTest.class,
   us.ihmc.robotics.math.trajectories.ProviderBasedConstantOrientationTrajectoryGeneratorTest.class,
   us.ihmc.robotics.robotSide.QuadrantDependentListTest.class,
   us.ihmc.robotics.quadTree.QuadTreeForGroundTest.class,
   us.ihmc.robotics.numericalMethods.QuarticEquationSolverTest.class,
   us.ihmc.robotics.numericalMethods.QuarticRootFinderTest.class,
   us.ihmc.robotics.math.QuaternionCalculusTest.class,
   us.ihmc.robotics.geometry.QuaternionRotationRelationshipTest.class,
   us.ihmc.robotics.geometry.shapes.QuickHull3DWrapperTest.class,
   us.ihmc.robotics.math.interpolators.QuinticSplineInterpolatorTest.class,
   us.ihmc.robotics.geometry.shapes.Ramp3dTest.class,
   us.ihmc.robotics.random.RandomToolsTest.class,
   us.ihmc.robotics.math.filters.RateLimitedYoVariableTest.class,
   us.ihmc.robotics.lists.RecyclingArrayListTest.class,
   us.ihmc.robotics.robotSide.RecyclingQuadrantDependentListTest.class,
   us.ihmc.robotics.referenceFrames.ReferenceFrameTest.class,
   us.ihmc.robotics.screwTheory.RigidBodyInertiaTest.class,
   us.ihmc.robotics.geometry.RigidBodyTransformTest.class,
   us.ihmc.robotics.robotSide.RobotQuadrantTest.class,
   us.ihmc.robotics.robotSide.RobotSideTest.class,
   us.ihmc.robotics.geometry.RotationalInertiaCalculatorTest.class,
   us.ihmc.robotics.geometry.RotationToolsTest.class,
   us.ihmc.robotics.screwTheory.ScrewToolsTest.class,
   us.ihmc.robotics.robotSide.SideDependentListTest.class,
   us.ihmc.robotics.math.trajectories.waypoints.SimpleEuclideanTrajectoryPointTest.class,
   us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariableTest.class,
   us.ihmc.robotics.math.trajectories.SimpleOrientationTrajectoryGeneratorTest.class,
   us.ihmc.robotics.math.trajectories.waypoints.SimpleSE3TrajectoryPointTest.class
})

public class IHMCRoboticsToolkitBFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
