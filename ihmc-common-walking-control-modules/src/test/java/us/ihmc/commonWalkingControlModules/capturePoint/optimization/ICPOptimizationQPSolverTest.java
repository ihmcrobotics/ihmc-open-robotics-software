package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.jcodec.common.Assert;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationQPSolver;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.tools.exceptions.NoConvergenceException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ICPOptimizationQPSolverTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 1e-3;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStandingWithPerfectTrackingAndAngularMomentum() throws Exception
   {
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(0.0, 0.0, 10, false);

      solver.setCMPFeedbackConditions(10.0, true);
      solver.setFeedbackConditions(0.1, 3.0, 500.0);

      FrameVector2D icpError = new FrameVector2D();
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.05, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();

      Assert.assertTrue(copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStandingUnconstrainedWithAndAngularMomentum() throws Exception
   {
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(0.0, 0.0, 10, false);

      solver.setCMPFeedbackConditions(1000.0, true);
      solver.setFeedbackConditions(0.1, 0.1, 3.0, 3.0, 100000.0);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.05, 0.10);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.05, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();

      copFeedbackExpected.set(icpError);
      copFeedbackExpected.scale(3.0);

      Assert.assertTrue(copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStandingUnconstrained() throws Exception
   {
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(0.0, 0.0, 10, false);

      solver.setFeedbackConditions(0.1, 3.0, 100000.0);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.05, 0.10);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.05, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();

      copFeedbackExpected.set(icpError);
      copFeedbackExpected.scale(3.0);

      Assert.assertTrue(copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStandingConstrained() throws Exception
   {
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(0.0, 0.0, 10, false);
      solver.setMaxNumberOfIterations(10);

      // create support polygon constraint
      double sideLength = 0.1;
      FrameConvexPolygon2d supportPolygon = createSupportPolygon(sideLength);

      solver.setFeedbackConditions(0.1, 3.0, 1000.0);
      solver.addSupportPolygon(supportPolygon);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.04);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();

      // unconstrained CMP location
      copFeedbackExpected.set(icpError);
      copFeedbackExpected.scale(3.0);
      copFeedbackExpected.add(perfectCMP);

      // clip to support polygon
      copFeedbackExpected.setX(Math.min(copFeedbackExpected.getX(), sideLength));
      copFeedbackExpected.setY(Math.min(copFeedbackExpected.getY(), sideLength));
      copFeedbackExpected.sub(perfectCMP);

      Assert.assertTrue(copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStandingConstrainedWithAngularMomentum() throws Exception
   {
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(0.0, 0.0, 10, false);
      solver.setMaxNumberOfIterations(10);

      // create support polygon constraint
      double sideLength = 0.1;
      FrameConvexPolygon2d supportPolygon = createSupportPolygon(sideLength);

      solver.setFeedbackConditions(0.1, 3.0, 10000.0);
      solver.setCMPFeedbackConditions(10.0, true);
      solver.addSupportPolygon(supportPolygon);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.04);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();

      // unconstrained CMP location
      copFeedbackExpected.set(icpError);
      copFeedbackExpected.scale(3.0);
      copFeedbackExpected.add(perfectCMP);

      // clip to support polygon
      cmpCoPDifferenceExpected.set(copFeedbackExpected);
      copFeedbackExpected.setX(Math.min(copFeedbackExpected.getX(), sideLength));
      copFeedbackExpected.setY(Math.min(copFeedbackExpected.getY(), sideLength));

      // angular momentum difference
      cmpCoPDifferenceExpected.sub(copFeedbackExpected);

      // find delta
      copFeedbackExpected.sub(perfectCMP);

      Assert.assertTrue(copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testSteppingUnconstrainedFeedbackPreferred() throws Exception
   {
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(0.0, 0.0, 10, false);
      solver.setMaxNumberOfIterations(10);

      double feedbackWeight = 0.1;
      double feedbackGain = 3.0;
      double footstepWeight = 100.0;

      solver.setFeedbackConditions(feedbackWeight, feedbackGain, 10000.0);

      double timeRemainingInState = 1.0;
      double omega = 1.0;
      double footstepMultiplier = Math.exp(-omega * timeRemainingInState);
      FramePoint2D desiredFootstepLocation = new FramePoint2D(worldFrame, 0.3, 0.1);
      solver.setFootstepAdjustmentConditions(footstepMultiplier, footstepWeight, desiredFootstepLocation);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.04);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();
      FramePoint2D footstepLocation = new FramePoint2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);
      solver.getFootstepSolutionLocation(0, footstepLocation);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();
      FramePoint2D footstepLocationExpected = new FramePoint2D();

      copFeedbackExpected.set(icpError);
      copFeedbackExpected.scale(feedbackGain);

      footstepLocationExpected.set(desiredFootstepLocation);

      Assert.assertTrue(copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
      Assert.assertTrue(footstepLocation.epsilonEquals(footstepLocationExpected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testSteppingUnconstrainedFootstepAdjustmentPreferred() throws Exception
   {
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(0.0, 0.0, 10, false);
      solver.setMaxNumberOfIterations(10);

      double feedbackWeight = 1000.0;
      double feedbackGain = 3.0;
      double footstepWeight = 0.1;

      solver.setFeedbackConditions(feedbackWeight, feedbackGain, 10000.0);

      double timeRemainingInState = 1.0;
      double omega = 1.0;
      double footstepMultiplier = Math.exp(-omega * timeRemainingInState);
      FramePoint2D desiredFootstepLocation = new FramePoint2D(worldFrame, 0.3, 0.1);
      solver.setFootstepAdjustmentConditions(footstepMultiplier, footstepWeight, desiredFootstepLocation);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.04);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();
      FramePoint2D footstepLocation = new FramePoint2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);
      solver.getFootstepSolutionLocation(0, footstepLocation);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();
      FramePoint2D footstepLocationExpected = new FramePoint2D();

      footstepLocationExpected.set(desiredFootstepLocation);
      icpError.scale(1.0 / footstepMultiplier);
      footstepLocationExpected.add(icpError);

      Assert.assertTrue(copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
      Assert.assertTrue(footstepLocation.epsilonEquals(footstepLocationExpected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testSteppingUnconstrainedFootstepAdjustmentPreferredWithAngularMomentum() throws Exception
   {
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(0.0, 0.0, 10, false);
      solver.setMaxNumberOfIterations(10);

      double feedbackWeight = 1000.0;
      double feedbackGain = 3.0;
      double footstepWeight = 0.1;

      solver.setFeedbackConditions(feedbackWeight, feedbackGain, 10000.0);
      solver.setCMPFeedbackConditions(10.0 * feedbackWeight, true);

      double timeRemainingInState = 1.0;
      double omega = 1.0;
      double footstepMultiplier = Math.exp(-omega * timeRemainingInState);
      FramePoint2D desiredFootstepLocation = new FramePoint2D(worldFrame, 0.3, 0.1);
      solver.setFootstepAdjustmentConditions(footstepMultiplier, footstepWeight, desiredFootstepLocation);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.04);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();
      FramePoint2D footstepLocation = new FramePoint2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);
      solver.getFootstepSolutionLocation(0, footstepLocation);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();
      FramePoint2D footstepLocationExpected = new FramePoint2D();

      footstepLocationExpected.set(desiredFootstepLocation);
      icpError.scale(1.0 / footstepMultiplier);
      footstepLocationExpected.add(icpError);

      Assert.assertTrue(copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
      Assert.assertTrue(footstepLocation.epsilonEquals(footstepLocationExpected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testSteppingUnconstrainedWithAdjustment() throws Exception
   {
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(0.0, 0.0, 10, false);
      solver.setMaxNumberOfIterations(10);

      double feedbackWeight = 1.0;
      double feedbackGain = 3.0;
      double footstepWeight = 1.0;

      solver.setFeedbackConditions(feedbackWeight, feedbackGain, 10000.0);

      double timeRemainingInState = 1.0;
      double omega = 1.0;
      double footstepMultiplier = Math.exp(-omega * timeRemainingInState);
      FramePoint2D desiredFootstepLocation = new FramePoint2D(worldFrame, 0.3, 0.1);
      solver.setFootstepAdjustmentConditions(footstepMultiplier, footstepWeight, desiredFootstepLocation);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.06, 0.10);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();
      FramePoint2D footstepLocation = new FramePoint2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);
      solver.getFootstepSolutionLocation(0, footstepLocation);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();
      FramePoint2D footstepLocationExpected = new FramePoint2D();

      LinearSolver<DenseMatrix64F> simpleSolver = LinearSolverFactory.linear(6);
      DenseMatrix64F quadratic = new DenseMatrix64F(6, 6);
      DenseMatrix64F equals = new DenseMatrix64F(6, 1);
      DenseMatrix64F solution = new DenseMatrix64F(6, 1);
      quadratic.set(0, 0, feedbackWeight);
      quadratic.set(1, 1, feedbackWeight);
      quadratic.set(0, 4, 1.0);
      quadratic.set(1, 5, 1.0);
      quadratic.set(2, 2, footstepWeight);
      quadratic.set(3, 3, footstepWeight);
      quadratic.set(2, 4, feedbackGain * footstepMultiplier);
      quadratic.set(3, 5, feedbackGain * footstepMultiplier);
      quadratic.set(4, 0, 1.0);
      quadratic.set(5, 1, 1.0);
      quadratic.set(4, 2, feedbackGain * footstepMultiplier);
      quadratic.set(5, 3, feedbackGain * footstepMultiplier);

      equals.set(4, 0, feedbackGain * icpError.getX());
      equals.set(5, 0, feedbackGain * icpError.getY());

      simpleSolver.setA(quadratic);
      simpleSolver.solve(equals, solution);

      copFeedbackExpected.setX(solution.get(0));
      copFeedbackExpected.setY(solution.get(1));

      footstepLocationExpected.setX(desiredFootstepLocation.getX() + solution.get(2));
      footstepLocationExpected.setY(desiredFootstepLocation.getY() + solution.get(3));

      Assert.assertTrue(copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
      Assert.assertTrue(footstepLocation.epsilonEquals(footstepLocationExpected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testSteppingUnconstrainedWithAdjustmentAndAngularMomentum() throws Exception
   {
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(0.0, 0.0, 10, false);
      solver.setMaxNumberOfIterations(10);

      double feedbackWeight = 1.0;
      double feedbackGain = 3.0;
      double footstepWeight = 0.1;

      solver.setFeedbackConditions(feedbackWeight, feedbackGain, 10000.0);
      solver.setCMPFeedbackConditions(100.0, true);

      double timeRemainingInState = 1.0;
      double omega = 1.0;
      double footstepMultiplier = Math.exp(-omega * timeRemainingInState);
      FramePoint2D desiredFootstepLocation = new FramePoint2D(worldFrame, 0.3, 0.1);
      solver.setFootstepAdjustmentConditions(footstepMultiplier, footstepWeight, desiredFootstepLocation);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.06, 0.10);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();
      FramePoint2D footstepLocation = new FramePoint2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);
      solver.getFootstepSolutionLocation(0, footstepLocation);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();
      FramePoint2D footstepLocationExpected = new FramePoint2D();

      LinearSolver<DenseMatrix64F> simpleSolver = LinearSolverFactory.linear(6);
      DenseMatrix64F quadratic = new DenseMatrix64F(6, 6);
      DenseMatrix64F equals = new DenseMatrix64F(6, 1);
      DenseMatrix64F solution = new DenseMatrix64F(6, 1);
      quadratic.set(0, 0, feedbackWeight);
      quadratic.set(1, 1, feedbackWeight);
      quadratic.set(0, 4, 1.0);
      quadratic.set(1, 5, 1.0);
      quadratic.set(2, 2, footstepWeight);
      quadratic.set(3, 3, footstepWeight);
      quadratic.set(2, 4, feedbackGain * footstepMultiplier);
      quadratic.set(3, 5, feedbackGain * footstepMultiplier);
      quadratic.set(4, 0, 1.0);
      quadratic.set(5, 1, 1.0);
      quadratic.set(4, 2, feedbackGain * footstepMultiplier);
      quadratic.set(5, 3, feedbackGain * footstepMultiplier);

      equals.set(4, 0, feedbackGain * icpError.getX());
      equals.set(5, 0, feedbackGain * icpError.getY());

      simpleSolver.setA(quadratic);
      simpleSolver.solve(equals, solution);

      copFeedbackExpected.setX(solution.get(0));
      copFeedbackExpected.setY(solution.get(1));

      footstepLocationExpected.setX(desiredFootstepLocation.getX() + solution.get(2));
      footstepLocationExpected.setY(desiredFootstepLocation.getY() + solution.get(3));

      Assert.assertTrue(copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
      Assert.assertTrue(footstepLocation.epsilonEquals(footstepLocationExpected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testSteppingCoPConstrainedWithAdjustment() throws Exception
   {
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(0.0, 0.0, 10, false);
      solver.setMaxNumberOfIterations(10);

      double feedbackWeight = 0.1;
      double feedbackGain = 3.0;
      double footstepWeight = 50.0;

      solver.setFeedbackConditions(feedbackWeight, feedbackGain, 100000.0);

      double sideLength = 0.1;
      FrameConvexPolygon2d supportPolygon = createSupportPolygon(sideLength);
      solver.addSupportPolygon(supportPolygon);

      double timeRemainingInState = 1.0;
      double omega = 1.0;
      double footstepMultiplier = Math.exp(-omega * timeRemainingInState);
      FramePoint2D desiredFootstepLocation = new FramePoint2D(worldFrame, 0.3, 0.1);
      solver.setFootstepAdjustmentConditions(footstepMultiplier, footstepWeight, desiredFootstepLocation);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.04, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.02, 0.04);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();
      FramePoint2D footstepLocation = new FramePoint2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);
      solver.getFootstepSolutionLocation(0, footstepLocation);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();
      FramePoint2D footstepLocationExpected = new FramePoint2D();

      copFeedbackExpected.set(0.08, 0.06);

      footstepLocationExpected.set(icpError);
      footstepLocationExpected.scale(feedbackGain);
      footstepLocationExpected.sub(copFeedbackExpected);
      footstepLocationExpected.scale(1.0 / (feedbackGain * footstepMultiplier));

      footstepLocationExpected.add(desiredFootstepLocation);

      Assert.assertTrue(copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
      Assert.assertTrue(footstepLocation.epsilonEquals(footstepLocationExpected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testCoPAndFootstepConstrainedWithAdjustment() throws Exception
   {
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(0.0, 0.0, 10, false);
      solver.setMaxNumberOfIterations(10);

      double feedbackWeight = 0.1;
      double feedbackGain = 3.0;
      double footstepWeight = 50.0;

      solver.setFeedbackConditions(feedbackWeight, feedbackGain, 100000.0);

      double sideLength = 0.1;
      FrameConvexPolygon2d supportPolygon = createSupportPolygon(sideLength);
      solver.addSupportPolygon(supportPolygon);

      FrameConvexPolygon2d reachabilityPolygon = createReachabilityPolygon();
      solver.addReachabilityPolygon(reachabilityPolygon);

      double timeRemainingInState = 1.0;
      double omega = 1.0;
      double footstepMultiplier = Math.exp(-omega * timeRemainingInState);
      FramePoint2D desiredFootstepLocation = new FramePoint2D(worldFrame, 0.3, 0.1);
      solver.setFootstepAdjustmentConditions(footstepMultiplier, footstepWeight, desiredFootstepLocation);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.04, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.02, 0.04);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();
      FramePoint2D footstepLocation = new FramePoint2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);
      solver.getFootstepSolutionLocation(0, footstepLocation);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();
      FramePoint2D footstepLocationExpected = new FramePoint2D();

      copFeedbackExpected.set(0.08, 0.06);

      footstepLocationExpected.set(icpError);
      footstepLocationExpected.scale(feedbackGain);
      footstepLocationExpected.sub(copFeedbackExpected);
      footstepLocationExpected.scale(1.0 / (feedbackGain * footstepMultiplier));

      footstepLocationExpected.add(desiredFootstepLocation);

      // project the footstep into the reachability polygon
      reachabilityPolygon.orthogonalProjection(footstepLocationExpected);

      Assert.assertTrue(copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
      Assert.assertTrue(footstepLocation.epsilonEquals(footstepLocationExpected, epsilon));
   }

   private FrameConvexPolygon2d createSupportPolygon(double sideLength)
   {
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d();
      FramePoint2D backLeft = new FramePoint2D(worldFrame, -sideLength, sideLength);
      FramePoint2D frontLeft = new FramePoint2D(worldFrame, sideLength, sideLength);
      FramePoint2D frontRight = new FramePoint2D(worldFrame, sideLength, -sideLength);
      FramePoint2D backRight = new FramePoint2D(worldFrame, -sideLength, -sideLength);

      supportPolygon.addVertex(backLeft);
      supportPolygon.addVertex(frontLeft);
      supportPolygon.addVertex(frontRight);
      supportPolygon.addVertex(backRight);

      supportPolygon.update();

      return supportPolygon;
   }

   private FrameConvexPolygon2d createReachabilityPolygon()
   {
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d();
      FramePoint2D backLeft = new FramePoint2D(worldFrame, 0.35, 0.08);
      FramePoint2D frontLeft = new FramePoint2D(worldFrame, 0.35, 0.13);
      FramePoint2D frontRight = new FramePoint2D(worldFrame, 0.05, 0.13);
      FramePoint2D backRight = new FramePoint2D(worldFrame, 0.05, 0.08);

      supportPolygon.addVertex(backLeft);
      supportPolygon.addVertex(frontLeft);
      supportPolygon.addVertex(frontRight);
      supportPolygon.addVertex(backRight);

      supportPolygon.update();

      return supportPolygon;
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testNoExceptions() throws Exception
   {
         ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(0.0, 0.0, 10, false);

         // feedback
         FramePoint2D scaledFeedbackWeight = new FramePoint2D(worldFrame, 0.2826586940121205, 0.2826586940121205);
         double feedbackGainX = 3.61466302555;
         double feedbackGainY = 3.88533697445;
         solver.resetCoPFeedbackConditions();
         solver.setFeedbackConditions(scaledFeedbackWeight.getX(), scaledFeedbackWeight.getY(), feedbackGainX, feedbackGainY, 10000.0);
         solver.setMaxCMPDistanceFromEdge(0.06);
         solver.setCopSafeDistanceToEdge(0.002);
         solver.setFeedbackRateWeight(0.0025);

         // angular momentum
         solver.resetCMPFeedbackConditions();
         solver.setCMPFeedbackConditions(10.769105592072197, true);

         // footstep
         FramePoint2D footstepLocation = new FramePoint2D(worldFrame, 0.29744601815922606, -0.6204387201028974);
         solver.resetFootstepConditions();
         solver.setFootstepAdjustmentConditions(0.27117253504559974, 9.999820184919336, 9.990452153569914, 1.0, footstepLocation);
         solver.setFootstepRateWeight(0.0);

         // set previous solutions
         FramePoint2D previousFootstepSolution = new FramePoint2D(worldFrame, 0.2881204908019306, -0.6381315022331429);
         FramePoint2D previousFeedbackDeltaSolution = new FramePoint2D(worldFrame, -0.141764770527381, -0.04745626887921585);
         solver.resetFootstepRate(previousFootstepSolution);
         solver.resetFeedbackRate(previousFeedbackDeltaSolution);

         // set constraints
         FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame);
         supportPolygon.addVertex(new FramePoint2D(worldFrame, -0.12497345851902948, 0.22376754760881368));
         supportPolygon.addVertex(new FramePoint2D(worldFrame, 0.10434696885177858, 0.2110446302571209));
         supportPolygon.addVertex(new FramePoint2D(worldFrame, 0.10438193051572264, 0.12238795176076478));
         supportPolygon.addVertex(new FramePoint2D(worldFrame, -0.12418538788019835, 0.10913840779165224));
         supportPolygon.update();

         FrameConvexPolygon2d reachabilityPolygon = new FrameConvexPolygon2d(worldFrame);
         reachabilityPolygon.addVertex(new FramePoint2D(worldFrame, -0.5065556308152076, -0.0163509564360857));
         reachabilityPolygon.addVertex(new FramePoint2D(worldFrame, 0.893444112012186, -0.015502379303478447));
         reachabilityPolygon.addVertex(new FramePoint2D(worldFrame, 0.8938502168058939, -0.6855022562280167));
         reachabilityPolygon.addVertex(new FramePoint2D(worldFrame, -0.5061495260214995, -0.6863508334088992));
         reachabilityPolygon.update();

         solver.addSupportPolygon(supportPolygon);
         solver.addReachabilityPolygon(reachabilityPolygon);

         FrameVector2D icpError = new FrameVector2D(worldFrame, -0.07380072407166109, -0.05497512056196603);
         FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.020230791294742534, 0.1586256502408977);

         NoConvergenceException exception = null;
         try
         {
            solver.compute(icpError, perfectCMP);
         }
         catch (NoConvergenceException e)
         {
            exception = e;
         }

         Assert.assertTrue(exception == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testSimpleNoExceptions() throws Exception
   {
      ICPOptimizationQPSolver solver = new ICPOptimizationQPSolver(0.0, 0.0, 10, false);


      // feedback
      FramePoint2D scaledFeedbackWeight = new FramePoint2D(worldFrame, 0.283, 0.283);
      double feedbackGainX = 3.5;
      double feedbackGainY = 4.0;
      solver.resetCoPFeedbackConditions();
      solver.setFeedbackConditions(scaledFeedbackWeight.getX(), scaledFeedbackWeight.getY(), feedbackGainX, feedbackGainY, 100000.0);
      solver.setMaxCMPDistanceFromEdge(0.06);
      solver.setCopSafeDistanceToEdge(0.002);
      solver.setFeedbackRateWeight(0.0025);

      // angular momentum
      solver.resetCMPFeedbackConditions();
      solver.setCMPFeedbackConditions(10.77, true);

      // footstep
      FramePoint2D footstepLocation = new FramePoint2D(worldFrame, 0.297, -0.620);
      solver.resetFootstepConditions();
      solver.setFootstepAdjustmentConditions(0.271, 10.0, 10.0, 1.0, footstepLocation);
      solver.setFootstepRateWeight(0.0);

      // set previous solutions
      FramePoint2D previousFootstepSolution = new FramePoint2D(worldFrame, 0.288, -0.638);
      FramePoint2D previousFeedbackDeltaSolution = new FramePoint2D(worldFrame, -0.142, -0.047);
      solver.resetFootstepRate(previousFootstepSolution);
      solver.resetFeedbackRate(previousFeedbackDeltaSolution);

      // set constraints
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d(worldFrame);
      supportPolygon.addVertex(new FramePoint2D(worldFrame, -0.125, 0.224));
      supportPolygon.addVertex(new FramePoint2D(worldFrame, 0.104, 0.211));
      supportPolygon.addVertex(new FramePoint2D(worldFrame, 0.104, 0.122));
      supportPolygon.addVertex(new FramePoint2D(worldFrame, -0.124, 0.109));
      supportPolygon.update();

      FrameConvexPolygon2d reachabilityPolygon = new FrameConvexPolygon2d(worldFrame);
      reachabilityPolygon.addVertex(new FramePoint2D(worldFrame, -0.507, -0.016));
      reachabilityPolygon.addVertex(new FramePoint2D(worldFrame, 0.893, -0.016));
      reachabilityPolygon.addVertex(new FramePoint2D(worldFrame, 0.894, -0.686));
      reachabilityPolygon.addVertex(new FramePoint2D(worldFrame, -0.506, -0.686));
      reachabilityPolygon.update();

      solver.addSupportPolygon(supportPolygon);
      solver.addReachabilityPolygon(reachabilityPolygon);

      FrameVector2D icpError = new FrameVector2D(worldFrame, -0.074, -0.055);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.020, 0.159);

      NoConvergenceException exception = null;
      try
      {
         solver.compute(icpError, perfectCMP);
      }
      catch (NoConvergenceException e)
      {
         exception = e;
      }

      Assert.assertTrue(exception == null);
   }
}

