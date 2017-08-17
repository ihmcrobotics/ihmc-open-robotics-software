package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.jcodec.common.Assert;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

public class SimpleICPOptimizationQPSolverTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 1e-3;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStandingWithPerfectTrackingAndAngularMomentum() throws Exception
   {
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(0.0, 0.0, 10, false);

      solver.setAngularMomentumConditions(10.0, true);
      solver.setFeedbackConditions(0.1, 3.0, 500.0);

      FrameVector2D icpError = new FrameVector2D();
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.05, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
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
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(0.0, 0.0, 10, false);

      solver.setAngularMomentumConditions(1000.0, true);
      solver.setFeedbackConditions(0.1, 0.1, 3.0, 3.0, 100000.0);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.05, 0.10);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.05, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
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
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(0.0, 0.0, 10, false);

      solver.setFeedbackConditions(0.1, 3.0, 100000.0);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.05, 0.10);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.05, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
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
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(0.0, 0.0, 10, false);
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

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
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
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(0.0, 0.0, 10, false);
      solver.setMaxNumberOfIterations(10);

      // create support polygon constraint
      double sideLength = 0.1;
      FrameConvexPolygon2d supportPolygon = createSupportPolygon(sideLength);

      solver.setFeedbackConditions(0.1, 3.0, 10000.0);
      solver.setAngularMomentumConditions(10.0, true);
      solver.addSupportPolygon(supportPolygon);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.04);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
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
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(0.0, 0.0, 10, false);
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

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
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
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(0.0, 0.0, 10, false);
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

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
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
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(0.0, 0.0, 10, false);
      solver.setMaxNumberOfIterations(10);

      double feedbackWeight = 1000.0;
      double feedbackGain = 3.0;
      double footstepWeight = 0.1;

      solver.setFeedbackConditions(feedbackWeight, feedbackGain, 10000.0);
      solver.setAngularMomentumConditions(10.0 * feedbackWeight, true);

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

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
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
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(0.0, 0.0, 10, false);
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

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
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
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(0.0, 0.0, 10, false);
      solver.setMaxNumberOfIterations(10);

      double feedbackWeight = 1.0;
      double feedbackGain = 3.0;
      double footstepWeight = 0.1;

      solver.setFeedbackConditions(feedbackWeight, feedbackGain, 10000.0);
      solver.setAngularMomentumConditions(100.0, true);

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

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
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
   public void testSteppingConstrainedWithAdjustment() throws Exception
   {
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(0.0, 0.0, 10, false);
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

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
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
}

