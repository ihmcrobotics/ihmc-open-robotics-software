package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.jcodec.common.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController.SimpleICPOptimizationQPSolver;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

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

      FrameVector2d icpError = new FrameVector2d();
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.05, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2d cmpCoPDifference = new FrameVector2d();
      FrameVector2d copFeedback = new FrameVector2d();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2d cmpCoPDifferenceExpected = new FrameVector2d();
      FrameVector2d copFeedbackExpected = new FrameVector2d();

      Assert.assertTrue(copFeedback.eplilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.eplilonEquals(cmpCoPDifferenceExpected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStandingUnconstrainedWithAndAngularMomentum() throws Exception
   {
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(0.0, 0.0, 10, false);

      solver.setAngularMomentumConditions(1000.0, true);
      solver.setFeedbackConditions(0.1, 0.1, 3.0, 3.0, 100000.0);

      FrameVector2d icpError = new FrameVector2d(worldFrame, 0.05, 0.10);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.05, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2d cmpCoPDifference = new FrameVector2d();
      FrameVector2d copFeedback = new FrameVector2d();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2d cmpCoPDifferenceExpected = new FrameVector2d();
      FrameVector2d copFeedbackExpected = new FrameVector2d();

      copFeedbackExpected.set(icpError);
      copFeedbackExpected.scale(3.0);

      Assert.assertTrue(copFeedback.eplilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.eplilonEquals(cmpCoPDifferenceExpected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStandingUnconstrained() throws Exception
   {
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(0.0, 0.0, 10, false);

      solver.setFeedbackConditions(0.1, 3.0, 100000.0);

      FrameVector2d icpError = new FrameVector2d(worldFrame, 0.05, 0.10);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.05, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2d cmpCoPDifference = new FrameVector2d();
      FrameVector2d copFeedback = new FrameVector2d();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2d cmpCoPDifferenceExpected = new FrameVector2d();
      FrameVector2d copFeedbackExpected = new FrameVector2d();

      copFeedbackExpected.set(icpError);
      copFeedbackExpected.scale(3.0);

      Assert.assertTrue(copFeedback.eplilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.eplilonEquals(cmpCoPDifferenceExpected, epsilon));
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

      FrameVector2d icpError = new FrameVector2d(worldFrame, 0.03, 0.04);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2d cmpCoPDifference = new FrameVector2d();
      FrameVector2d copFeedback = new FrameVector2d();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2d cmpCoPDifferenceExpected = new FrameVector2d();
      FrameVector2d copFeedbackExpected = new FrameVector2d();

      // unconstrained CMP location
      copFeedbackExpected.set(icpError);
      copFeedbackExpected.scale(3.0);
      copFeedbackExpected.add(perfectCMP);

      // clip to support polygon
      copFeedbackExpected.setX(Math.min(copFeedbackExpected.getX(), sideLength));
      copFeedbackExpected.setY(Math.min(copFeedbackExpected.getY(), sideLength));
      copFeedbackExpected.sub(perfectCMP);

      Assert.assertTrue(copFeedback.eplilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.eplilonEquals(cmpCoPDifferenceExpected, epsilon));
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

      FrameVector2d icpError = new FrameVector2d(worldFrame, 0.03, 0.04);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2d cmpCoPDifference = new FrameVector2d();
      FrameVector2d copFeedback = new FrameVector2d();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2d cmpCoPDifferenceExpected = new FrameVector2d();
      FrameVector2d copFeedbackExpected = new FrameVector2d();

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

      Assert.assertTrue(copFeedback.eplilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.eplilonEquals(cmpCoPDifferenceExpected, epsilon));
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
      FramePoint2d desiredFootstepLocation = new FramePoint2d(worldFrame, 0.3, 0.1);
      solver.setFootstepAdjustmentConditions(footstepMultiplier, footstepWeight, desiredFootstepLocation);

      FrameVector2d icpError = new FrameVector2d(worldFrame, 0.03, 0.04);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2d cmpCoPDifference = new FrameVector2d();
      FrameVector2d copFeedback = new FrameVector2d();
      FramePoint2d footstepLocation = new FramePoint2d();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);
      solver.getFootstepSolutionLocation(0, footstepLocation);

      FrameVector2d cmpCoPDifferenceExpected = new FrameVector2d();
      FrameVector2d copFeedbackExpected = new FrameVector2d();
      FramePoint2d footstepLocationExpected = new FramePoint2d();

      copFeedbackExpected.set(icpError);
      copFeedbackExpected.scale(feedbackGain);

      footstepLocationExpected.set(desiredFootstepLocation);

      Assert.assertTrue(copFeedback.eplilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.eplilonEquals(cmpCoPDifferenceExpected, epsilon));
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
      FramePoint2d desiredFootstepLocation = new FramePoint2d(worldFrame, 0.3, 0.1);
      solver.setFootstepAdjustmentConditions(footstepMultiplier, footstepWeight, desiredFootstepLocation);

      FrameVector2d icpError = new FrameVector2d(worldFrame, 0.03, 0.04);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2d cmpCoPDifference = new FrameVector2d();
      FrameVector2d copFeedback = new FrameVector2d();
      FramePoint2d footstepLocation = new FramePoint2d();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);
      solver.getFootstepSolutionLocation(0, footstepLocation);

      FrameVector2d cmpCoPDifferenceExpected = new FrameVector2d();
      FrameVector2d copFeedbackExpected = new FrameVector2d();
      FramePoint2d footstepLocationExpected = new FramePoint2d();

      footstepLocationExpected.set(desiredFootstepLocation);
      icpError.scale(1.0 / footstepMultiplier);
      footstepLocationExpected.add(icpError);

      Assert.assertTrue(copFeedback.eplilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.eplilonEquals(cmpCoPDifferenceExpected, epsilon));
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
      FramePoint2d desiredFootstepLocation = new FramePoint2d(worldFrame, 0.3, 0.1);
      solver.setFootstepAdjustmentConditions(footstepMultiplier, footstepWeight, desiredFootstepLocation);

      FrameVector2d icpError = new FrameVector2d(worldFrame, 0.03, 0.04);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2d cmpCoPDifference = new FrameVector2d();
      FrameVector2d copFeedback = new FrameVector2d();
      FramePoint2d footstepLocation = new FramePoint2d();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);
      solver.getFootstepSolutionLocation(0, footstepLocation);

      FrameVector2d cmpCoPDifferenceExpected = new FrameVector2d();
      FrameVector2d copFeedbackExpected = new FrameVector2d();
      FramePoint2d footstepLocationExpected = new FramePoint2d();

      footstepLocationExpected.set(desiredFootstepLocation);
      icpError.scale(1.0 / footstepMultiplier);
      footstepLocationExpected.add(icpError);

      Assert.assertTrue(copFeedback.eplilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.eplilonEquals(cmpCoPDifferenceExpected, epsilon));
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
      FramePoint2d desiredFootstepLocation = new FramePoint2d(worldFrame, 0.3, 0.1);
      solver.setFootstepAdjustmentConditions(footstepMultiplier, footstepWeight, desiredFootstepLocation);

      FrameVector2d icpError = new FrameVector2d(worldFrame, 0.06, 0.10);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2d cmpCoPDifference = new FrameVector2d();
      FrameVector2d copFeedback = new FrameVector2d();
      FramePoint2d footstepLocation = new FramePoint2d();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);
      solver.getFootstepSolutionLocation(0, footstepLocation);

      FrameVector2d cmpCoPDifferenceExpected = new FrameVector2d();
      FrameVector2d copFeedbackExpected = new FrameVector2d();
      FramePoint2d footstepLocationExpected = new FramePoint2d();

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


      Assert.assertTrue(copFeedback.eplilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.eplilonEquals(cmpCoPDifferenceExpected, epsilon));
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
      FramePoint2d desiredFootstepLocation = new FramePoint2d(worldFrame, 0.3, 0.1);
      solver.setFootstepAdjustmentConditions(footstepMultiplier, footstepWeight, desiredFootstepLocation);

      FrameVector2d icpError = new FrameVector2d(worldFrame, 0.06, 0.10);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.02, 0.01);

      solver.compute(icpError, perfectCMP);

      FrameVector2d cmpCoPDifference = new FrameVector2d();
      FrameVector2d copFeedback = new FrameVector2d();
      FramePoint2d footstepLocation = new FramePoint2d();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);
      solver.getFootstepSolutionLocation(0, footstepLocation);

      FrameVector2d cmpCoPDifferenceExpected = new FrameVector2d();
      FrameVector2d copFeedbackExpected = new FrameVector2d();
      FramePoint2d footstepLocationExpected = new FramePoint2d();

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

      Assert.assertTrue(copFeedback.eplilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.eplilonEquals(cmpCoPDifferenceExpected, epsilon));
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
      FramePoint2d desiredFootstepLocation = new FramePoint2d(worldFrame, 0.3, 0.1);
      solver.setFootstepAdjustmentConditions(footstepMultiplier, footstepWeight, desiredFootstepLocation);

      FrameVector2d icpError = new FrameVector2d(worldFrame, 0.04, 0.06);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.02, 0.04);

      solver.compute(icpError, perfectCMP);

      FrameVector2d cmpCoPDifference = new FrameVector2d();
      FrameVector2d copFeedback = new FrameVector2d();
      FramePoint2d footstepLocation = new FramePoint2d();

      solver.getCMPDifferenceFromCoP(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);
      solver.getFootstepSolutionLocation(0, footstepLocation);

      FrameVector2d cmpCoPDifferenceExpected = new FrameVector2d();
      FrameVector2d copFeedbackExpected = new FrameVector2d();
      FramePoint2d footstepLocationExpected = new FramePoint2d();

      copFeedbackExpected.set(0.08, 0.06);

      footstepLocationExpected.set(icpError);
      footstepLocationExpected.scale(feedbackGain);
      footstepLocationExpected.sub(copFeedbackExpected);
      footstepLocationExpected.scale(1.0 / (feedbackGain * footstepMultiplier));

      footstepLocationExpected.add(desiredFootstepLocation);

      Assert.assertTrue(copFeedback.eplilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue(cmpCoPDifference.eplilonEquals(cmpCoPDifferenceExpected, epsilon));
      Assert.assertTrue(footstepLocation.epsilonEquals(footstepLocationExpected, epsilon));
   }

   private FrameConvexPolygon2d createSupportPolygon(double sideLength)
   {
      FrameConvexPolygon2d supportPolygon = new FrameConvexPolygon2d();
      FramePoint2d backLeft = new FramePoint2d(worldFrame, -sideLength, sideLength);
      FramePoint2d frontLeft = new FramePoint2d(worldFrame, sideLength, sideLength);
      FramePoint2d frontRight = new FramePoint2d(worldFrame, sideLength, -sideLength);
      FramePoint2d backRight = new FramePoint2d(worldFrame, -sideLength, -sideLength);

      supportPolygon.addVertex(backLeft);
      supportPolygon.addVertex(frontLeft);
      supportPolygon.addVertex(frontRight);
      supportPolygon.addVertex(backRight);

      supportPolygon.update();

      return supportPolygon;
   }


   private class TestICPOptimizationParameters extends ICPOptimizationParameters
   {
      @Override
      public int numberOfFootstepsToConsider()
      {
         return 1;
      }

      @Override
      public double getForwardFootstepWeight()
      {
         return 10.0;
      }

      @Override
      public double getLateralFootstepWeight()
      {
         return 10.0;
      }

      @Override
      public double getFootstepRegularizationWeight()
      {
         return 0.0001;
      }

      @Override
      public double getFeedbackForwardWeight()
      {
         return 0.5;
      }

      @Override
      public double getFeedbackLateralWeight()
      {
         return 0.5;
      }

      @Override
      public double getFeedbackRegularizationWeight()
      {
         return 0.0001;
      }

      @Override
      public double getFeedbackParallelGain()
      {
         return 3.0;
      }

      @Override
      public double getFeedbackOrthogonalGain()
      {
         return 2.5;
      }

      @Override
      public double getDynamicRelaxationWeight()
      {
         return 500.0;
      }

      @Override
      public double getDynamicRelaxationDoubleSupportWeightModifier()
      {
         return 1.0;
      }

      @Override
      public double getAngularMomentumMinimizationWeight()
      {
         return 50;
      }

      @Override
      public boolean scaleStepRegularizationWeightWithTime()
      {
         return false;
      }

      @Override
      public boolean scaleFeedbackWeightWithGain()
      {
         return false;
      }

      @Override
      public boolean scaleUpcomingStepWeights()
      {
         return false;
      }

      @Override
      public boolean useFeedbackRegularization()
      {
         return false;
      }

      @Override
      public boolean useStepAdjustment()
      {
         return false;
      }

      @Override
      public boolean useAngularMomentum()
      {
         return false;
      }

      @Override
      public boolean useTimingOptimization()
      {
         return false;
      }

      @Override
      public boolean useFootstepRegularization()
      {
         return false;
      }

      @Override
      public double getMinimumFootstepWeight()
      {
         return 0.0001;
      }

      @Override
      public double getMinimumFeedbackWeight()
      {
         return 0.0001;
      }

      @Override
      public double getMinimumTimeRemaining()
      {
         return 0.0001;
      }

      @Override
      public double getAdjustmentDeadband()
      {
         return 0.03;
      }
   }

}
