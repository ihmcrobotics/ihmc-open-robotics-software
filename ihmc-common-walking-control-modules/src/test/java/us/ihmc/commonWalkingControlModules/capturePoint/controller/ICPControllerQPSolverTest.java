package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;

import static org.junit.jupiter.api.Assertions.*;

public class ICPControllerQPSolverTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 1e-3;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testStandingWithPerfectTrackingAndAngularMomentum()
   {
      ICPControllerQPSolver solver = new ICPControllerQPSolver(10);

      solver.setCMPFeedbackConditions(10.0, true);
      solver.setFeedbackConditions(0.1, 3.0, 500.0);

      FrameVector2D icpError = new FrameVector2D();
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.05, 0.01);

      assertTrue(solver.compute(icpError, perfectCMP));

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();

      EuclidFrameTestTools.assertGeometricallyEquals("The CoP feedback is wrong.", copFeedbackExpected, copFeedback, epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals("The CMP feedback is wrong.", cmpCoPDifferenceExpected, cmpCoPDifference, epsilon);
   }

   @Test
   public void testStandingUnconstrainedWithAngularMomentum()
   {
      ICPControllerQPSolver solver = new ICPControllerQPSolver(10);

      solver.setCMPFeedbackConditions(1000.0, true);
      solver.setFeedbackConditions(0.1, 0.1, 3.0, 3.0, 100000.0);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.05, 0.10);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.05, 0.01);

      assertTrue(solver.compute(icpError, perfectCMP));

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();

      copFeedbackExpected.set(icpError);
      copFeedbackExpected.scale(3.0);

      EuclidFrameTestTools.assertGeometricallyEquals("The CoP feedback is wrong.", copFeedbackExpected, copFeedback, epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals("The CMP feedback is wrong.", cmpCoPDifferenceExpected, cmpCoPDifference, epsilon);
   }

   @Test
   public void testStandingUnconstrained()
   {
      ICPControllerQPSolver solver = new ICPControllerQPSolver(10);

      solver.setFeedbackConditions(0.1, 3.0, 100000.0);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.05, 0.10);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.05, 0.01);

      assertTrue(solver.compute(icpError, perfectCMP));

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();

      copFeedbackExpected.set(icpError);
      copFeedbackExpected.scale(3.0);

      EuclidFrameTestTools.assertGeometricallyEquals("The CoP feedback is wrong.", copFeedbackExpected, copFeedback, epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals("The CMP feedback is wrong.", cmpCoPDifferenceExpected, cmpCoPDifference, epsilon);
   }

   @Test
   public void testStandingConstrained()
   {
      ICPControllerQPSolver solver = new ICPControllerQPSolver(10);
      solver.setMaxNumberOfIterations(10);

      // create support polygon constraint
      double sideLength = 0.1;
      FrameConvexPolygon2D supportPolygon = createSupportPolygon(sideLength);

      solver.setFeedbackConditions(0.1, 3.0, 1000.0);
      solver.addSupportPolygon(supportPolygon);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.04);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.02, 0.01);

      assertTrue(solver.compute(icpError, perfectCMP));

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

      EuclidFrameTestTools.assertGeometricallyEquals("The CoP feedback is wrong.", copFeedbackExpected, copFeedback, epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals("The CMP feedback is wrong.", cmpCoPDifferenceExpected, cmpCoPDifference, epsilon);
   }

   @Test
   public void testStandingWithRateLimitConstraints()
   {
      ICPControllerQPSolver solver = new ICPControllerQPSolver(10);
      solver.setMaxNumberOfIterations(10);

      // create support polygon constraint
      double feedbackGain = 3.0;
      solver.setFeedbackConditions(0.1, feedbackGain, 100000.0);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.05, 0.10);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.05, 0.01);

      assertTrue(solver.compute(icpError, perfectCMP));

      FrameVector2D cmpCoPDifference = new FrameVector2D();
      FrameVector2D copFeedback = new FrameVector2D();

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2D cmpCoPDifferenceExpected = new FrameVector2D();
      FrameVector2D copFeedbackExpected = new FrameVector2D();

      copFeedbackExpected.set(icpError);
      copFeedbackExpected.scale(feedbackGain);

      EuclidFrameTestTools.assertGeometricallyEquals("The CoP feedback is wrong.", copFeedbackExpected, copFeedback, epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals("The CMP feedback is wrong.", cmpCoPDifferenceExpected, cmpCoPDifference, epsilon);

      FrameVector2D previousCMPFeedback = new FrameVector2D();
      previousCMPFeedback.add(copFeedback, cmpCoPDifference);

      // now set the rate limit, and introduce a large change in the icp error
      double maxCMPRate = 3.0;
      double controlDt = 1e-3;
      solver.setMaximumFeedbackRate(maxCMPRate, controlDt);

      icpError.set(-0.05, -0.1);

      assertTrue(solver.compute(icpError, perfectCMP));

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      FrameVector2D unlimitedCopFeedbackExpected = new FrameVector2D();
      unlimitedCopFeedbackExpected.set(icpError);
      unlimitedCopFeedbackExpected.scale(feedbackGain);

      double maxChange = maxCMPRate * controlDt;


      FrameVector2D copFeedbackDelta = new FrameVector2D();
      copFeedbackDelta.sub(unlimitedCopFeedbackExpected, copFeedbackExpected);
      copFeedbackDelta.setX(MathTools.clamp(copFeedbackDelta.getX(), maxChange));
      copFeedbackDelta.setY(MathTools.clamp(copFeedbackDelta.getY(), maxChange));

      FrameVector2D limitedCoPFeedbackExpected = new FrameVector2D();
      limitedCoPFeedbackExpected.add(copFeedbackExpected, copFeedbackDelta);

      FrameVector2D cmpFeedback = new FrameVector2D();
      cmpFeedback.add(copFeedback, cmpCoPDifference);

      EuclidFrameTestTools.assertGeometricallyEquals("The CoP feedback is wrong.", limitedCoPFeedbackExpected, copFeedback, epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals("The CMP feedback is wrong.", cmpCoPDifferenceExpected, cmpCoPDifference, epsilon);

      double actualXChange = Math.abs(cmpFeedback.getX() - previousCMPFeedback.getX());
      double actualYChange = Math.abs(cmpFeedback.getY() - previousCMPFeedback.getY());
      assertTrue(actualXChange < maxChange + epsilon, "The actual X change " + actualXChange + " is greater than the maximum allowed change from the rate limit " + maxChange);
      assertTrue(actualYChange < maxChange + epsilon, "The actual Y change " + actualYChange + " is greater than the maximum allowed change from the rate limit " + maxChange);

   }

   @Test
   public void testStandingConstrainedWithAngularMomentum()
   {
      ICPControllerQPSolver solver = new ICPControllerQPSolver(10);
      solver.setMaxNumberOfIterations(10);

      // create support polygon constraint
      double sideLength = 0.1;
      FrameConvexPolygon2D supportPolygon = createSupportPolygon(sideLength);

      solver.setFeedbackConditions(0.1, 3.0, 10000.0);
      solver.setCMPFeedbackConditions(10.0, true);
      solver.addSupportPolygon(supportPolygon);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.04);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.02, 0.01);

      assertTrue(solver.compute(icpError, perfectCMP));

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

      EuclidFrameTestTools.assertGeometricallyEquals("The CoP feedback is wrong.", copFeedbackExpected, copFeedback, epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals("The CMP feedback is wrong.", cmpCoPDifferenceExpected, cmpCoPDifference, epsilon);
   }

   private FrameConvexPolygon2D createSupportPolygon(double sideLength)
   {
      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
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

   @Test
   public void testNoExceptions() throws Exception
   {
      ICPControllerQPSolver solver = new ICPControllerQPSolver(10);

      // feedback
      FramePoint2D scaledFeedbackWeight = new FramePoint2D(worldFrame, 0.2826586940121205, 0.2826586940121205);
      double feedbackGainX = 3.61466302555;
      double feedbackGainY = 3.88533697445;
      solver.resetCoPFeedbackConditions();
      solver.setFeedbackConditions(scaledFeedbackWeight.getX(), scaledFeedbackWeight.getY(), feedbackGainX, feedbackGainY, 10000.0);
      solver.setMaxCMPDistanceFromEdge(0.06);
      solver.setCopSafeDistanceToEdge(0.002);
      solver.setFeedbackRateWeight(0.0025, 0.0025);

      // angular momentum
      solver.resetCMPFeedbackConditions();
      solver.setCMPFeedbackConditions(10.769105592072197, true);

      // set previous solutions
      FramePoint2D previousFootstepSolution = new FramePoint2D(worldFrame, 0.2881204908019306, -0.6381315022331429);
      FramePoint2D previousFeedbackDeltaSolution = new FramePoint2D(worldFrame, -0.141764770527381, -0.04745626887921585);
      solver.resetFeedbackRate(previousFeedbackDeltaSolution);

      // set constraints
      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D(worldFrame);
      supportPolygon.addVertex(new FramePoint2D(worldFrame, -0.12497345851902948, 0.22376754760881368));
      supportPolygon.addVertex(new FramePoint2D(worldFrame, 0.10434696885177858, 0.2110446302571209));
      supportPolygon.addVertex(new FramePoint2D(worldFrame, 0.10438193051572264, 0.12238795176076478));
      supportPolygon.addVertex(new FramePoint2D(worldFrame, -0.12418538788019835, 0.10913840779165224));
      supportPolygon.update();

      solver.addSupportPolygon(supportPolygon);

      FrameVector2D icpError = new FrameVector2D(worldFrame, -0.07380072407166109, -0.05497512056196603);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.020230791294742534, 0.1586256502408977);

      assertTrue(solver.compute(icpError, perfectCMP));
   }

   @Test
   public void testSimpleNoExceptions() throws Exception
   {
      ICPControllerQPSolver solver = new ICPControllerQPSolver(10);

      // feedback
      FramePoint2D scaledFeedbackWeight = new FramePoint2D(worldFrame, 0.283, 0.283);
      double feedbackGainX = 3.5;
      double feedbackGainY = 4.0;
      solver.resetCoPFeedbackConditions();
      solver.setFeedbackConditions(scaledFeedbackWeight.getX(), scaledFeedbackWeight.getY(), feedbackGainX, feedbackGainY, 100000.0);
      solver.setMaxCMPDistanceFromEdge(0.06);
      solver.setCopSafeDistanceToEdge(0.002);
      solver.setFeedbackRateWeight(0.0025, 0.0025);

      // angular momentum
      solver.resetCMPFeedbackConditions();
      solver.setCMPFeedbackConditions(10.77, true);

      // set previous solutions
      FramePoint2D previousFeedbackDeltaSolution = new FramePoint2D(worldFrame, -0.142, -0.047);
      solver.resetFeedbackRate(previousFeedbackDeltaSolution);

      // set constraints
      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D(worldFrame);
      supportPolygon.addVertex(new FramePoint2D(worldFrame, -0.125, 0.224));
      supportPolygon.addVertex(new FramePoint2D(worldFrame, 0.104, 0.211));
      supportPolygon.addVertex(new FramePoint2D(worldFrame, 0.104, 0.122));
      supportPolygon.addVertex(new FramePoint2D(worldFrame, -0.124, 0.109));
      supportPolygon.update();

      solver.addSupportPolygon(supportPolygon);

      FrameVector2D icpError = new FrameVector2D(worldFrame, -0.074, -0.055);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.020, 0.159);

      assertTrue(solver.compute(icpError, perfectCMP));
   }

   @Test
   public void testStandingConstrainedWithDesiredFeedbackDirection()
   {
      ICPControllerQPSolver solver = new ICPControllerQPSolver(10);
      solver.setMaxNumberOfIterations(10);

      // create support polygon constraint
      double footLength = 0.2;
      double toeWidth = 0.08;
      double heelWidth = 0.1;
      FrameConvexPolygon2D supportPolygon = createWeirdSupportPolygon(footLength, toeWidth, heelWidth);

      solver.setFeedbackConditions(10.0, 3.0, 1000.0);
      solver.addSupportPolygon(supportPolygon);

      // push it out the side. Likely would result in some weird offset
      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.0, 0.08);
      FramePoint2D perfectCMP = new FramePoint2D(supportPolygon.getCentroid());

      //      solver.setDesiredFeedbackDirection(icpError, 1e5);

      assertTrue(solver.compute(icpError, perfectCMP));

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

      double midY = 0.25 * (toeWidth + heelWidth);

      // clip to support polygon
      copFeedbackExpected.setX(Math.min(copFeedbackExpected.getX(), 0.0));
      copFeedbackExpected.setY(Math.min(copFeedbackExpected.getY(), midY));
      copFeedbackExpected.sub(perfectCMP);

      assertFalse(copFeedback.epsilonEquals(copFeedbackExpected, epsilon), "The CoP feedback is wrong.");

      solver.setDesiredFeedbackDirection(icpError, 1e5);
      assertTrue(solver.compute(icpError, perfectCMP));

      solver.getCMPFeedbackDifference(cmpCoPDifference);
      solver.getCoPFeedbackDifference(copFeedback);

      EuclidFrameTestTools.assertGeometricallyEquals("The CoP feedback is wrong.", copFeedbackExpected, copFeedback, epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals("The CMP feedback is wrong.", cmpCoPDifferenceExpected, cmpCoPDifference, epsilon);

   }

   private FrameConvexPolygon2D createWeirdSupportPolygon(double footLength, double toeWidth, double heelWidth)
   {
      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
      FramePoint2D backLeft = new FramePoint2D(worldFrame, -0.5 * footLength, 0.5 * heelWidth);
      FramePoint2D frontLeft = new FramePoint2D(worldFrame, 0.5 * footLength, 0.5 * toeWidth);
      FramePoint2D frontRight = new FramePoint2D(worldFrame, 0.5 * footLength, -0.5 * toeWidth);
      FramePoint2D backRight = new FramePoint2D(worldFrame, -0.5 * footLength, -0.5 * heelWidth);

      supportPolygon.addVertex(backLeft);
      supportPolygon.addVertex(frontLeft);
      supportPolygon.addVertex(frontRight);
      supportPolygon.addVertex(backRight);

      supportPolygon.update();

      return supportPolygon;
   }

   @Test
   public void testTransferConstrainedTrickyCase()
   {
      ICPControllerQPSolver solver = new ICPControllerQPSolver(10);
      ICPControllerHelper helper = new ICPControllerHelper();
      solver.setMaxNumberOfIterations(10);

      // create support polygon constraint
      double forwardWeight = 1.5;
      double lateralWeight = 1.5;
      double dynamicsObjectiveWeight = 1e4;
      double parallelGain = 2.5;
      double orthogonalGain = 2.0;
      double feedbackDirectionWeight = 1e6;
      double maxCMPDistanceFromEdge = 0.04;
      double copSafeDistanceToEdge = 0.001;
      double feedbackpartMaxRate = Double.POSITIVE_INFINITY;
      double controlDt = 0.004;

      FrameConvexPolygon2D leftFootPolygon = new FrameConvexPolygon2D();
      FrameConvexPolygon2D rightFootPolygon = new FrameConvexPolygon2D();
      leftFootPolygon.addVertex(0.6743, 0.5168);
      leftFootPolygon.addVertex(0.8812, 0.5926);
      leftFootPolygon.addVertex(0.9148, 0.5148);
      leftFootPolygon.addVertex(0.7178, 0.416);
      rightFootPolygon.addVertex(0.4147, 0.0098);
      rightFootPolygon.addVertex(0.6317, 0.0469);
      rightFootPolygon.addVertex(0.651, -0.0357);
      rightFootPolygon.addVertex(0.4396, -0.0972);
      leftFootPolygon.update();
      rightFootPolygon.update();
      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
      supportPolygon.set(leftFootPolygon, rightFootPolygon);

      FrameVector2D desiredICPVelocity = new FrameVector2D(worldFrame, 0.1667, 0.5145);

      DMatrixRMaj transformedGains = new DMatrixRMaj(2, 2);
      helper.transformGainsFromDynamicsFrame(transformedGains, desiredICPVelocity, parallelGain, orthogonalGain);


      DMatrixRMaj scaledCoPFeedbackWeight = new DMatrixRMaj(2, 2);
      helper.transformFromDynamicsFrame(scaledCoPFeedbackWeight, desiredICPVelocity, forwardWeight, lateralWeight);
      double magnitude = helper.transformGainsFromDynamicsFrame(transformedGains, desiredICPVelocity, parallelGain, orthogonalGain);
      CommonOps_DDRM.scale(1.0 / magnitude, scaledCoPFeedbackWeight);

      FrameVector2D unconstrainedFeedbackDelta = new FrameVector2D(worldFrame, -0.0609, -0.1832);
      FramePoint2D unconstrainedFeedbackCMP = new FramePoint2D(worldFrame, 0.5367, -0.1996);

      solver.addSupportPolygon(supportPolygon);

      solver.setFeedbackConditions(scaledCoPFeedbackWeight, transformedGains, dynamicsObjectiveWeight);
      solver.setMaxCMPDistanceFromEdge(maxCMPDistanceFromEdge);
      solver.setCopSafeDistanceToEdge(copSafeDistanceToEdge);
      solver.setDesiredFeedbackDirection(unconstrainedFeedbackDelta, feedbackDirectionWeight);

      solver.setMaximumFeedbackMagnitude(CommonOps_DDRM.identity(2), Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      solver.setMaximumFeedbackRate(feedbackpartMaxRate, controlDt);

      solver.setFeedbackRateWeight(0.0, 5e-8 / (controlDt * controlDt));
      solver.setCMPFeedbackConditions(10.0, true);

      // push it out the side. Likely would result in some weird offset
      FrameVector2D icpError = new FrameVector2D(worldFrame, -0.0175, -0.0523);
      FramePoint2D perfectCoP = new FramePoint2D(worldFrame, 0.5955, -0.0249);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.5976, -0.0164);
      FrameVector2D perfectCMPOffset = new FrameVector2D();
      perfectCMPOffset.sub(perfectCMP, perfectCoP);

      assertTrue(solver.compute(icpError, perfectCoP, perfectCMPOffset));

      FrameVector2D feedbackCMPDelta = new FrameVector2D();
      FrameVector2D feedbackCoPDelta = new FrameVector2D();

      solver.getCMPFeedbackDifference(feedbackCMPDelta);
      solver.getCoPFeedbackDifference(feedbackCoPDelta);

      FrameVector2D feedbackDelta = new FrameVector2D();
      feedbackDelta.add(feedbackCMPDelta, feedbackCoPDelta);

      FramePoint2D feedbackCMP = new FramePoint2D(perfectCMP);
      feedbackCMP.add(feedbackDelta);


      FramePoint2D unconstrainedCMPAlt = new FramePoint2D(perfectCMP);
      unconstrainedCMPAlt.add(unconstrainedFeedbackDelta);

      EuclidFrameTestTools.assertGeometricallyEquals(unconstrainedCMPAlt, unconstrainedFeedbackCMP, 1e-5);

      DMatrixRMaj expectedSolution = new DMatrixRMaj(4, 1);
      feedbackCoPDelta.get(expectedSolution);
      feedbackCMPDelta.get(2, expectedSolution);

      MatrixTestTools.assertMatrixEquals(expectedSolution, solver.solution, 1e-5);

      // check to make sure the objective was almost perfectly satisfied
      DMatrixRMaj resultant = new DMatrixRMaj(1, 1);
      CommonOps_DDRM.mult(solver.inputCalculator.directionJacobian, solver.solution, resultant);
      assertEquals(0.0, resultant.get(0, 0), 1e-3);

      FrameVector3D unconstrainedVector3D = new FrameVector3D(unconstrainedFeedbackDelta);
      unconstrainedVector3D.normalize();
      FrameVector3D feedbackDelta3D = new FrameVector3D(feedbackDelta);
      FrameVector3D cross = new FrameVector3D();
      cross.cross(unconstrainedVector3D, feedbackDelta3D);
      assertEquals(0.0, cross.getZ(), 1e-3);

      // check to make sure the actual goal was almost perfectly satisfied
      assertEquals(0.0, unconstrainedFeedbackDelta.angle(feedbackDelta), 2e-3);

      ConvexPolygonScaler scaler = new ConvexPolygonScaler();
      FrameConvexPolygon2D scaledSupportPolygon = new FrameConvexPolygon2D();
      scaler.scaleConvexPolygon(supportPolygon, -maxCMPDistanceFromEdge, scaledSupportPolygon);

      Point2D[] intersection = EuclidGeometryPolygonTools.intersectionBetweenLineSegment2DAndConvexPolygon2D(perfectCMP,
                                                                                                             unconstrainedFeedbackCMP,
                                                                                                             scaledSupportPolygon.getVertexBufferView(),
                                                                                                             scaledSupportPolygon.getNumberOfVertices(),
                                                                                                             true);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(intersection[0], feedbackCMP, 1e-3);
   }
}

