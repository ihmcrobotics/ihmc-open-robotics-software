package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import static us.ihmc.robotics.Assert.assertTrue;

import org.jcodec.common.Assert;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;

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

      EuclidFrameTestTools.assertFrameVector2DGeometricallyEquals("The CoP feedback is wrong.", copFeedbackExpected, copFeedback, epsilon);
      EuclidFrameTestTools.assertFrameVector2DGeometricallyEquals("The CMP feedback is wrong.", cmpCoPDifferenceExpected, cmpCoPDifference, epsilon);
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

      Assert.assertTrue("The CoP feedback is wrong.", copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue("The CMP Feedback is wrong.", cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
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

      Assert.assertTrue("The CoP feedback is wrong.", copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue("The CMP feedback is wrong.", cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
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

      Assert.assertTrue("The CoP feedback is wrong.", copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue("The CMP feedback is wrong.", cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
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

      Assert.assertTrue("The CoP feedback is wrong.", copFeedback.epsilonEquals(copFeedbackExpected, epsilon));
      Assert.assertTrue("The CMP feedback is wrong.", cmpCoPDifference.epsilonEquals(cmpCoPDifferenceExpected, epsilon));
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
}

