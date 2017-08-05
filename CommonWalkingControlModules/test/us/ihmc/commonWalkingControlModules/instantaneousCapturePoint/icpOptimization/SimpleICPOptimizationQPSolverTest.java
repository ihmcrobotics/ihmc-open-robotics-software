package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import org.jcodec.common.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController.SimpleICPOptimizationQPSolver;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SimpleICPOptimizationQPSolverTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 1e-4;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStandingWithPerfectTrackingAndAngularMomentum() throws Exception
   {
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(0.0, 0.0, 10, false);

      solver.setAngularMomentumConditions(10.0, true);
      solver.setFeedbackConditions(0.1, 0.1, 2.5, 3.0, 500.0);

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
