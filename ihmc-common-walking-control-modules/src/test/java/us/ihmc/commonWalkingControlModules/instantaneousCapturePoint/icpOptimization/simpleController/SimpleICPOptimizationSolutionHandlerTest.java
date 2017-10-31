package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController;

import org.junit.Assert;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.exceptions.NoConvergenceException;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;

import static org.junit.Assert.assertTrue;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class SimpleICPOptimizationSolutionHandlerTest
{
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testWithinDeadband()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      ICPOptimizationParameters parameters = new TestICPOptimizationParameters(0.05);
      SimpleICPOptimizationSolutionHandler solutionHandler = new SimpleICPOptimizationSolutionHandler(parameters, false, "test", registry);
      SimpleICPOptimizationQPSolver solver = new SimpleICPOptimizationQPSolver(parameters, 4, false);

      int numberOfSteps = 3;
      ArrayList<YoFramePoint2d> foostepSolutions = new ArrayList<>();
      ArrayList<FramePoint2D> unclippedFootstepSolutions = new ArrayList<>();
      ArrayList<FramePoint2D> referenceFootstepPositions = new ArrayList<>();
      ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();
      RobotSide robotSide = RobotSide.LEFT;
      for (int i = 0; i < numberOfSteps; i++)
      {
         foostepSolutions.add(new YoFramePoint2d("footstepSolution" + i, ReferenceFrame.getWorldFrame(), registry));
         unclippedFootstepSolutions.add(new FramePoint2D(ReferenceFrame.getWorldFrame()));

         FramePose footstepPose = new FramePose(ReferenceFrame.getWorldFrame());

         footstepPose.setPosition(0.5 * (i + 1), robotSide.negateIfRightSide(0.1), 0.0);
         upcomingFootsteps.add(new Footstep(robotSide, footstepPose, false));

         FramePoint2D referenceFootstepPosition = new FramePoint2D();
         footstepPose.getPosition2dIncludingFrame(referenceFootstepPosition);
         referenceFootstepPositions.add(referenceFootstepPosition);

         robotSide = robotSide.getOppositeSide();
      }

      double recursionMultiplier = Math.exp(-3.0 * 0.5);
      double footstepWeight = 5.0;
      FramePoint2D referenceFootstepPosition = referenceFootstepPositions.get(0);
      solver.setFootstepAdjustmentConditions(recursionMultiplier, footstepWeight, referenceFootstepPosition);

      double feedbackGain = 2.5;
      double feedbackWeight = 100.0;
      double dynamicsWeight = 10000.0;

      solver.setFeedbackConditions(feedbackGain, feedbackWeight, dynamicsWeight);

      FrameVector2D currentICPError = new FrameVector2D(ReferenceFrame.getWorldFrame(), 0.05, 0.0);
      FramePoint2D perfectCMP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.1, 0.0);
      try
      {
         solver.compute(currentICPError, perfectCMP);
      }
      catch (NoConvergenceException e)
      {
      }

      solutionHandler.extractFootstepSolutions(foostepSolutions, unclippedFootstepSolutions, upcomingFootsteps, 1, solver);

      assertTrue(false);
   }

   private class TestICPOptimizationParameters extends ICPOptimizationParameters
   {
      private final double deadbandSize;

      public TestICPOptimizationParameters(double deadbandSize)
      {
         this.deadbandSize = deadbandSize;
      }

      @Override public boolean useSimpleOptimization()
      {
         return false;
      }

      @Override public int getMaximumNumberOfFootstepsToConsider()
      {
         return 5;
      }

      @Override public int numberOfFootstepsToConsider()
      {
         return 0;
      }

      @Override public double getForwardFootstepWeight()
      {
         return 5.0;
      }

      @Override public double getLateralFootstepWeight()
      {
         return 5.0;
      }

      @Override public double getFootstepRegularizationWeight()
      {
         return 0.0001;
      }

      @Override public double getFeedbackForwardWeight()
      {
         return 2.0;
      }

      @Override public double getFeedbackLateralWeight()
      {
         return 2.0;
      }

      @Override public double getFeedbackRegularizationWeight()
      {
         return 0.0001;
      }

      @Override public double getFeedbackParallelGain()
      {
         return 2.0;
      }

      @Override public double getFeedbackOrthogonalGain()
      {
         return 3.0;
      }

      @Override public double getDynamicRelaxationWeight()
      {
         return 1000.0;
      }

      @Override public double getDynamicRelaxationDoubleSupportWeightModifier()
      {
         return 1.0;
      }

      @Override public double getAngularMomentumMinimizationWeight()
      {
         return 0.5;
      }

      @Override public boolean scaleStepRegularizationWeightWithTime()
      {
         return false;
      }

      @Override public boolean scaleFeedbackWeightWithGain()
      {
         return false;
      }

      @Override public boolean scaleUpcomingStepWeights()
      {
         return false;
      }

      @Override public boolean useFeedbackRegularization()
      {
         return false;
      }

      @Override public boolean useStepAdjustment()
      {
         return true;
      }

      @Override public boolean useTimingOptimization()
      {
         return false;
      }

      @Override public boolean useAngularMomentum()
      {
         return false;
      }

      @Override public boolean useFootstepRegularization()
      {
         return false;
      }

      @Override public double getMinimumFootstepWeight()
      {
         return 0.0001;
      }

      @Override public double getMinimumFeedbackWeight()
      {
         return 0.0001;
      }

      @Override
      public double getMinimumTimeRemaining()
      {
         return 0.001;
      }

      @Override public double getAdjustmentDeadband()
      {
         return deadbandSize;
      }
   };
}
