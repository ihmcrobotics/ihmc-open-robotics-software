package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ErrorBasedFeedForwardAlphaCalculator implements ICPControllerParameters.FeedForwardAlphaCalculator
{
   // Control Parameters:
   private final DoubleProvider pureFeedbackErrorThreshold;


   // Algorithm Inputs:
   private final FrameVector2D parallelDirection = new FrameVector2D();
   private final FrameVector2D perpDirection = new FrameVector2D();

   private final FrameVector2D icpError = new FrameVector2D();

   public ErrorBasedFeedForwardAlphaCalculator(String yoNamePrefix, YoRegistry registry)
   {
      pureFeedbackErrorThreshold = new DoubleParameter(yoNamePrefix + "PureFeedbackErrorThresh",
                                                       "Amount of ICP error before feedforward terms are ignored.",
                                                       registry,
                                                       0.06);
   }

   @Override
   public double computeAlpha(FramePoint2DReadOnly currentICP,
                              FramePoint2DReadOnly referenceICP,
                              FramePoint2DReadOnly finalICP,
                              FramePoint2DReadOnly referenceCMP,
                              FramePoint2DReadOnly unconstrainedFeedbackCMP,
                              FrameConvexPolygon2DReadOnly supportPolygon)
   {
      this.parallelDirection.sub(referenceICP, referenceCMP);

      double icpPerpError = 0.0;

      if (parallelDirection.lengthSquared() > 1e-7)
      {
         parallelDirection.normalize();
         perpDirection.set(-parallelDirection.getY(), parallelDirection.getX());

         icpPerpError = icpError.dot(perpDirection);
      }

      // Compute feedbackFeedforwardAlpha, which if 1.0 means to ignore the feedforward terms from the perfectCoP/CMP.
      // If it equals 0.0, then use all of the feedforward.
      // As the perpendicular error grows, start ignoring the feedforward at a certain percentage of the threshold.
      // Ignore the feedforward more and more as the perpendicular error grows. If the perpendicular error is greater
      // than pureFeedbackErrorThreshold, then apply only feedback.
      double percentOfPerpendicularThresholdToStartIgnoringFeedforward = 0.5;
      double perpendicularErrorToStartIgnoringFeedforward = pureFeedbackErrorThreshold.getValue() * percentOfPerpendicularThresholdToStartIgnoringFeedforward;
      double perpendicularErrorMagnitude = Math.abs(icpPerpError);
      return computePercentageOfRangeClampedBetweenZeroAndOne(perpendicularErrorMagnitude,
                                                              perpendicularErrorToStartIgnoringFeedforward,
                                                              pureFeedbackErrorThreshold.getValue());
   }

   private static double computePercentageOfRangeClampedBetweenZeroAndOne(double value, double lowerBoundOfRange, double upperBoundOfRange)
   {
      double percent = (value - lowerBoundOfRange) / (upperBoundOfRange - lowerBoundOfRange);
      return EuclidCoreTools.clamp(percent, 0.0, 1.0);
   }
}
