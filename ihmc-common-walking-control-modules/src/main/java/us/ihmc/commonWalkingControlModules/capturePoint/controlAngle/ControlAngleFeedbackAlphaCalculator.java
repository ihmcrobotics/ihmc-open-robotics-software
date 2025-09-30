package us.ihmc.commonWalkingControlModules.capturePoint.controlAngle;

import us.ihmc.commonWalkingControlModules.capturePoint.controller.ICPControllerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ControlAngleFeedbackAlphaCalculator implements ICPControllerParameters.FeedbackAlphaCalculator
{
   private static final double defaultAngleToStartDecreasingControl = Math.toRadians(40.0);
   private static final double defaultAngleToEndDecreasingControl = Math.toRadians(10.0);

   private final BooleanParameter useICPFeedbackScaling;
   private final DoubleParameter angleToStartDecreasingControl;
   private final DoubleParameter angleToEndDecreasingControl;

   private final YoDouble feedbackControlAngle;

   private final FramePoint2D lineOfSightStartVertex = new FramePoint2D();
   private final FramePoint2D lineOfSightEndVertex = new FramePoint2D();

   private final FrameVector2D startVector = new FrameVector2D();
   private final FrameVector2D endVector = new FrameVector2D();

   public ControlAngleFeedbackAlphaCalculator(YoRegistry registry)
   {
      useICPFeedbackScaling = new BooleanParameter("useICPFeedbackScaling", registry, true);
      angleToStartDecreasingControl = new DoubleParameter("angleToStartDecreasingControl", registry, defaultAngleToStartDecreasingControl);
      angleToEndDecreasingControl = new DoubleParameter("angleToEndDecreasingControl", registry, defaultAngleToEndDecreasingControl);

      feedbackControlAngle = new YoDouble("feedbackControlAngle", registry);
   }

   @Override
   public double computeAlpha(FramePoint2DReadOnly currentICP, FrameConvexPolygon2DReadOnly supportPolygon)
   {
      if (!useICPFeedbackScaling.getValue() || supportPolygon == null || supportPolygon.isPointInside(currentICP))
         return 0.0;

      supportPolygon.lineOfSightStartVertex(currentICP, lineOfSightStartVertex);
      supportPolygon.lineOfSightEndVertex(currentICP, lineOfSightEndVertex);

      startVector.sub(currentICP, lineOfSightStartVertex);
      endVector.sub(currentICP, lineOfSightEndVertex);

      feedbackControlAngle.set(Math.abs(startVector.angle(endVector)));
      double alphaValue = (angleToStartDecreasingControl.getValue() - feedbackControlAngle.getValue()) / (angleToStartDecreasingControl.getValue()
                                                                                                          - angleToEndDecreasingControl.getValue());

      return MathTools.clamp(alphaValue, 0.0, 1.0);
   }
}
