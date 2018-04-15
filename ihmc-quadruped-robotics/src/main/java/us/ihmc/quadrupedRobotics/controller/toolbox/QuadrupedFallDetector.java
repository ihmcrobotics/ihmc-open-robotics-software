package us.ihmc.quadrupedRobotics.controller.toolbox;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameConvexPolygon2DBasics;
import us.ihmc.robotics.math.filters.GlitchFilteredYoBoolean;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.IntegerParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class QuadrupedFallDetector
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public enum FallDetectionType
   {
      NONE, ROLL_LIMIT, PITCH_LIMIT, PITCH_AND_ROLL_LIMIT, DCM_OUTSIDE_SUPPORT_POLYGON_LIMIT, ALL
   }

   // Parameters
   private static final int DEFAULT_FALL_GLITCH_WINDOW = 1;
   private final DoubleParameter maxPitchInRad = new DoubleParameter("maxPitchInRad", registry, 0.5);
   private final DoubleParameter maxRollInRad = new DoubleParameter("maxRollInRad", registry, 0.5);
   private final DoubleParameter dcmOutsideSupportThreshold = new DoubleParameter("dcmDistanceOutsideSupportPolygonSupportThreshold", registry, 0.15);
   private final IntegerParameter fallDetectorGlitchFilterWindow = new IntegerParameter("fallDetectorGlitchFilterWindow", registry, DEFAULT_FALL_GLITCH_WINDOW);

   //Estimation Variables
   private final FrameQuaternion bodyOrientation = new FrameQuaternion();

   private final ReferenceFrame bodyFrame;
   private final FramePoint3D dcmPositionEstimate;
   private final FramePoint2D dcmPositionEstimate2D;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;
   private final FixedFrameConvexPolygon2DBasics supportPolygon;

   // Yo Variables
   private final YoDouble yoDcmDistanceOutsideSupportPolygon = new YoDouble("dcmDistanceOutsideSupportPolygon", registry);
   private final YoEnum<FallDetectionType> fallDetectionType = YoEnum.create("fallDetectionType", FallDetectionType.class, registry);
   private final GlitchFilteredYoBoolean isFallDetected;

   public QuadrupedFallDetector(ReferenceFrame bodyFrame, DivergentComponentOfMotionEstimator dcmPositionEstimator,
                                FixedFrameConvexPolygon2DBasics supportPolygon, YoVariableRegistry parentRegistry)
   {
      this.bodyFrame = bodyFrame;
      this.fallDetectionType.set(FallDetectionType.DCM_OUTSIDE_SUPPORT_POLYGON_LIMIT);
      this.dcmPositionEstimator = dcmPositionEstimator;
      this.supportPolygon = supportPolygon;

      this.isFallDetected = new GlitchFilteredYoBoolean("isFallDetected", registry, DEFAULT_FALL_GLITCH_WINDOW);
      this.isFallDetected.set(false);

      dcmPositionEstimate = new FramePoint3D();
      dcmPositionEstimate2D = new FramePoint2D();

      parentRegistry.addChild(registry);
   }

   public boolean detect()
   {
      updateEstimates();

      boolean isFallDetectedUnfiltered;
      switch (fallDetectionType.getEnumValue())
      {
      case DCM_OUTSIDE_SUPPORT_POLYGON_LIMIT:
         isFallDetectedUnfiltered = detectDcmDistanceOutsideSupportPolygonLimitFailure();
         break;
      case ROLL_LIMIT:
         isFallDetectedUnfiltered = detectRollLimitFailure();
         break;
      case PITCH_LIMIT:
         isFallDetectedUnfiltered = detectPitchLimitFailure();
         break;
      case PITCH_AND_ROLL_LIMIT:
         isFallDetectedUnfiltered = detectPitchLimitFailure() || detectRollLimitFailure();
         break;
      case ALL:
         isFallDetectedUnfiltered = detectDcmDistanceOutsideSupportPolygonLimitFailure() || detectPitchLimitFailure() || detectRollLimitFailure();
         break;
      default:
         isFallDetectedUnfiltered = false;
         break;
      }
      isFallDetected.setWindowSize(fallDetectorGlitchFilterWindow.getValue());
      isFallDetected.update(isFallDetectedUnfiltered);
      return isFallDetected.getBooleanValue();
   }

   private void updateEstimates()
   {
      bodyOrientation.setToZero(bodyFrame);
      bodyOrientation.changeFrame(worldFrame);

      dcmPositionEstimator.getDCMPositionEstimate(dcmPositionEstimate);
      dcmPositionEstimate2D.set(dcmPositionEstimate);
   }

   private boolean detectPitchLimitFailure()
   {
      return Math.abs(bodyOrientation.getPitch()) > maxPitchInRad.getValue();
   }

   private boolean detectRollLimitFailure()
   {
      return Math.abs(bodyOrientation.getRoll()) > maxRollInRad.getValue();
   }

   private boolean detectDcmDistanceOutsideSupportPolygonLimitFailure()
   {
      yoDcmDistanceOutsideSupportPolygon.set(supportPolygon.distance(dcmPositionEstimate2D));
      return supportPolygon.distance(dcmPositionEstimate2D) > dcmOutsideSupportThreshold.getValue();
   }
}
