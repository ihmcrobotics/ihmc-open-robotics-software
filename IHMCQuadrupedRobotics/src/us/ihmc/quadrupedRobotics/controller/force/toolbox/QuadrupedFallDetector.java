package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.quadrupedRobotics.geometry.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.IntegerParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.filters.GlitchFilteredBooleanYoVariable;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedFallDetector
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   public enum FallDetectionType
   {
      NONE, ROLL_LIMIT, PITCH_LIMIT, PITCH_AND_ROLL_LIMIT, DCM_OUTSIDE_SUPPORT_POLYGON_LIMIT, ALL
   }

   // Parameters
   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter maxPitchInRad = parameterFactory.createDouble("maxPitchInRad", 1.0);
   private final DoubleParameter maxRollInRad = parameterFactory.createDouble("maxRollInRad", 1.0);
   private final DoubleParameter dcmOutsideSupportThreshold = parameterFactory.createDouble("dcmDistanceOutsideSupportPolygonSupportThreshold", 0.05);
   private final IntegerParameter fallDetectorGlitchFilterWindow = parameterFactory.createInteger("fallDetectorGlitchFilterWindow", 1);

   //Estimation Variables
   private final QuadrupedTaskSpaceEstimator.Estimates taskSpaceEstimates;
   private final QuadrupedTaskSpaceEstimator taskSpaceEstimator;
   private final FramePoint dcmPositionEstimate;
   private final DivergentComponentOfMotionEstimator dcmPositionEstimator;
   private final QuadrupedSupportPolygon supportPolygon;

   // Yo Variables
   private final DoubleYoVariable yoDcmDistanceOutsideSupportPolygon = new DoubleYoVariable("dcmDistanceOutsideSupportPolygon", registry);
   private final EnumYoVariable<FallDetectionType> fallDetectionType = EnumYoVariable.create("fallDetectionType", FallDetectionType.class, registry);
   private final GlitchFilteredBooleanYoVariable isFallDetected;

   public QuadrupedFallDetector(QuadrupedTaskSpaceEstimator taskSpaceEstimator,
         DivergentComponentOfMotionEstimator dcmPositionEstimator, YoVariableRegistry parentRegistry)
   {
      this.fallDetectionType.set(FallDetectionType.DCM_OUTSIDE_SUPPORT_POLYGON_LIMIT);
      this.taskSpaceEstimator = taskSpaceEstimator;
      this.isFallDetected = new GlitchFilteredBooleanYoVariable("isFallDetected", registry, fallDetectorGlitchFilterWindow.get());
      this.isFallDetected.set(false);
      taskSpaceEstimates = new QuadrupedTaskSpaceEstimator.Estimates();
      dcmPositionEstimate = new FramePoint();
      this.dcmPositionEstimator = dcmPositionEstimator;
      supportPolygon = new QuadrupedSupportPolygon(taskSpaceEstimates.getSolePosition());
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
      isFallDetected.setWindowSize(fallDetectorGlitchFilterWindow.get());
      isFallDetected.update(isFallDetectedUnfiltered);
      return isFallDetected.getBooleanValue();
   }

   private void updateEstimates()
   {
      taskSpaceEstimator.compute(taskSpaceEstimates);
      dcmPositionEstimator.compute(dcmPositionEstimate, taskSpaceEstimates.getComVelocity());
      for (RobotQuadrant quadrant : RobotQuadrant.values())
      {
         taskSpaceEstimates.getSolePosition(quadrant).changeFrame(ReferenceFrame.getWorldFrame());
         supportPolygon.setFootstep(quadrant, taskSpaceEstimates.getSolePosition(quadrant));
      }
   }

   private boolean detectPitchLimitFailure()
   {
      taskSpaceEstimates.getBodyOrientation().changeFrame(ReferenceFrame.getWorldFrame());
      return Math.abs(taskSpaceEstimates.getBodyOrientation().getPitch()) > maxPitchInRad.get();
   }

   private boolean detectRollLimitFailure()
   {
      taskSpaceEstimates.getBodyOrientation().changeFrame(ReferenceFrame.getWorldFrame());
      return Math.abs(taskSpaceEstimates.getBodyOrientation().getRoll()) > maxRollInRad.get();
   }

   private boolean detectDcmDistanceOutsideSupportPolygonLimitFailure()
   {
      yoDcmDistanceOutsideSupportPolygon.set(supportPolygon.getDistanceOutside2d(dcmPositionEstimate));
      return supportPolygon.getDistanceOutside2d(dcmPositionEstimate) > dcmOutsideSupportThreshold.get();
   }
}
