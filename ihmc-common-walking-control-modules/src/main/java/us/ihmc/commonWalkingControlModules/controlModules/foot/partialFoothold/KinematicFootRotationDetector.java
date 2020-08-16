package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This class is designed to detect whether or not a foot is rotating. It does this by detecting if the angular velocity of the foot is above a certain
 * threshold, and also if a certain point on the foot has lifted or dropped above a certain threshold.
 */
public class KinematicFootRotationDetector implements FootRotationDetector
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FrameVector2D angularVelocity = new FrameVector2D();

   /** Alpha filter to filter the foot angular velocity. */
   private final DoubleProvider angularVelocityFilterBreakFrequency;
   /** Foot filtered angular velocity in the sole frame. The yaw rate is intentionally ignored. */
   private final AlphaFilteredYoFrameVector2d footAngularVelocityFiltered;

   /** Foot angular velocity around the estimated line of rotation. */
   private final YoDouble angularVelocityMagnitude;
   /** Threshold on the foot angular velocity. */
   private final DoubleProvider angularVelocityMagnitudeThreshold;
   private final YoBoolean isAngularVelocityPastThreshold;

   /** Amount that the foot drops or lifts around the axis of rotation */
   private final YoDouble footDropOrLift;
   /** Threshold on the foot drop around the line of rotation. */
   private final DoubleProvider footDropThreshold;
   private final YoBoolean isFootDropPastThreshold;

   private final YoBoolean isFootRotating;

   private final FrameVector2D normalizedFootAngularVelocity = new FrameVector2D();
   private final FrameVector3D pointingBackwardVector = new FrameVector3D();

   private final MovingReferenceFrame soleFrame;

   public KinematicFootRotationDetector(RobotSide side,
                                        MovingReferenceFrame soleFrame,
                                        FootholdRotationParameters rotationParameters,
                                        double controllerDt,
                                        YoRegistry parentRegistry)
   {
      this.soleFrame = soleFrame;

      String namePrefix = side.getLowerCaseName() + "Kinematic";
      YoRegistry registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      parentRegistry.addChild(registry);

      angularVelocityFilterBreakFrequency = rotationParameters.getAngularVelocityFilterBreakFrequency();
      footAngularVelocityFiltered = new AlphaFilteredYoFrameVector2d(namePrefix + "AngularVelocityFiltered",
                                                                     "",
                                                                     registry,
                                                                     () -> AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(
                                                                           angularVelocityFilterBreakFrequency.getValue(),
                                                                           controllerDt),
                                                                     soleFrame);

      angularVelocityMagnitude = new YoDouble(namePrefix + "AngularVelocityMagnitude", registry);
      angularVelocityMagnitudeThreshold = rotationParameters.getAngularVelocityAroundLoRThreshold();
      isAngularVelocityPastThreshold = new YoBoolean(namePrefix + "IsAngularVelocityPastThreshold", registry);

      footDropOrLift = new YoDouble(namePrefix + "FootDropOrLift", registry);
      footDropThreshold = rotationParameters.getFootDropThreshold();
      isFootDropPastThreshold = new YoBoolean(namePrefix + "IsFootDropPastThreshold", registry);

      isFootRotating = new YoBoolean(namePrefix + "IsRotating", registry);
   }

   public void reset()
   {
      footAngularVelocityFiltered.reset();
      angularVelocityMagnitude.setToNaN();

      footDropOrLift.setToNaN();

      isFootRotating.set(false);
      isFootDropPastThreshold.set(false);
      isAngularVelocityPastThreshold.set(false);
   }

   public boolean compute()
   {
      angularVelocity.setIncludingFrame(soleFrame.getTwistOfFrame().getAngularPart());
      angularVelocity.changeFrameAndProjectToXYPlane(soleFrame);
      footAngularVelocityFiltered.update(angularVelocity);

      normalizedFootAngularVelocity.setIncludingFrame(footAngularVelocityFiltered);
      normalizedFootAngularVelocity.normalize();

      // Compute Foot Drop or Lift...
      pointingBackwardVector.setIncludingFrame(soleFrame, normalizedFootAngularVelocity.getY(), -normalizedFootAngularVelocity.getX(), 0.0);
      pointingBackwardVector.normalize();
      pointingBackwardVector.scale(0.15); // FIXME magic number?
      pointingBackwardVector.changeFrame(worldFrame);

      footDropOrLift.set(pointingBackwardVector.getZ());
      angularVelocityMagnitude.set(footAngularVelocityFiltered.length());

      isFootDropPastThreshold.set(Math.abs(footDropOrLift.getDoubleValue()) > Math.abs(footDropThreshold.getValue()));

      isAngularVelocityPastThreshold.set(angularVelocityMagnitude.getDoubleValue() > angularVelocityMagnitudeThreshold.getValue());

      isFootRotating.set(isAngularVelocityPastThreshold.getBooleanValue() && isFootDropPastThreshold.getBooleanValue());

      return isFootRotating.getBooleanValue();
   }

   public boolean isRotating()
   {
      return isFootRotating.getBooleanValue();
   }
}
