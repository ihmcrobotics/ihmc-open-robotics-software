package us.ihmc.commonWalkingControlModules.controlModules.foot.partialFoothold;

import us.ihmc.commonWalkingControlModules.controlModules.foot.ExplorationParameters;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector2d;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class KinematicFootRotationDetector implements FootRotationDetector
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FrameVector2D angularVelocity = new FrameVector2D();

   /** Alpha filter to filter the foot angular velocity. */
   private final YoDouble angularVelocityFilterBreakFrequency;
   /** Foot filtered angular velocity in the sole frame. The yaw rate is intentionally ignored. */
   private final AlphaFilteredYoFrameVector2d footAngularVelocityFiltered;

   /** Foot angular velocity around the estimated line of rotation. */
   private final YoDouble angularVelocityMagnitude;
   /** Threshold on the foot angular velocity. */
   private final YoDouble angularVelocityMagnitudeThreshold;
   private final YoBoolean isAngularVelocityPastThreshold;

   /** Threshold on the yaw rate of the line of rotation to determine whether or not the line of rotation is stable. */
   //   private final YoDouble stableLoRAngularVelocityThreshold;
   //   private final YoBoolean isLineOfRotationStable;

   /** Threshold on the transversal velocity of the CoR w.r.t. the LoR to determine whether or not the CoR is stable. */
   //   private final YoDouble stableCoRLinearVelocityThreshold;
   //   private final YoBoolean isCenterOfRotationStable;

   /** Amount that the foot drops or lifts around the axis of rotation */
   private final YoDouble footDropOrLift;
   /** Threshold on the foot drop around the line of rotation. */
   private final YoDouble footDropThreshold;
   private final YoBoolean isFootDropPastThreshold;

   /** Main output of this class that informs on whether or not the foot is rotating. */
   private final YoBoolean isFootRotating;

   private final FrameVector2D normalizedFootAngularVelocity = new FrameVector2D();
   private final FrameVector3D pointingBackwardVector = new FrameVector3D();

   private final MovingReferenceFrame soleFrame;

   public KinematicFootRotationDetector(String namePrefix, MovingReferenceFrame soleFrame, ExplorationParameters explorationParameters, double controllerDt,
                                        YoVariableRegistry parentRegistry)
   {
      this.soleFrame = soleFrame;

      String name = getClass().getSimpleName();
      YoVariableRegistry registry = new YoVariableRegistry(namePrefix + name);
      parentRegistry.addChild(registry);

      angularVelocityFilterBreakFrequency = explorationParameters.getAngularVelocityFilterBreakFrequency();
      footAngularVelocityFiltered = new AlphaFilteredYoFrameVector2d(namePrefix + "AngularVelocityFiltered", "", registry, () -> AlphaFilteredYoVariable
            .computeAlphaGivenBreakFrequencyProperly(angularVelocityFilterBreakFrequency.getDoubleValue(), controllerDt), soleFrame);

      //      stableLoRAngularVelocityThreshold = explorationParameters.getStableLoRAngularVelocityThreshold();
      //      isLineOfRotationStable = new YoBoolean(namePrefix + "IsLoRStable", registry);

      //      stableCoRLinearVelocityThreshold = explorationParameters.getStableCoRLinearVelocityThreshold();
      //      isCenterOfRotationStable = new YoBoolean(namePrefix + "IsCoRStable", registry);

      angularVelocityMagnitude = new YoDouble(namePrefix + "AngularVelocityMagnitude", registry);
      angularVelocityMagnitudeThreshold = explorationParameters.getAngularVelocityAroundLoRThreshold();
      isAngularVelocityPastThreshold = new YoBoolean(namePrefix + "IsAngularVelocityPastThreshold", registry);

      footDropOrLift = new YoDouble(namePrefix + "FootDropOrLift", registry);
      footDropThreshold = explorationParameters.getFootDropThreshold();
      isFootDropPastThreshold = new YoBoolean(namePrefix + "IsFootDropPastThreshold", registry);

      isFootRotating = new YoBoolean(namePrefix + "RotatingVelocity", registry);
   }

   public void reset()
   {
      footAngularVelocityFiltered.reset();
      angularVelocityMagnitude.set(Double.NaN);

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

      updateFootRotationEstimate();

      return isFootRotating.getBooleanValue();
   }

   public boolean isRotating()
   {
      return isFootRotating.getBooleanValue();
   }

   private void updateFootRotationEstimate()
   {
      isFootDropPastThreshold.set(footDropOrLift.getDoubleValue() < footDropThreshold.getDoubleValue());

      //      isLineOfRotationStable.set(Math.abs(lineOfRotationAngularVelocityFiltered.getDoubleValue()) < stableLoRAngularVelocityThreshold.getDoubleValue());
      //
      //      isCenterOfRotationStable.set(Math.abs(centerOfRotationTransverseVelocity.getDoubleValue()) < stableCoRLinearVelocityThreshold.getDoubleValue());

      isAngularVelocityPastThreshold.set(angularVelocityMagnitude.getDoubleValue() > angularVelocityMagnitudeThreshold.getDoubleValue());

      //      isFootRotating
      //            .set(isLineOfRotationStable.getBooleanValue() && isCenterOfRotationStable.getBooleanValue() && isAngularVelocityPastThreshold.getBooleanValue()
      //                       && isFootDropPastThreshold.getBooleanValue());
      //
      isFootRotating.set(isAngularVelocityPastThreshold.getBooleanValue() && isFootDropPastThreshold.getBooleanValue());
   }
}
