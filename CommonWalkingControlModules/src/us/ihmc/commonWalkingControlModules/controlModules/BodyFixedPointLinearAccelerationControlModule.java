package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.robotics.controllers.PositionPIDGainsInterface;
import us.ihmc.robotics.controllers.YoEuclideanPositionGains;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class BodyFixedPointLinearAccelerationControlModule
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;
   private final TwistCalculator twistCalculator;
   private final RigidBody endEffector;
   private final Twist endEffectorTwist = new Twist();

   private final FramePoint bodyFixedPoint = new FramePoint();
   private final YoTranslationFrame frameAtBodyFixedPoint;

   private final YoFrameVector positionError;
   private final YoFrameVector positionErrorCumulated;
   private final YoFrameVector velocityError;

   private final Matrix3DReadOnly proportionalGainMatrix;
   private final Matrix3DReadOnly derivativeGainMatrix;
   private final Matrix3DReadOnly integralGainMatrix;

   private final FrameVector currentVelocity = new FrameVector();

   private final FrameVector proportionalTerm = new FrameVector();
   private final FrameVector derivativeTerm = new FrameVector();
   private final FrameVector integralTerm = new FrameVector();

   private final YoFrameVector feedbackLinearAcceleration;
   private final RateLimitedYoFrameVector rateLimitedFeedbackLinearAcceleration;

   private final YoPositionPIDGainsInterface gains;
   private final double dt;

   public BodyFixedPointLinearAccelerationControlModule(String namePrefix, TwistCalculator twistCalculator, RigidBody endEffector, double dt,
         YoVariableRegistry parentRegistry)
   {
      this(namePrefix, twistCalculator, endEffector, dt, null, parentRegistry);
   }

   public BodyFixedPointLinearAccelerationControlModule(String namePrefix, TwistCalculator twistCalculator, RigidBody endEffector, double dt,
         YoPositionPIDGainsInterface gains, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());

      this.dt = dt;
      this.twistCalculator = twistCalculator;
      this.endEffector = endEffector;

      if (gains == null)
         gains = new YoEuclideanPositionGains(namePrefix, registry);

      this.gains = gains;
      proportionalGainMatrix = gains.createProportionalGainMatrix();
      derivativeGainMatrix = gains.createDerivativeGainMatrix();
      integralGainMatrix = gains.createIntegralGainMatrix();

      frameAtBodyFixedPoint = new YoTranslationFrame(namePrefix + "BodyFixedPointFrame", endEffector.getBodyFixedFrame(), registry);

      positionError = new YoFrameVector(namePrefix + "PositionError", worldFrame, registry);
      positionErrorCumulated = new YoFrameVector(namePrefix + "PositionErrorCumulated", worldFrame, registry);
      velocityError = new YoFrameVector(namePrefix + "LinearVelocityError", worldFrame, registry);

      feedbackLinearAcceleration = new YoFrameVector(namePrefix + "FeedbackLinearAcceleration", worldFrame, registry);
      rateLimitedFeedbackLinearAcceleration = RateLimitedYoFrameVector.createRateLimitedYoFrameVector(namePrefix + "RateLimitedFeedbackLinearAcceleration", "",
            registry, gains.getYoMaximumFeedbackRate(), dt, feedbackLinearAcceleration);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      rateLimitedFeedbackLinearAcceleration.reset();
   }

   public void compute(FrameVector acceleration, FramePoint desiredPosition, FrameVector desiredVelocity, FrameVector feedForward, RigidBody base)
   {
      ReferenceFrame baseFrame = base.getBodyFixedFrame();

      computeProportionalTerm(desiredPosition, baseFrame);
      computeDerivativeTerm(desiredVelocity, base);
      computeIntegralTerm(baseFrame);

      acceleration.setIncludingFrame(proportionalTerm);
      acceleration.add(derivativeTerm);
      acceleration.add(integralTerm);

      // Limit the max acceleration of the feedback, but not of the feedforward...
      // JEP changed 150430 based on Atlas hitting limit stops.
      double feedbackLinearAccelerationMagnitude = acceleration.length();
      if (feedbackLinearAccelerationMagnitude > gains.getMaximumFeedback())
      {
         acceleration.scale(gains.getMaximumFeedback() / feedbackLinearAccelerationMagnitude);
      }

      feedbackLinearAcceleration.setAndMatchFrame(acceleration);
      rateLimitedFeedbackLinearAcceleration.update();
      rateLimitedFeedbackLinearAcceleration.getFrameTupleIncludingFrame(acceleration);

      acceleration.changeFrame(baseFrame);
      feedForward.changeFrame(baseFrame);
      acceleration.add(feedForward);
   }

   private void computeProportionalTerm(FramePoint desiredPosition, ReferenceFrame baseFrame)
   {
      desiredPosition.changeFrame(frameAtBodyFixedPoint);
      positionError.setAndMatchFrame(desiredPosition);

      positionError.getFrameTupleIncludingFrame(proportionalTerm);
      proportionalGainMatrix.transform(proportionalTerm.getVector());
      proportionalTerm.changeFrame(baseFrame);
   }

   private void computeDerivativeTerm(FrameVector desiredVelocity, RigidBody base)
   {
      ReferenceFrame baseFrame = base.getBodyFixedFrame();
      twistCalculator.getRelativeTwist(endEffectorTwist, base, endEffector);
      endEffectorTwist.changeFrame(baseFrame);
      bodyFixedPoint.setToZero(frameAtBodyFixedPoint);
      bodyFixedPoint.changeFrame(baseFrame);
      endEffectorTwist.getLinearVelocityOfPointFixedInBodyFrame(currentVelocity, bodyFixedPoint);

      desiredVelocity.changeFrame(baseFrame);
      currentVelocity.changeFrame(baseFrame);

      derivativeTerm.setToZero(baseFrame);
      derivativeTerm.sub(desiredVelocity, currentVelocity);

      velocityError.setAndMatchFrame(derivativeTerm);
      derivativeGainMatrix.transform(derivativeTerm.getVector());
   }

   private void computeIntegralTerm(ReferenceFrame baseFrame)
   {
      if (gains.getMaximumIntegralError() < 1e-5)
      {
         integralTerm.setToZero(baseFrame);
         return;
      }

      double errorIntegratedX = positionError.getX() * dt;
      double errorIntegratedY = positionError.getY() * dt;
      double errorIntegratedZ = positionError.getZ() * dt;
      positionErrorCumulated.add(errorIntegratedX, errorIntegratedY, errorIntegratedZ);

      double errorMagnitude = positionErrorCumulated.length();
      if (errorMagnitude > gains.getMaximumIntegralError())
      {
         positionErrorCumulated.scale(gains.getMaximumIntegralError() / errorMagnitude);
      }

      positionErrorCumulated.getFrameTupleIncludingFrame(integralTerm);
      integralTerm.changeFrame(baseFrame);
      integralGainMatrix.transform(integralTerm.getVector());
   }

   public void setGains(PositionPIDGainsInterface gains)
   {
      this.gains.set(gains);
   }

   public void getBodyFixedPoint(FramePoint bodyFixedPointToPack)
   {
      bodyFixedPointToPack.setToZero(frameAtBodyFixedPoint);
      bodyFixedPointToPack.changeFrame(endEffector.getBodyFixedFrame());
   }

   public void getBodyFixedPointCurrentLinearVelocity(FrameVector bodyFixedPointLinearVelocityToPack)
   {
      bodyFixedPointLinearVelocityToPack.setIncludingFrame(currentVelocity);
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
   }

   public void setPointToControl(FramePoint tempPosition)
   {
      frameAtBodyFixedPoint.setTranslationToParent(tempPosition);
   }
}
