package us.ihmc.robotics.controllers;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class EuclideanPositionController implements PositionController
{
   private final YoVariableRegistry registry;

   private final YoFrameVector positionError;
   private final YoFrameVector positionErrorCumulated;
   private final YoFrameVector velocityError;

   private final Matrix3d proportionalGainMatrix;
   private final Matrix3d derivativeGainMatrix;
   private final Matrix3d integralGainMatrix;

   private final ReferenceFrame bodyFrame;
   private final FrameVector proportionalTerm;
   private final FrameVector derivativeTerm;
   private final FrameVector integralTerm;

   private final YoFrameVector feedbackLinearAcceleration;
   private final RateLimitedYoFrameVector rateLimitedFeedbackLinearAcceleration;

   private final double dt;

   private final YoPositionPIDGainsInterface gains;

   public EuclideanPositionController(String prefix, ReferenceFrame bodyFrame, double dt, YoVariableRegistry parentRegistry)
   {
      this(prefix, bodyFrame, dt, null, parentRegistry);
   }

   public EuclideanPositionController(String prefix, ReferenceFrame bodyFrame, double dt, YoPositionPIDGainsInterface gains, YoVariableRegistry parentRegistry)
   {
      this.dt = dt;

      this.bodyFrame = bodyFrame;
      registry = new YoVariableRegistry(prefix + getClass().getSimpleName());

      if (gains == null)
         gains = new YoEuclideanPositionGains(prefix, registry);

      this.gains = gains;
      proportionalGainMatrix = gains.createProportionalGainMatrix();
      derivativeGainMatrix = gains.createDerivativeGainMatrix();
      integralGainMatrix = gains.createIntegralGainMatrix();

      positionError = new YoFrameVector(prefix + "PositionError", bodyFrame, registry);
      positionErrorCumulated = new YoFrameVector(prefix + "PositionErrorCumulated", bodyFrame, registry);
      velocityError = new YoFrameVector(prefix + "LinearVelocityError", bodyFrame, registry);

      proportionalTerm = new FrameVector(bodyFrame);
      derivativeTerm = new FrameVector(bodyFrame);
      integralTerm = new FrameVector(bodyFrame);

      feedbackLinearAcceleration = new YoFrameVector(prefix + "FeedbackLinearAcceleration", bodyFrame, registry);
      rateLimitedFeedbackLinearAcceleration = RateLimitedYoFrameVector.createRateLimitedYoFrameVector(prefix + "RateLimitedFeedbackLinearAcceleration", "",
            registry, gains.getYoMaximumJerk(), dt, feedbackLinearAcceleration);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      rateLimitedFeedbackLinearAcceleration.reset();
   }

   @Override
   public void compute(FrameVector output, FramePoint desiredPosition, FrameVector desiredVelocity, FrameVector currentVelocity, FrameVector feedForward)
   {
      computeProportionalTerm(desiredPosition);
      computeDerivativeTerm(desiredVelocity, currentVelocity);
      computeIntegralTerm();
      output.setToNaN(bodyFrame);
      output.add(proportionalTerm, derivativeTerm);
      output.add(integralTerm);

      // Limit the max acceleration of the feedback, but not of the feedforward...
      // JEP changed 150430 based on Atlas hitting limit stops.
      double feedbackLinearAccelerationMagnitude = output.length();
      if (feedbackLinearAccelerationMagnitude > gains.getMaximumAcceleration())
      {
         output.scale(gains.getMaximumAcceleration() / feedbackLinearAccelerationMagnitude);
      }

      feedbackLinearAcceleration.set(output);
      rateLimitedFeedbackLinearAcceleration.update();
      rateLimitedFeedbackLinearAcceleration.getFrameTuple(output);

      feedForward.changeFrame(bodyFrame);
      output.add(feedForward);
   }

   private void computeProportionalTerm(FramePoint desiredPosition)
   {
      desiredPosition.changeFrame(bodyFrame);
      positionError.set(desiredPosition);

      // Limit the maximum position error considered for control action
      double maximumError = gains.getMaximumProportionalError();
      double errorMagnitude = positionError.length();
      if (errorMagnitude > maximumError)
      {
         derivativeTerm.scale(maximumError / errorMagnitude);
      }

      proportionalTerm.set(desiredPosition);
      proportionalGainMatrix.transform(proportionalTerm.getVector());
   }

   private void computeDerivativeTerm(FrameVector desiredVelocity, FrameVector currentVelocity)
   {
      desiredVelocity.changeFrame(bodyFrame);
      currentVelocity.changeFrame(bodyFrame);

      derivativeTerm.sub(desiredVelocity, currentVelocity);

      // Limit the maximum velocity error considered for control action
      double maximumVelocityError = gains.getMaximumDerivativeError();
      double velocityErrorMagnitude = derivativeTerm.length();
      if (velocityErrorMagnitude > maximumVelocityError)
      {
         derivativeTerm.scale(maximumVelocityError / velocityErrorMagnitude);
      }

      velocityError.set(derivativeTerm);
      derivativeGainMatrix.transform(derivativeTerm.getVector());
   }

   private void computeIntegralTerm()
   {
      if (gains.getMaximumIntegralError() < 1e-5)
      {
         integralTerm.setToZero(bodyFrame);
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

      positionErrorCumulated.getFrameTuple(integralTerm);
      integralGainMatrix.transform(integralTerm.getVector());
   }

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      gains.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      gains.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public void setIntegralGains(double integralGainX, double integralGainY, double integralGainZ, double maxIntegralError)
   {
      gains.setIntegralGains(integralGainX, integralGainY, integralGainZ, maxIntegralError);
   }

   public void setProportionalGains(double[] proportionalGains)
   {
      gains.setProportionalGains(proportionalGains);
   }

   public void setDerivativeGains(double[] derivativeGains)
   {
      gains.setDerivativeGains(derivativeGains);
   }

   public void setIntegralGains(double[] integralGains, double maxIntegralError)
   {
      gains.setIntegralGains(integralGains, maxIntegralError);
   }

   public void getPositionError(FrameVector positionErrorToPack)
   {
      positionError.getFrameTuple(positionErrorToPack);
   }

   @Override
   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }

   public void setMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      gains.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }

   public void setMaxDerivativeError(double maxDerivativeError)
   {
      gains.setMaxDerivativeError(maxDerivativeError);
   }

   public void setMaxProportionalError(double maxProportionalError)
   {
      gains.setMaxProportionalError(maxProportionalError);
   }

   public void setGains(PositionPIDGainsInterface gains)
   {
      this.gains.set(gains);
   }
}
