package us.ihmc.robotics.controllers;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;

public class EuclideanPositionController implements PositionController
{
   private final YoVariableRegistry registry;

   private final YoFrameVector positionError;
   private final YoFrameVector positionErrorCumulated;
   private final YoFrameVector velocityError;

   private final Matrix3DReadOnly proportionalGainMatrix;
   private final Matrix3DReadOnly derivativeGainMatrix;
   private final Matrix3DReadOnly integralGainMatrix;

   private final ReferenceFrame bodyFrame;
   private final FrameVector proportionalTerm;
   private final FrameVector derivativeTerm;
   private final FrameVector integralTerm;
   
   private final FramePoint desiredPosition = new FramePoint();
   private final FrameVector desiredVelocity = new FrameVector();
   private final FrameVector feedForwardLinearAction = new FrameVector();
   private final FrameVector actionFromPositionController = new FrameVector();

   private final YoFrameVector feedbackLinearAction;
   private final RateLimitedYoFrameVector rateLimitedFeedbackLinearAction;

   private final EuclideanTangentialDampingCalculator tangentialDampingCalculator;
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

      feedbackLinearAction = new YoFrameVector(prefix + "FeedbackLinearAction", bodyFrame, registry);
      rateLimitedFeedbackLinearAction = RateLimitedYoFrameVector.createRateLimitedYoFrameVector(prefix + "RateLimitedFeedbackLinearAction", "",
            registry, gains.getYoMaximumFeedbackRate(), dt, feedbackLinearAction);

      tangentialDampingCalculator = new EuclideanTangentialDampingCalculator(prefix, bodyFrame, gains.getTangentialDampingGains());

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      rateLimitedFeedbackLinearAction.reset();
   }

   public void resetIntegrator()
   {
      positionErrorCumulated.setToZero();
   }

   @Override
   public void compute(FrameVector output, FramePoint desiredPosition, FrameVector desiredVelocity, FrameVector currentVelocity, FrameVector feedForward)
   {
      computeProportionalTerm(desiredPosition);
      if (currentVelocity != null)
         computeDerivativeTerm(desiredVelocity, currentVelocity);

      computeIntegralTerm();
      output.setToNaN(bodyFrame);
      output.add(proportionalTerm, derivativeTerm);
      output.add(integralTerm);

      // Limit the max acceleration of the feedback, but not of the feedforward...
      // JEP changed 150430 based on Atlas hitting limit stops.
      double feedbackLinearActionMagnitude = output.length();
      if (feedbackLinearActionMagnitude > gains.getMaximumFeedback())
      {
         output.scale(gains.getMaximumFeedback() / feedbackLinearActionMagnitude);
      }

      feedbackLinearAction.set(output);
      rateLimitedFeedbackLinearAction.update();
      rateLimitedFeedbackLinearAction.getFrameTuple(output);

      feedForward.changeFrame(bodyFrame);
      output.add(feedForward);
   }
   
   /**
    * Computes linear portion of twist to pack
    */
   public void compute(Twist twistToPack, FramePose desiredPose, Twist desiredTwist)
   {
      checkBodyFrames(desiredTwist, twistToPack);
      checkBaseFrames(desiredTwist, twistToPack);
      checkExpressedInFrames(desiredTwist, twistToPack);

      twistToPack.setToZero(bodyFrame, desiredTwist.getBaseFrame(), bodyFrame);

      desiredPose.getPositionIncludingFrame(desiredPosition);
      desiredTwist.getLinearPart(desiredVelocity);
      desiredTwist.getLinearPart(feedForwardLinearAction);
      compute(actionFromPositionController, desiredPosition, desiredVelocity, null, feedForwardLinearAction);
      twistToPack.setLinearPart(actionFromPositionController.getVector());
   }
   
   private void checkBodyFrames(Twist desiredTwist, Twist currentTwist)
   {
      desiredTwist.getBodyFrame().checkReferenceFrameMatch(bodyFrame);
      currentTwist.getBodyFrame().checkReferenceFrameMatch(bodyFrame);
   }

   private void checkBaseFrames(Twist desiredTwist, Twist currentTwist)
   {
      desiredTwist.getBaseFrame().checkReferenceFrameMatch(currentTwist.getBaseFrame());
   }

   private void checkExpressedInFrames(Twist desiredTwist, Twist currentTwist)
   {
      desiredTwist.getExpressedInFrame().checkReferenceFrameMatch(bodyFrame);
      currentTwist.getExpressedInFrame().checkReferenceFrameMatch(bodyFrame);
   }

   private void computeProportionalTerm(FramePoint desiredPosition)
   {
      desiredPosition.changeFrame(bodyFrame);
      positionError.set(desiredPosition);

      // Limit the maximum position error considered for control action
      double maximumError = gains.getMaximumProportionalError();
      double errorMagnitude = positionError.length();
      positionError.getFrameTuple(proportionalTerm);
      if (errorMagnitude > maximumError)
      {
         proportionalTerm.scale(maximumError / errorMagnitude);
      }

      proportionalGainMatrix.transform(proportionalTerm.getVector());
   }

   private final Matrix3D tempMatrix = new Matrix3D();
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

      tempMatrix.set(derivativeGainMatrix);
      tangentialDampingCalculator.compute(positionError, tempMatrix);

      tempMatrix.transform(derivativeTerm.getVector());
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

   public void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      gains.setMaxFeedbackAndFeedbackRate(maxFeedback, maxFeedbackRate);
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
