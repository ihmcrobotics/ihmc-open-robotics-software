package us.ihmc.robotics.controllers;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class EuclideanPositionController implements PositionController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final DoubleYoVariable positionErrorMagnitude;
   private final YoFrameVector positionError;
   private final YoFrameVector positionErrorCumulated;
   private final YoFrameVector velocityError;

   private final Matrix3d proportionalGainMatrix;
   private final Matrix3d derivativeGainMatrix;
   private final Matrix3d integralGainMatrix;

   //TODO: Take the filtering out of here if it doesn't make sense to be here.
   // If it does make sense, then clean it up and support it better.
   private final AlphaFilteredYoFrameVector filteredVelocity;
   private final FrameVector filteredVelocityTemp;
   private final DoubleYoVariable alphaVelocityFilter;

   private final ReferenceFrame bodyFrame;
   private final FrameVector proportionalTerm;
   private final FrameVector derivativeTerm;
   private final FrameVector integralTerm;
   private final FramePoint currentPosition;

   private final boolean visualize;
   private final YoFramePoint currentPositionViz, desiredPositionViz;

   private final YoFrameVector preLimitedOutput;
   private final RateLimitedYoFrameVector rateLimitedOutput;

   private final double dt;

   private final YoPositionPIDGains gains;

   public EuclideanPositionController(String prefix, ReferenceFrame bodyFrame, double dt, YoVariableRegistry parentRegistry)
   {
      this(prefix, bodyFrame, dt, false, parentRegistry);
   }

   public EuclideanPositionController(String prefix, ReferenceFrame bodyFrame, double dt, boolean visualize, YoVariableRegistry parentRegistry)
   {
      this(prefix, bodyFrame, dt, null, visualize, parentRegistry);
   }

   public EuclideanPositionController(String prefix, ReferenceFrame bodyFrame, double dt, YoPositionPIDGains gains, boolean visualize, YoVariableRegistry parentRegistry)
   {
      this.visualize = visualize;

      this.dt = dt;

      this.bodyFrame = bodyFrame;
      registry = new YoVariableRegistry(prefix + getClass().getSimpleName());

      if (gains == null) gains = new YoEuclideanPositionGains(prefix, registry);

      this.gains = gains;
      proportionalGainMatrix = gains.createProportionalGainMatrix();
      derivativeGainMatrix = gains.createDerivativeGainMatrix();
      integralGainMatrix = gains.createIntegralGainMatrix();

      positionErrorMagnitude = new DoubleYoVariable(prefix + "PositionErrorMagnitude", registry);
      positionError = new YoFrameVector(prefix + "PositionError", bodyFrame, registry);
      positionErrorCumulated = new YoFrameVector(prefix + "PositionErrorCumulated", bodyFrame, registry);
      velocityError = new YoFrameVector(prefix + "LinearVelocityError", bodyFrame, registry);

      currentPosition = new FramePoint(bodyFrame);
      proportionalTerm = new FrameVector(bodyFrame);
      derivativeTerm = new FrameVector(bodyFrame);
      integralTerm = new FrameVector(bodyFrame);

      alphaVelocityFilter = new DoubleYoVariable(prefix + "AlphaPosVel", registry);
      alphaVelocityFilter.set(0.0);
      filteredVelocity = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(prefix, "FiltVelocity", registry, alphaVelocityFilter, bodyFrame);
      filteredVelocityTemp = new FrameVector(bodyFrame);

      preLimitedOutput = new YoFrameVector(prefix + "PositionPreLimitedOutput", bodyFrame, registry);
      rateLimitedOutput = RateLimitedYoFrameVector.createRateLimitedYoFrameVector(prefix + "PositionRateLimitedOutput", "", registry, gains.getYoMaximumJerk(), dt, preLimitedOutput);

      currentPositionViz = visualize ? new YoFramePoint(prefix + "CurrentPosition", worldFrame, registry) : null;
      desiredPositionViz = visualize ? new YoFramePoint(prefix + "DesiredPosition", worldFrame, registry) : null;

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      rateLimitedOutput.reset();
   }

   public void compute(FrameVector output, FramePoint desiredPosition, FrameVector desiredVelocity, FrameVector currentVelocity, FrameVector feedForward)
   {
      currentVelocity.changeFrame(bodyFrame);
      filteredVelocity.update(currentVelocity);
      filteredVelocity.getFrameTuple(filteredVelocityTemp);

      computeProportionalTerm(desiredPosition);
      computeDerivativeTerm(desiredVelocity, filteredVelocityTemp);
      computeIntegralTerm();
      output.setToNaN(bodyFrame);
      output.add(proportionalTerm, derivativeTerm);
      output.add(integralTerm);

      
      // Limit the max acceleration of the feedback, but not of the feedforward...
      // JEP changed 150430 based on Atlas hitting limit stops.
      double outputLength = output.length();
      if (outputLength > gains.getMaximumAcceleration())
      {
         output.scale(gains.getMaximumAcceleration() / outputLength);
      }
      
      preLimitedOutput.set(output);
      rateLimitedOutput.update();
      rateLimitedOutput.getFrameTuple(output);

      feedForward.changeFrame(bodyFrame);
      output.add(feedForward);
   }

   private void computeProportionalTerm(FramePoint desiredPosition)
   {
      visualizeDesiredAndActualPositions(desiredPosition);

      desiredPosition.changeFrame(bodyFrame);
      positionErrorMagnitude.set(desiredPosition.distance(currentPosition));
      positionError.set(desiredPosition);

      proportionalTerm.set(desiredPosition);
      proportionalGainMatrix.transform(proportionalTerm.getVector());
   }

   private void computeDerivativeTerm(FrameVector desiredVelocity, FrameVector currentVelocity)
   {
      desiredVelocity.changeFrame(bodyFrame);
      currentVelocity.changeFrame(bodyFrame);

      derivativeTerm.sub(desiredVelocity, currentVelocity);
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

   private void visualizeDesiredAndActualPositions(FramePoint desiredPosition)
   {
      if (visualize)
      {
         desiredPositionViz.setAndMatchFrame(desiredPosition);
         currentPositionViz.setAndMatchFrame(currentPosition);
      }
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

   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }

   public void setMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      gains.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }

   public void setGains(YoPositionPIDGains gains)
   {
      setProportionalGains(gains.getProportionalGains());
      setDerivativeGains(gains.getDerivativeGains());
      setIntegralGains(gains.getIntegralGains(), gains.getMaximumIntegralError());
      setMaxAccelerationAndJerk(gains.getMaximumAcceleration(), gains.getMaximumJerk());
   }
}
