package us.ihmc.robotics.controllers;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.filters.AlphaFilteredYoFrameVector;
import us.ihmc.robotics.math.filters.RateLimitedYoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;


public class AxisAngleOrientationController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;

   private final YoFrameVector axisAngleErrorInBody;
   private final DoubleYoVariable orientationErrorMagnitude;
   private final YoFrameVector orientationErrorCumulated;
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

   private final AxisAngle4d desiredAngleAxis = new AxisAngle4d();
   private final Quat4d desiredQuaternion = new Quat4d();

   private final YoFrameVector preLimitedOutput;
   private final RateLimitedYoFrameVector rateLimitedOutput;

   private final double dt;

   private final boolean visualize;
   private final YoFrameOrientation currentOrientationViz, desiredOrientationViz;
   private final FrameOrientation currentOrientation;

   private final YoOrientationPIDGains gains;

   public AxisAngleOrientationController(String prefix, ReferenceFrame bodyFrame, double dt, YoVariableRegistry parentRegistry)
   {
      this(prefix, bodyFrame, dt, false, parentRegistry);
   }

   public AxisAngleOrientationController(String prefix, ReferenceFrame bodyFrame, double dt, YoOrientationPIDGains gains, YoVariableRegistry parentRegistry)
   {
      this(prefix, bodyFrame, dt, gains, false, parentRegistry);
   }

   public AxisAngleOrientationController(String prefix, ReferenceFrame bodyFrame, double dt, boolean visualize, YoVariableRegistry parentRegistry)
   {
      this(prefix, bodyFrame, dt, null, visualize, parentRegistry);
   }

   public AxisAngleOrientationController(String prefix, ReferenceFrame bodyFrame, double dt, YoOrientationPIDGains gains, boolean visualize, YoVariableRegistry parentRegistry)
   {
      this.visualize = visualize;

      this.dt = dt;
      this.bodyFrame = bodyFrame;
      registry = new YoVariableRegistry(prefix + getClass().getSimpleName());

      if (gains == null) gains = new YoAxisAngleOrientationGains(prefix, registry);

      this.gains = gains;
      proportionalGainMatrix = gains.createProportionalGainMatrix();
      derivativeGainMatrix = gains.createDerivativeGainMatrix();
      integralGainMatrix = gains.createIntegralGainMatrix();

      axisAngleErrorInBody = new YoFrameVector(prefix + "AxisAngleErrorInBody", bodyFrame, registry);
      orientationErrorMagnitude = new DoubleYoVariable(prefix + "OrientationErrorMagnitude", registry);
      orientationErrorCumulated = new YoFrameVector(prefix + "OrientationErrorCumulated", bodyFrame, registry);
      velocityError = new YoFrameVector(prefix + "AngularVelocityError", bodyFrame, registry);

      proportionalTerm = new FrameVector(bodyFrame);
      derivativeTerm = new FrameVector(bodyFrame);
      integralTerm = new FrameVector(bodyFrame);

      alphaVelocityFilter = new DoubleYoVariable(prefix + "AlphaAngleVel", registry);
      alphaVelocityFilter.set(0.0);
      filteredVelocity = AlphaFilteredYoFrameVector.createAlphaFilteredYoFrameVector(prefix, "FiltAngularVelocity", registry, alphaVelocityFilter, bodyFrame);
      filteredVelocityTemp = new FrameVector(bodyFrame);

      preLimitedOutput = new YoFrameVector(prefix + "OrientationPreLimitedOutput", bodyFrame, registry);
      rateLimitedOutput = RateLimitedYoFrameVector.createRateLimitedYoFrameVector(prefix + "OrientationRateLimitedOutput", "", registry,
            gains.getYoMaximumJerk(), dt, preLimitedOutput);

      currentOrientationViz = visualize ? new YoFrameOrientation(prefix + "CurrentOrientation", worldFrame, registry) : null;
      desiredOrientationViz = visualize ? new YoFrameOrientation(prefix + "DesiredOrientation", worldFrame, registry) : null;
      currentOrientation = visualize ? new FrameOrientation(bodyFrame) : null;

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      rateLimitedOutput.reset();
   }

   public void compute(FrameVector output, FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector currentAngularVelocity,
         FrameVector feedForward)
   {
      filteredVelocity.update(currentAngularVelocity);
      filteredVelocity.getFrameTuple(filteredVelocityTemp);

      computeProportionalTerm(desiredOrientation);
      computeDerivativeTerm(desiredAngularVelocity, filteredVelocityTemp);
      computeIntegralTerm();
      output.setToZero(proportionalTerm.getReferenceFrame());
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

   private void computeProportionalTerm(FrameOrientation desiredOrientation)
   {
      visualizeDesiredAndActualOrientations(desiredOrientation);

      desiredOrientation.changeFrame(bodyFrame);
      desiredOrientation.getQuaternion(desiredQuaternion);
      desiredAngleAxis.set(desiredQuaternion);
      desiredAngleAxis.setAngle(AngleTools.trimAngleMinusPiToPi(desiredAngleAxis.getAngle()));
      orientationErrorMagnitude.set(desiredAngleAxis.getAngle());

      proportionalTerm.set(desiredAngleAxis.getX(), desiredAngleAxis.getY(), desiredAngleAxis.getZ());
      proportionalTerm.scale(desiredAngleAxis.getAngle());
      axisAngleErrorInBody.set(proportionalTerm);

      proportionalGainMatrix.transform(proportionalTerm.getVector());
   }

   private void computeDerivativeTerm(FrameVector desiredAngularVelocity, FrameVector currentAngularVelocity)
   {
      desiredAngularVelocity.changeFrame(bodyFrame);
      currentAngularVelocity.changeFrame(bodyFrame);

      derivativeTerm.sub(desiredAngularVelocity, currentAngularVelocity);
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

      double errorIntegratedX = desiredAngleAxis.getX() * desiredAngleAxis.getAngle() * dt;
      double errorIntegratedY = desiredAngleAxis.getY() * desiredAngleAxis.getAngle() * dt;
      double errorIntegratedZ = desiredAngleAxis.getZ() * desiredAngleAxis.getAngle() * dt;
      orientationErrorCumulated.add(errorIntegratedX, errorIntegratedY, errorIntegratedZ);

      double errorMagnitude = orientationErrorCumulated.length();
      if (errorMagnitude > gains.getMaximumIntegralError())
      {
         orientationErrorCumulated.scale(gains.getMaximumIntegralError() / errorMagnitude);
      }

      orientationErrorCumulated.getFrameTuple(integralTerm);
      integralGainMatrix.transform(integralTerm.getVector());
   }

   private void visualizeDesiredAndActualOrientations(FrameOrientation desiredOrientation)
   {
      if (visualize)
      {
         desiredOrientationViz.setAndMatchFrame(desiredOrientation);
         currentOrientationViz.setAndMatchFrame(currentOrientation);
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

   public void setMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      gains.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }

   public void setGains(YoOrientationPIDGains gains)
   {
      setProportionalGains(gains.getProportionalGains());
      setDerivativeGains(gains.getDerivativeGains());
      setIntegralGains(gains.getIntegralGains(), gains.getMaximumIntegralError());
      setMaxAccelerationAndJerk(gains.getMaximumAcceleration(), gains.getMaximumJerk());
   }
}
