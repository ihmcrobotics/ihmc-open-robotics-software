package us.ihmc.robotics.controllers;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;

/**
 * Double-geodesic PD controller with feed forward for a left-invariant system (i.e. in body coordinates)
 * from: Bullo, Murray. Proportional Derivative (PD) Control on the Euclidean Group
 * @author Twan
 *
 */
public class SE3PIDController
{
   private final ReferenceFrame bodyFrame;

   private final AxisAngleOrientationController orientationController;
   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();
   private final FrameVector currentAngularVelocity = new FrameVector();
   private final FrameVector angularAccelerationFromOrientationController = new FrameVector();

   private final EuclideanPositionController positionController;
   private final FramePoint desiredPosition = new FramePoint();
   private final FrameVector desiredVelocity = new FrameVector();
   private final FrameVector feedForwardLinearAcceleration = new FrameVector();
   private final FrameVector currentVelocity = new FrameVector();
   private final FrameVector accelerationFromPositionController = new FrameVector();

   public SE3PIDController(String namePrefix, ReferenceFrame bodyFrame, boolean visualize, double dt, YoVariableRegistry registry)
   {
      this(namePrefix, bodyFrame, visualize, dt, null, registry);
   }

   public SE3PIDController(String namePrefix, ReferenceFrame bodyFrame, boolean visualize, double dt, YoSE3PIDGains gains, YoVariableRegistry registry)
   {
      this.bodyFrame = bodyFrame;
      if (gains != null)
      {
         orientationController = new AxisAngleOrientationController(namePrefix, bodyFrame, dt, gains.getOrientationGains(), visualize, registry);
         positionController = new EuclideanPositionController(namePrefix, bodyFrame, dt, gains.getPositionGains(), visualize, registry);
      }
      else
      {
         orientationController = new AxisAngleOrientationController(namePrefix, bodyFrame, dt, visualize, registry);
         positionController = new EuclideanPositionController(namePrefix, bodyFrame, dt, visualize, registry);
      }
   }

   public void reset()
   {
      orientationController.reset();
      positionController.reset();
   }

   public void compute(SpatialAccelerationVector spatialAccelerationToPack, FramePose desiredPose, Twist desiredTwist,
         SpatialAccelerationVector feedForwardAcceleration, Twist currentTwist)
   {
      checkBodyFrames(desiredTwist, feedForwardAcceleration, currentTwist);
      checkBaseFrames(desiredTwist, feedForwardAcceleration, currentTwist);
      checkExpressedInFrames(desiredTwist, feedForwardAcceleration, currentTwist);

      spatialAccelerationToPack.setToZero(bodyFrame, feedForwardAcceleration.getBaseFrame(), bodyFrame);

      desiredPose.getOrientationIncludingFrame(desiredOrientation);
      desiredTwist.packAngularPart(desiredAngularVelocity);
      feedForwardAcceleration.packAngularPart(feedForwardAngularAcceleration);
      currentTwist.packAngularPart(currentAngularVelocity);
      orientationController.compute(angularAccelerationFromOrientationController, desiredOrientation, desiredAngularVelocity, currentAngularVelocity, feedForwardAngularAcceleration);
      spatialAccelerationToPack.setAngularPart(angularAccelerationFromOrientationController.getVector());

      desiredPose.getPositionIncludingFrame(desiredPosition);
      desiredTwist.packLinearPart(desiredVelocity);
      feedForwardAcceleration.packLinearPart(feedForwardLinearAcceleration);
      currentTwist.packLinearPart(currentVelocity);
      positionController.compute(accelerationFromPositionController, desiredPosition, desiredVelocity, currentVelocity, feedForwardLinearAcceleration);
      spatialAccelerationToPack.setLinearPart(accelerationFromPositionController.getVector());
   }

   private void checkBodyFrames(Twist desiredTwist, SpatialAccelerationVector feedForwardAcceleration, Twist currentTwist)
   {
      desiredTwist.getBodyFrame().checkReferenceFrameMatch(bodyFrame);
      feedForwardAcceleration.getBodyFrame().checkReferenceFrameMatch(bodyFrame);
      currentTwist.getBodyFrame().checkReferenceFrameMatch(bodyFrame);
   }

   private void checkBaseFrames(Twist desiredTwist, SpatialAccelerationVector feedForwardAcceleration, Twist currentTwist)
   {
      desiredTwist.getBaseFrame().checkReferenceFrameMatch(feedForwardAcceleration.getBaseFrame());
      desiredTwist.getBaseFrame().checkReferenceFrameMatch(currentTwist.getBaseFrame());
   }

   private void checkExpressedInFrames(Twist desiredTwist, SpatialAccelerationVector feedForwardAcceleration, Twist currentTwist)
   {
      desiredTwist.getExpressedInFrame().checkReferenceFrameMatch(bodyFrame);
      feedForwardAcceleration.getExpressedInFrame().checkReferenceFrameMatch(bodyFrame);
      currentTwist.getExpressedInFrame().checkReferenceFrameMatch(bodyFrame);
   }

   public void setPositionProportionalGains(double kpx, double kpy, double kpz)
   {
      positionController.setProportionalGains(kpx, kpy, kpz);
   }

   public void setPositionDerivativeGains(double kdx, double kdy, double kdz)
   {
      positionController.setDerivativeGains(kdx, kdy, kdz);
   }

   public void setPositionIntegralGains(double kix, double kiy, double kiz, double maxIntegralError)
   {
      positionController.setIntegralGains(kix, kiy, kiz, maxIntegralError);
   }

   public void setOrientationProportionalGains(double kpx, double kpy, double kpz)
   {
      orientationController.setProportionalGains(kpx, kpy, kpz);
   }

   public void setOrientationDerivativeGains(double kdx, double kdy, double kdz)
   {
      orientationController.setDerivativeGains(kdx, kdy, kdz);
   }

   public void setOrientationIntegralGains(double kix, double kiy, double kiz, double maxIntegralError)
   {
      orientationController.setIntegralGains(kix, kiy, kiz, maxIntegralError);
   }

   public void setGains(SE3PIDGains gains)
   {
      positionController.setProportionalGains(gains.getPositionProportionalGains());
      positionController.setDerivativeGains(gains.getPositionDerivativeGains());
      positionController.setIntegralGains(gains.getPositionIntegralGains(), gains.getPositionMaxIntegralError());
      positionController.setMaxAccelerationAndJerk(gains.getPositionMaximumAcceleration(), gains.getPositionMaximumJerk());

      orientationController.setProportionalGains(gains.getOrientationProportionalGains());
      orientationController.setDerivativeGains(gains.getOrientationDerivativeGains());
      orientationController.setIntegralGains(gains.getOrientationIntegralGains(), gains.getOrientationMaxIntegralError());
      orientationController.setMaxAccelerationAndJerk(gains.getOrientationMaximumAcceleration(), gains.getOrientationMaximumJerk());
   }

   public void setGains(YoSE3PIDGains gains)
   {
      positionController.setGains(gains.getPositionGains());
      orientationController.setGains(gains.getOrientationGains());
   }

   public void setPositionGains(YoPositionPIDGains gains)
   {
      positionController.setGains(gains);
   }

   public void setOrientationGains(YoOrientationPIDGains gains)
   {
      orientationController.setGains(gains);
   }

   public void setPositionMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      positionController.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }

   public void setOrientationMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      orientationController.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }

   public void getPositionError(FrameVector positionErrorToPack)
   {
      positionController.getPositionError(positionErrorToPack);
   }

   //TODO: Implement this
   //   public void getOrientationError(FrameVector orientationErrorToPack)
   //   {
   //      orientationController.getOrientationError(orientationErrorToPack);
   //   }

}
