package us.ihmc.robotics.controllers;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
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
   private final FrameVector feedForwardAngularAction = new FrameVector();
   private final FrameVector currentAngularVelocity = new FrameVector();
   private final FrameVector angularActionFromOrientationController = new FrameVector();

   private final EuclideanPositionController positionController;
   private final FramePoint desiredPosition = new FramePoint();
   private final FrameVector desiredVelocity = new FrameVector();
   private final FrameVector feedForwardLinearAction = new FrameVector();
   private final FrameVector currentVelocity = new FrameVector();
   private final FrameVector actionFromPositionController = new FrameVector();

   public SE3PIDController(String namePrefix, ReferenceFrame bodyFrame, double dt, YoVariableRegistry registry)
   {
      this(namePrefix, bodyFrame, dt, null, registry);
   }

   public SE3PIDController(String namePrefix, ReferenceFrame bodyFrame, double dt, YoSE3PIDGainsInterface gains, YoVariableRegistry registry)
   {
      this.bodyFrame = bodyFrame;
      if (gains != null)
      {
         orientationController = new AxisAngleOrientationController(namePrefix, bodyFrame, dt, gains.getOrientationGains(), registry);
         positionController = new EuclideanPositionController(namePrefix, bodyFrame, dt, gains.getPositionGains(), registry);
      }
      else
      {
         orientationController = new AxisAngleOrientationController(namePrefix, bodyFrame, dt, registry);
         positionController = new EuclideanPositionController(namePrefix, bodyFrame, dt, registry);
      }
   }

   public void reset()
   {
      orientationController.reset();
      positionController.reset();
   }

   public void resetIntegrator()
   {
      orientationController.resetIntegrator();
      positionController.resetIntegrator();
   }

   /**
    * This method provides a twist feedback controller, intended for spatial velocity control.
    * @param twistToPack twist to return
    * @param desiredPose desired pose that we want to achieve.
    * @param desiredTwist feed forward twist from a reference trajectory
    */
   public void compute(Twist twistToPack, FramePose desiredPose, Twist desiredTwist)
   {
      checkBodyFrames(desiredTwist, twistToPack);
      checkBaseFrames(desiredTwist, twistToPack);
      checkExpressedInFrames(desiredTwist, twistToPack);

      twistToPack.setToZero(bodyFrame, desiredTwist.getBaseFrame(), bodyFrame);

      desiredPose.getOrientationIncludingFrame(desiredOrientation);
      desiredTwist.getAngularPart(desiredAngularVelocity);
      desiredTwist.getAngularPart(feedForwardAngularAction);
      orientationController.compute(angularActionFromOrientationController, desiredOrientation, desiredAngularVelocity, null, feedForwardAngularAction);
      twistToPack.setAngularPart(angularActionFromOrientationController.getVector());

      desiredPose.getPositionIncludingFrame(desiredPosition);
      desiredTwist.getLinearPart(desiredVelocity);
      desiredTwist.getLinearPart(feedForwardLinearAction);
      positionController.compute(actionFromPositionController, desiredPosition, desiredVelocity, null, feedForwardLinearAction);
      twistToPack.setLinearPart(actionFromPositionController.getVector());
   }

   /**
    * This method provides a spatial acceleration controller.
    * @param spatialAccelerationToPack spatial acceleration to return.
    * @param desiredPose desired pose that we want to achieve.
    * @param desiredTwist desired twist to damp around.
    * @param feedForwardAcceleration feed forward acceleration from a reference trajectory.
    * @param currentTwist current twist of the rigid body.
    */
   public void compute(SpatialAccelerationVector spatialAccelerationToPack, FramePose desiredPose, Twist desiredTwist,
         SpatialAccelerationVector feedForwardAcceleration, Twist currentTwist)
   {
      checkBodyFrames(desiredTwist, feedForwardAcceleration, currentTwist);
      checkBaseFrames(desiredTwist, feedForwardAcceleration, currentTwist);
      checkExpressedInFrames(desiredTwist, feedForwardAcceleration, currentTwist);

      spatialAccelerationToPack.setToZero(bodyFrame, feedForwardAcceleration.getBaseFrame(), bodyFrame);

      desiredPose.getOrientationIncludingFrame(desiredOrientation);
      desiredTwist.getAngularPart(desiredAngularVelocity);
      feedForwardAcceleration.getAngularPart(feedForwardAngularAction);
      currentTwist.getAngularPart(currentAngularVelocity);
      orientationController.compute(angularActionFromOrientationController, desiredOrientation, desiredAngularVelocity, currentAngularVelocity, feedForwardAngularAction);
      spatialAccelerationToPack.setAngularPart(angularActionFromOrientationController.getVector());

      desiredPose.getPositionIncludingFrame(desiredPosition);
      desiredTwist.getLinearPart(desiredVelocity);
      feedForwardAcceleration.getLinearPart(feedForwardLinearAction);
      currentTwist.getLinearPart(currentVelocity);
      positionController.compute(actionFromPositionController, desiredPosition, desiredVelocity, currentVelocity, feedForwardLinearAction);
      spatialAccelerationToPack.setLinearPart(actionFromPositionController.getVector());
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

   private void checkBodyFrames(Twist desiredTwist, SpatialAccelerationVector feedForwardAcceleration, Twist currentTwist)
   {
      checkBodyFrames(desiredTwist, currentTwist);
      feedForwardAcceleration.getBodyFrame().checkReferenceFrameMatch(bodyFrame);
   }

   private void checkBaseFrames(Twist desiredTwist, SpatialAccelerationVector feedForwardAcceleration, Twist currentTwist)
   {
      checkBaseFrames(desiredTwist, currentTwist);
      desiredTwist.getBaseFrame().checkReferenceFrameMatch(feedForwardAcceleration.getBaseFrame());
   }

   private void checkExpressedInFrames(Twist desiredTwist, SpatialAccelerationVector feedForwardAcceleration, Twist currentTwist)
   {
      checkExpressedInFrames(desiredTwist, currentTwist);
      feedForwardAcceleration.getExpressedInFrame().checkReferenceFrameMatch(bodyFrame);
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

   public void setPositionMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      positionController.setMaxFeedbackAndFeedbackRate(maxFeedback, maxFeedbackRate);
   }

   public void setPositionMaxDerivativeError(double maxDerivativeError)
   {
      positionController.setMaxDerivativeError(maxDerivativeError);
   }

   public void setPositionMaxProportionalError(double maxProportionalError)
   {
      positionController.setMaxProportionalError(maxProportionalError);
   }

   public void setOrientationMaxFeedbackAndFeedbackRate(double maxAcceleration, double maxFeedbackRate)
   {
      orientationController.setMaxFeedbackAndFeedbackRate(maxAcceleration, maxFeedbackRate);
   }

   public void setOrientationMaxDerivativeError(double maxDerivativeError)
   {
      orientationController.setMaxDerivativeError(maxDerivativeError);
   }

   public void setOrientationMaxProportionalError(double maxProportionalError)
   {
      orientationController.setMaxProportionalError(maxProportionalError);
   }

   public void setGains(SE3PIDGainsInterface gains)
   {
      positionController.setGains(gains.getPositionGains());
      orientationController.setGains(gains.getOrientationGains());
   }

   public void setPositionGains(PositionPIDGainsInterface gains)
   {
      positionController.setGains(gains);
   }

   public void setOrientationGains(OrientationPIDGainsInterface gains)
   {
      orientationController.setGains(gains);
   }
}
