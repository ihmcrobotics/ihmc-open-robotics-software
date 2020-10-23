package us.ihmc.robotics.controllers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.yoVariables.registry.YoRegistry;

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
   private final FrameQuaternion desiredOrientation = new FrameQuaternion();
   private final FrameVector3D desiredAngularVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardAngularAction = new FrameVector3D();
   private final FrameVector3D currentAngularVelocity = new FrameVector3D();
   private final FrameVector3D angularActionFromOrientationController = new FrameVector3D();

   private final EuclideanPositionController positionController;
   private final FramePoint3D desiredPosition = new FramePoint3D();
   private final FrameVector3D desiredVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardLinearAction = new FrameVector3D();
   private final FrameVector3D currentVelocity = new FrameVector3D();
   private final FrameVector3D actionFromPositionController = new FrameVector3D();

   public SE3PIDController(String namePrefix, ReferenceFrame bodyFrame, double dt, YoRegistry registry)
   {
      this(namePrefix, bodyFrame, dt, null, registry);
   }

   public SE3PIDController(String namePrefix, ReferenceFrame bodyFrame, double dt, YoPIDSE3Gains gains, YoRegistry registry)
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
   public void compute(Twist twistToPack, FramePose3D desiredPose, TwistReadOnly desiredTwist)
   {
      checkBodyFrames(desiredTwist, twistToPack);
      checkBaseFrames(desiredTwist, twistToPack);
      checkExpressedInFrames(desiredTwist, twistToPack);

      twistToPack.setToZero(bodyFrame, desiredTwist.getBaseFrame(), bodyFrame);

      desiredOrientation.setIncludingFrame(desiredPose.getOrientation());
      desiredAngularVelocity.setIncludingFrame(desiredTwist.getAngularPart());
      feedForwardAngularAction.setIncludingFrame(desiredTwist.getAngularPart());
      orientationController.compute(angularActionFromOrientationController, desiredOrientation, desiredAngularVelocity, null, feedForwardAngularAction);
      twistToPack.getAngularPart().set(angularActionFromOrientationController);

      desiredPosition.setIncludingFrame(desiredPose.getPosition());
      desiredVelocity.setIncludingFrame(desiredTwist.getLinearPart());
      feedForwardLinearAction.setIncludingFrame(desiredTwist.getLinearPart());
      positionController.compute(actionFromPositionController, desiredPosition, desiredVelocity, null, feedForwardLinearAction);
      twistToPack.getLinearPart().set(actionFromPositionController);
   }

   /**
    * This method provides a spatial acceleration controller.
    * @param spatialAccelerationToPack spatial acceleration to return.
    * @param desiredPose desired pose that we want to achieve.
    * @param desiredTwist desired twist to damp around.
    * @param feedForwardAcceleration feed forward acceleration from a reference trajectory.
    * @param currentTwist current twist of the rigid body.
    */
   public void compute(SpatialAcceleration spatialAccelerationToPack, FramePose3D desiredPose, TwistReadOnly desiredTwist,
         SpatialAccelerationReadOnly feedForwardAcceleration, TwistReadOnly currentTwist)
   {
      checkBodyFrames(desiredTwist, feedForwardAcceleration, currentTwist);
      checkBaseFrames(desiredTwist, feedForwardAcceleration, currentTwist);
      checkExpressedInFrames(desiredTwist, feedForwardAcceleration, currentTwist);

      spatialAccelerationToPack.setToZero(bodyFrame, feedForwardAcceleration.getBaseFrame(), bodyFrame);

      desiredOrientation.setIncludingFrame(desiredPose.getOrientation());
      desiredAngularVelocity.setIncludingFrame(desiredTwist.getAngularPart());
      feedForwardAngularAction.setIncludingFrame(feedForwardAcceleration.getAngularPart());
      currentAngularVelocity.setIncludingFrame(currentTwist.getAngularPart());
      orientationController.compute(angularActionFromOrientationController, desiredOrientation, desiredAngularVelocity, currentAngularVelocity, feedForwardAngularAction);
      spatialAccelerationToPack.getAngularPart().set(angularActionFromOrientationController);

      desiredPosition.setIncludingFrame(desiredPose.getPosition());
      desiredVelocity.setIncludingFrame(desiredTwist.getLinearPart());
      feedForwardLinearAction.setIncludingFrame(feedForwardAcceleration.getLinearPart());
      currentVelocity.setIncludingFrame(currentTwist.getLinearPart());
      positionController.compute(actionFromPositionController, desiredPosition, desiredVelocity, currentVelocity, feedForwardLinearAction);
      spatialAccelerationToPack.getLinearPart().set(actionFromPositionController);
   }

   private void checkBodyFrames(TwistReadOnly desiredTwist, TwistReadOnly currentTwist)
   {
      desiredTwist.getBodyFrame().checkReferenceFrameMatch(bodyFrame);
      currentTwist.getBodyFrame().checkReferenceFrameMatch(bodyFrame);
   }

   private void checkBaseFrames(TwistReadOnly desiredTwist, TwistReadOnly currentTwist)
   {
      desiredTwist.getBaseFrame().checkReferenceFrameMatch(currentTwist.getBaseFrame());
   }

   private void checkExpressedInFrames(TwistReadOnly desiredTwist, TwistReadOnly currentTwist)
   {
      desiredTwist.getReferenceFrame().checkReferenceFrameMatch(bodyFrame);
      currentTwist.getReferenceFrame().checkReferenceFrameMatch(bodyFrame);
   }

   private void checkBodyFrames(TwistReadOnly desiredTwist, SpatialAccelerationReadOnly feedForwardAcceleration, TwistReadOnly currentTwist)
   {
      checkBodyFrames(desiredTwist, currentTwist);
      feedForwardAcceleration.getBodyFrame().checkReferenceFrameMatch(bodyFrame);
   }

   private void checkBaseFrames(TwistReadOnly desiredTwist, SpatialAccelerationReadOnly feedForwardAcceleration, TwistReadOnly currentTwist)
   {
      checkBaseFrames(desiredTwist, currentTwist);
      desiredTwist.getBaseFrame().checkReferenceFrameMatch(feedForwardAcceleration.getBaseFrame());
   }

   private void checkExpressedInFrames(TwistReadOnly desiredTwist, SpatialAccelerationReadOnly feedForwardAcceleration, TwistReadOnly currentTwist)
   {
      checkExpressedInFrames(desiredTwist, currentTwist);
      feedForwardAcceleration.getReferenceFrame().checkReferenceFrameMatch(bodyFrame);
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

   public void setGains(PIDSE3Gains gains)
   {
      positionController.setGains(gains.getPositionGains());
      orientationController.setGains(gains.getOrientationGains());
   }

   public void setPositionGains(PID3DGains gains)
   {
      positionController.setGains(gains);
   }

   public void setOrientationGains(PID3DGains gains)
   {
      orientationController.setGains(gains);
   }
}
