package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.robotics.controllers.OrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.PositionPIDGainsInterface;
import us.ihmc.robotics.controllers.SE3PIDController;
import us.ihmc.robotics.controllers.SE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class RigidBodySpatialAccelerationControlModule
{
   private final YoVariableRegistry registry;
   private final TwistCalculator twistCalculator;
   private final SE3PIDController se3pdController;
   private final SpatialAccelerationVector acceleration;
   private final RigidBody endEffector;
   private final ReferenceFrame endEffectorFrame;

   private final Twist currentTwist = new Twist();
   private final FramePose desiredEndEffectorPose = new FramePose();
   private final Twist desiredEndEffectorTwist = new Twist();
   private final SpatialAccelerationVector feedForwardEndEffectorSpatialAcceleration = new SpatialAccelerationVector();

   private final FramePoint endEffectorPosition = new FramePoint();
   private final FrameOrientation endEffectorOrientation = new FrameOrientation();

   public RigidBodySpatialAccelerationControlModule(String namePrefix, TwistCalculator twistCalculator, RigidBody endEffector, ReferenceFrame endEffectorFrame,
         double dt, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, twistCalculator, endEffector, endEffectorFrame, dt, null, parentRegistry);
   }

   public RigidBodySpatialAccelerationControlModule(String namePrefix, TwistCalculator twistCalculator, RigidBody endEffector, ReferenceFrame endEffectorFrame,
         double dt, YoSE3PIDGainsInterface taskspaceGains, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.twistCalculator = twistCalculator;
      this.endEffector = endEffector;
      this.endEffectorFrame = endEffectorFrame;
      this.se3pdController = new SE3PIDController(namePrefix, endEffectorFrame, dt, taskspaceGains, registry);
      this.acceleration = new SpatialAccelerationVector();

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      se3pdController.reset();
   }

   public void doPositionControl(FramePose desiredEndEffectorPose, Twist desiredEndEffectorTwist,
         SpatialAccelerationVector feedForwardEndEffectorSpatialAcceleration, RigidBody base)
   {
      twistCalculator.getRelativeTwist(base, endEffector, currentTwist);
      currentTwist.changeBodyFrameNoRelativeTwist(endEffectorFrame);
      currentTwist.changeFrame(endEffectorFrame);

      se3pdController.compute(acceleration, desiredEndEffectorPose, desiredEndEffectorTwist, feedForwardEndEffectorSpatialAcceleration, currentTwist);
   }

   public void doPositionControl(FramePoint desiredPosition, FrameOrientation desiredOrientation, FrameVector desiredLinearVelocityOfOrigin,
         FrameVector desiredAngularVelocity, FrameVector desiredLinearAccelerationOfOrigin, FrameVector desiredAngularAcceleration, RigidBody base)
   {
      convertToFramePose(desiredPosition, desiredOrientation, desiredEndEffectorPose);
      convertToTwist(desiredLinearVelocityOfOrigin, desiredAngularVelocity, base, desiredEndEffectorTwist);
      convertToSpatialAcceleration(desiredLinearAccelerationOfOrigin, desiredAngularAcceleration, base, feedForwardEndEffectorSpatialAcceleration);

      doPositionControl(desiredEndEffectorPose, desiredEndEffectorTwist, feedForwardEndEffectorSpatialAcceleration, base);
   }

   public void convertToFramePose(FramePoint endEffectorPositionIn, FrameOrientation endEffectorOrientationIn, FramePose poseToPack)
   {
      endEffectorPosition.setIncludingFrame(endEffectorPositionIn);
      endEffectorPosition.changeFrame(endEffectorFrame);

      endEffectorOrientation.setIncludingFrame(endEffectorOrientationIn);
      endEffectorOrientation.changeFrame(endEffectorFrame);

      poseToPack.setPoseIncludingFrame(endEffectorPosition, endEffectorOrientation);
   }

   public void convertToTwist(FrameVector linearVelocityOfOrigin, FrameVector angularVelocity, RigidBody base, Twist twistToPack)
   {
      angularVelocity.changeFrame(endEffectorFrame);
      linearVelocityOfOrigin.changeFrame(endEffectorFrame);

      twistToPack.set(endEffectorFrame, base.getBodyFixedFrame(), endEffectorFrame, linearVelocityOfOrigin.getVector(), angularVelocity.getVector());
   }

   private final Twist twistOfEndEffectorWithRespectToElevator = new Twist();

   public void convertToSpatialAcceleration(FrameVector linearAccelerationOfOrigin, FrameVector angularAcceleration, RigidBody base,
         SpatialAccelerationVector toPack)
   {
      angularAcceleration.changeFrame(endEffectorFrame);

      linearAccelerationOfOrigin.changeFrame(endEffectorFrame);
      twistCalculator.getRelativeTwist(base, endEffector, twistOfEndEffectorWithRespectToElevator);
      twistOfEndEffectorWithRespectToElevator.changeBodyFrameNoRelativeTwist(endEffectorFrame);

      ReferenceFrame baseFrame = base.getBodyFixedFrame();
      toPack.setBasedOnOriginAcceleration(endEffectorFrame, baseFrame, endEffectorFrame, angularAcceleration, linearAccelerationOfOrigin,
            twistOfEndEffectorWithRespectToElevator);
   }

   public void setPositionProportionalGains(double kpx, double kpy, double kpz)
   {
      se3pdController.setPositionProportionalGains(kpx, kpy, kpz);
   }

   public void setPositionDerivativeGains(double kdx, double kdy, double kdz)
   {
      se3pdController.setPositionDerivativeGains(kdx, kdy, kdz);
   }

   public void setPositionIntegralGains(double kix, double kiy, double kiz, double maxIntegralError)
   {
      se3pdController.setPositionIntegralGains(kix, kiy, kiz, maxIntegralError);
   }

   public void setOrientationProportionalGains(double kpx, double kpy, double kpz)
   {
      se3pdController.setOrientationProportionalGains(kpx, kpy, kpz);
   }

   public void setOrientationDerivativeGains(double kdx, double kdy, double kdz)
   {
      se3pdController.setOrientationDerivativeGains(kdx, kdy, kdz);
   }

   public void setOrientationIntegralGains(double kix, double kiy, double kiz, double maxIntegralError)
   {
      se3pdController.setOrientationIntegralGains(kix, kiy, kiz, maxIntegralError);
   }

   public void setGains(SE3PIDGainsInterface gains)
   {
      se3pdController.setGains(gains);
   }

   public void setPositionGains(PositionPIDGainsInterface gains)
   {
      se3pdController.setPositionGains(gains);
   }

   public void setOrientationGains(OrientationPIDGainsInterface gains)
   {
      se3pdController.setOrientationGains(gains);
   }

   public void setPositionMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      se3pdController.setPositionMaxFeedbackAndFeedbackRate(maxAcceleration, maxJerk);
   }

   public void setPositionMaxDerivativeError(double maxDerivativeError)
   {
      se3pdController.setPositionMaxDerivativeError(maxDerivativeError);
   }

   public void setPositionMaxProportionalError(double maxProportionalError)
   {
      se3pdController.setPositionMaxProportionalError(maxProportionalError);
   }

   public void setOrientationMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      se3pdController.setOrientationMaxFeedbackAndFeedbackRate(maxAcceleration, maxJerk);
   }

   public void setOrientationMaxDerivativeError(double maxDerivativeError)
   {
      se3pdController.setOrientationMaxDerivativeError(maxDerivativeError);
   }

   public void setOrientationMaxProportionalError(double maxProportionalError)
   {
      se3pdController.setOrientationMaxProportionalError(maxProportionalError);
   }

   public void getEndEffectorCurrentLinearVelocity(FrameVector endEffectorLinearVelocityToPack)
   {
      currentTwist.getBodyOriginLinearPartInBaseFrame(endEffectorLinearVelocityToPack);
   }

   public void getEndEffectorCurrentAngularVelocity(FrameVector endEffectorAngularVelocityToPack)
   {
      currentTwist.getAngularVelocityInBaseFrame(endEffectorAngularVelocityToPack);
   }

   public ReferenceFrame getTrackingFrame()
   {
      return endEffectorFrame;
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
   }

   public void getAcceleration(SpatialAccelerationVector accelerationToPack)
   {
      accelerationToPack.set(acceleration);
   }
}
