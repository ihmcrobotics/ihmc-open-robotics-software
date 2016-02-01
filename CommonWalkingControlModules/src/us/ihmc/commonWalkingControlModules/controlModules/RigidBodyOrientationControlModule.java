package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.robotics.controllers.AxisAngleOrientationController;
import us.ihmc.robotics.controllers.YoOrientationPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;


public class RigidBodyOrientationControlModule
{
   private final AxisAngleOrientationController axisAngleOrientationController;

   private final RigidBody endEffector;
   private final RigidBody base;
   private final TwistCalculator twistCalculator;
   private final Twist endEffectorTwist = new Twist();
   private final FrameVector currentAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

   public RigidBodyOrientationControlModule(String namePrefix, RigidBody base, RigidBody endEffector, TwistCalculator twistCalculator, double dt,
         YoVariableRegistry parentRegistry)
   {
      this(namePrefix, base, endEffector, twistCalculator, dt, null, false, parentRegistry);
   }

   public RigidBodyOrientationControlModule(String namePrefix, RigidBody base, RigidBody endEffector, TwistCalculator twistCalculator, double dt,
         YoOrientationPIDGains gains, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, base, endEffector, twistCalculator, dt, gains, false, parentRegistry);
   }

   public RigidBodyOrientationControlModule(String namePrefix, RigidBody base, RigidBody endEffector, TwistCalculator twistCalculator, double dt,
         YoOrientationPIDGains gains, boolean visualize, YoVariableRegistry parentRegistry)
   {
      this.base = base;
      this.endEffector = endEffector;
      this.axisAngleOrientationController = new AxisAngleOrientationController(namePrefix, endEffector.getBodyFixedFrame(), dt, gains, visualize, parentRegistry);
      this.twistCalculator = twistCalculator;
   }

   public void reset()
   {
      axisAngleOrientationController.reset();
   }

   public void compute(FrameVector outputToPack, FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector feedForwardAngularAcceleration)
   {
      // using twists is a bit overkill; optimize if needed.
      twistCalculator.packRelativeTwist(endEffectorTwist, base, endEffector);
      currentAngularVelocity.setToZero(endEffectorTwist.getExpressedInFrame());
      endEffectorTwist.packAngularPart(currentAngularVelocity);

      desiredAngularVelocity.changeFrame(currentAngularVelocity.getReferenceFrame());

      feedForwardAngularAcceleration.changeFrame(endEffectorTwist.getExpressedInFrame());

      axisAngleOrientationController.compute(outputToPack, desiredOrientation, desiredAngularVelocity, currentAngularVelocity, feedForwardAngularAcceleration);
   }

   public void setGains(YoOrientationPIDGains gains)
   {
      axisAngleOrientationController.setGains(gains);
   }

   public void setProportionalGains(double proportionalGainX, double proportionalGainY, double proportionalGainZ)
   {
      axisAngleOrientationController.setProportionalGains(proportionalGainX, proportionalGainY, proportionalGainZ);
   }

   public void setDerivativeGains(double derivativeGainX, double derivativeGainY, double derivativeGainZ)
   {
      axisAngleOrientationController.setDerivativeGains(derivativeGainX, derivativeGainY, derivativeGainZ);
   }

   public void setMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      axisAngleOrientationController.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }

   public RigidBody getBase()
   {
      return base;
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
   }
}
