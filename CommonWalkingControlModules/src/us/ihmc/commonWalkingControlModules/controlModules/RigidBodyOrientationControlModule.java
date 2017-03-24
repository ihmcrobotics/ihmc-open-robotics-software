package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.robotics.controllers.AxisAngleOrientationController;
import us.ihmc.robotics.controllers.OrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
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
   private final TwistCalculator twistCalculator;
   private final Twist endEffectorTwist = new Twist();
   private final FrameVector currentAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

   public RigidBodyOrientationControlModule(String namePrefix, RigidBody endEffector, TwistCalculator twistCalculator, double dt, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, endEffector, twistCalculator, dt, null, parentRegistry);
   }

   public RigidBodyOrientationControlModule(String namePrefix, RigidBody endEffector, TwistCalculator twistCalculator, double dt, YoOrientationPIDGainsInterface gains,
         YoVariableRegistry parentRegistry)
   {
      this.endEffector = endEffector;
      this.axisAngleOrientationController = new AxisAngleOrientationController(namePrefix, endEffector.getBodyFixedFrame(), dt, gains, parentRegistry);
      this.twistCalculator = twistCalculator;
   }

   public void reset()
   {
      axisAngleOrientationController.reset();
   }

   public void compute(FrameVector outputToPack, FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity,
         FrameVector feedForwardAngularAcceleration, RigidBody base)
   {
      // using twists is a bit overkill; optimize if needed.
      twistCalculator.getRelativeTwist(base, endEffector, endEffectorTwist);
      currentAngularVelocity.setToZero(endEffectorTwist.getExpressedInFrame());
      endEffectorTwist.getAngularPart(currentAngularVelocity);

      desiredAngularVelocity.changeFrame(currentAngularVelocity.getReferenceFrame());

      feedForwardAngularAcceleration.changeFrame(endEffectorTwist.getExpressedInFrame());

      axisAngleOrientationController.compute(outputToPack, desiredOrientation, desiredAngularVelocity, currentAngularVelocity, feedForwardAngularAcceleration);
   }

   public void setGains(OrientationPIDGainsInterface gains)
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

   public void setMaxFeedbackAndFeedbackRate(double maxFeedback, double maxFeedbackRate)
   {
      axisAngleOrientationController.setMaxFeedbackAndFeedbackRate(maxFeedback, maxFeedbackRate);
   }

   public void setMaxDerivativeError(double maxDerivativeError)
   {
      axisAngleOrientationController.setMaxDerivativeError(maxDerivativeError);
   }

   public void setMaxProportionalError(double maxProportionalError)
   {
      axisAngleOrientationController.setMaxProportionalError(maxProportionalError);
   }

   public void getEndEffectorCurrentAngularVelocity(FrameVector angularVelocityToPack)
   {
      angularVelocityToPack.setIncludingFrame(currentAngularVelocity);
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
   }
}
