package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.controller.AxisAngleOrientationController;

public class RigidBodyOrientationControlModule
{
   private final AxisAngleOrientationController axisAngleOrientationController;
   
   private final RigidBody endEffector;
   private final RigidBody base;
   private final TwistCalculator twistCalculator;
   private final Twist endEffectorTwist = new Twist();
   private final FrameVector currentAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

   public RigidBodyOrientationControlModule(String namePrefix, RigidBody base, RigidBody endEffector, TwistCalculator twistCalculator, double dt, YoVariableRegistry parentRegistry)
   {
      this.base = base;
      this.endEffector = endEffector;
      this.axisAngleOrientationController = new AxisAngleOrientationController(namePrefix, endEffector.getBodyFixedFrame(), dt, parentRegistry);
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
      endEffectorTwist.packAngularPart(currentAngularVelocity.getVector());

      desiredAngularVelocity.changeFrame(currentAngularVelocity.getReferenceFrame());
      
      feedForwardAngularAcceleration.changeFrame(endEffectorTwist.getExpressedInFrame());

      axisAngleOrientationController.compute(outputToPack, desiredOrientation, desiredAngularVelocity, currentAngularVelocity, feedForwardAngularAcceleration);
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

