package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.controllers.EuclideanPositionController;
import us.ihmc.yoUtilities.controllers.YoPositionPIDGains;


public class RigidBodyPositionControlModule
{
   private static final boolean VISUALIZE = true;

   private final YoVariableRegistry registry;
   private final TwistCalculator twistCalculator;
   private final Twist endEffectorTwist = new Twist();
   private final EuclideanPositionController euclideanPositionController;
   private final RigidBody endEffector;
   private final ReferenceFrame endEffectorFrame;
   private final FrameVector currentLinearVelocity = new FrameVector(ReferenceFrame.getWorldFrame());

   public RigidBodyPositionControlModule(String namePrefix, TwistCalculator twistCalculator, RigidBody endEffector, ReferenceFrame endEffectorFrame, double dt,
         YoVariableRegistry parentRegistry)
   {
      this(namePrefix, twistCalculator, endEffector, endEffectorFrame, dt, null, parentRegistry);
   }

   public RigidBodyPositionControlModule(String namePrefix, TwistCalculator twistCalculator, RigidBody endEffector, ReferenceFrame endEffectorFrame, double dt,
         YoPositionPIDGains gains, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.twistCalculator = twistCalculator;
      this.endEffector = endEffector;
      this.endEffectorFrame = endEffectorFrame;
      this.euclideanPositionController = new EuclideanPositionController(namePrefix, endEffectorFrame, dt, gains, VISUALIZE, registry);

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      euclideanPositionController.reset();
   }

   public RigidBody getEndEffector()
   {
      return endEffector;
   }

   public void doPositionControl(FrameVector outputToPack, FramePoint desiredPosition, FrameVector desiredLinearVelocityOfOrigin, FrameVector desiredLinearAccelerationOfOrigin, RigidBody base)
   {
      // using twists is a bit overkill; optimize if needed.
      twistCalculator.packRelativeTwist(endEffectorTwist, base, endEffector);
      currentLinearVelocity.setToZero(endEffectorTwist.getExpressedInFrame());
      endEffectorTwist.packLinearPart(currentLinearVelocity.getVector());

      euclideanPositionController.compute(outputToPack, desiredPosition, desiredLinearVelocityOfOrigin, currentLinearVelocity, desiredLinearAccelerationOfOrigin);
   }

   public void setProportionalGains(double kpx, double kpy, double kpz)
   {
      euclideanPositionController.setProportionalGains(kpx, kpy, kpz);
   }

   public void setDerivativeGains(double kdx, double kdy, double kdz)
   {
      euclideanPositionController.setDerivativeGains(kdx, kdy, kdz);
   }

   public void setIntegralGains(double kix, double kiy, double kiz, double maxIntegralError)
   {
      euclideanPositionController.setIntegralGains(kix, kiy, kiz, maxIntegralError);
   }

   public ReferenceFrame getTrackingFrame()
   {
      return endEffectorFrame;
   }

   public void setGains(YoPositionPIDGains gains)
   {
      euclideanPositionController.setGains(gains);
   }

   public void setPositionMaxAccelerationAndJerk(double maxAcceleration, double maxJerk)
   {
      euclideanPositionController.setMaxAccelerationAndJerk(maxAcceleration, maxJerk);
   }
}
