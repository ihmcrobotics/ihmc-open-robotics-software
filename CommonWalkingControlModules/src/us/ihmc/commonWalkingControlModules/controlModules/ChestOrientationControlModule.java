package us.ihmc.commonWalkingControlModules.controlModules;


import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.controllers.YoOrientationPIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.math.frames.YoFrameQuaternion;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class ChestOrientationControlModule extends DegenerateOrientationControlModule
{
   private final YoFrameQuaternion desiredOrientation;
   private final YoFrameVector desiredAngularVelocity;
   private final YoFrameVector feedForwardAngularAcceleration;
   private final RigidBody chest;

   public ChestOrientationControlModule(ReferenceFrame chestOrientationExpressedInFrame, RigidBody chest, TwistCalculator twistCalculator, double controlDT,
         YoVariableRegistry parentRegistry)
   {
      this(chestOrientationExpressedInFrame, chest, twistCalculator, controlDT, null, parentRegistry);
   }

   public ChestOrientationControlModule(ReferenceFrame chestOrientationExpressedInFrame, RigidBody chest, TwistCalculator twistCalculator, double controlDT,
         YoOrientationPIDGains gains, YoVariableRegistry parentRegistry)
   {
      super("chest", new RigidBody[] {}, chest, new GeometricJacobian[] {}, twistCalculator, controlDT, gains, parentRegistry);

      this.chest = chest;
      this.desiredOrientation = new YoFrameQuaternion("desiredChestOrientation", chestOrientationExpressedInFrame, registry);
      this.desiredAngularVelocity = new YoFrameVector("desiredChestAngularVelocity", chestOrientationExpressedInFrame, registry);
      this.feedForwardAngularAcceleration = new YoFrameVector("desiredChestAngularAcceleration", chestOrientationExpressedInFrame, registry);
   }

   public RigidBody getChest()
   {
      return chest;
   }

   @Override
   protected void packDesiredFrameOrientation(FrameOrientation orientationToPack)
   {
      orientationToPack.setToZero(desiredOrientation.getReferenceFrame());
      desiredOrientation.getFrameOrientationIncludingFrame(orientationToPack);
   }

   @Override
   protected void packDesiredAngularVelocity(FrameVector angularVelocityToPack)
   {
      desiredAngularVelocity.getFrameTupleIncludingFrame(angularVelocityToPack);
   }

   @Override
   protected void packDesiredAngularAccelerationFeedForward(FrameVector angularAccelerationToPack)
   {
      feedForwardAngularAcceleration.getFrameTupleIncludingFrame(angularAccelerationToPack);
   }

   public void setDesireds(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector feedForwardAngularAcceleration)
   {
      desiredOrientation.changeFrame(this.desiredOrientation.getReferenceFrame());
      this.desiredOrientation.set(desiredOrientation);

      desiredAngularVelocity.changeFrame(this.desiredAngularVelocity.getReferenceFrame());
      this.desiredAngularVelocity.set(desiredAngularVelocity);

      feedForwardAngularAcceleration.changeFrame(this.feedForwardAngularAcceleration.getReferenceFrame());
      this.feedForwardAngularAcceleration.set(feedForwardAngularAcceleration);
   }
}
