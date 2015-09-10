package us.ihmc.commonWalkingControlModules.controlModules;


import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.controllers.YoOrientationPIDGains;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFrameQuaternion;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class ChestOrientationControlModule extends DegenerateOrientationControlModule
{
   private final YoFrameQuaternion desiredQuaternion;
   private final YoFrameVector desiredAngularVelocity;
   private final YoFrameVector feedForwardAngularAcceleration;
   private final RigidBody chest;

   private final YoFrameOrientation desiredOrientationInWorldViz;
   private final YoFrameQuaternion desiredQuaternionInWorldViz;

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
      this.desiredQuaternion = new YoFrameQuaternion("desiredChestOrientation", chestOrientationExpressedInFrame, registry);
      this.desiredOrientationInWorldViz = new YoFrameOrientation("desiredChestInWorld", ReferenceFrame.getWorldFrame(), registry);
      this.desiredQuaternionInWorldViz = new YoFrameQuaternion("desiredChestInWorld", ReferenceFrame.getWorldFrame(), registry);
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
      orientationToPack.setToZero(desiredQuaternion.getReferenceFrame());
      desiredQuaternion.getFrameOrientationIncludingFrame(orientationToPack);
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
      desiredOrientation.changeFrame(this.desiredOrientationInWorldViz.getReferenceFrame());
      this.desiredOrientationInWorldViz.set(desiredOrientation);
      this.desiredQuaternionInWorldViz.set(desiredOrientation);

      desiredOrientation.changeFrame(this.desiredQuaternion.getReferenceFrame());
      this.desiredQuaternion.set(desiredOrientation);

      desiredAngularVelocity.changeFrame(this.desiredAngularVelocity.getReferenceFrame());
      this.desiredAngularVelocity.set(desiredAngularVelocity);

      feedForwardAngularAcceleration.changeFrame(this.feedForwardAngularAcceleration.getReferenceFrame());
      this.feedForwardAngularAcceleration.set(feedForwardAngularAcceleration);
   }
}
