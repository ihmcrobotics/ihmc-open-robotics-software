package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver.SpatialAccelerationCommand;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameQuaternion;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class ChestOrientationControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFrameQuaternion yoDesiredQuaternion;
   private final YoFrameVector yoDesiredAngularVelocity;
   private final YoFrameVector yoFeedForwardAngularAcceleration;
   private final YoFrameOrientation yoDesiredOrientationInWorldViz;
   private final YoFrameQuaternion yoDesiredQuaternionInWorldViz;

   private final RigidBody chest;
   private final RigidBody elevator;
   private final RigidBodyOrientationControlModule orientationControlModule;
   private final SpatialAccelerationCommand spatialAccelerationCommand = new SpatialAccelerationCommand();

   public ChestOrientationControlModule(ReferenceFrame chestOrientationExpressedInFrame, RigidBody elevator, RigidBody chest, TwistCalculator twistCalculator, double controlDT,
         YoOrientationPIDGainsInterface gains, YoVariableRegistry parentRegistry)
   {
      this.chest = chest;
      this.elevator = elevator;
      this.yoDesiredQuaternion = new YoFrameQuaternion("desiredChestOrientation", chestOrientationExpressedInFrame, registry);
      this.yoDesiredOrientationInWorldViz = new YoFrameOrientation("desiredChestInWorld", ReferenceFrame.getWorldFrame(), registry);
      this.yoDesiredQuaternionInWorldViz = new YoFrameQuaternion("desiredChestInWorld", ReferenceFrame.getWorldFrame(), registry);
      this.yoDesiredAngularVelocity = new YoFrameVector("desiredChestAngularVelocity", chestOrientationExpressedInFrame, registry);
      this.yoFeedForwardAngularAcceleration = new YoFrameVector("desiredChestAngularAcceleration", chestOrientationExpressedInFrame, registry);

      orientationControlModule = new RigidBodyOrientationControlModule("chest", chest, twistCalculator, controlDT, gains, registry);

      spatialAccelerationCommand.set(elevator, chest);
   }

   public void setDesireds(FrameOrientation desiredOrientation, FrameVector desiredAngularVelocity, FrameVector feedForwardAngularAcceleration)
   {
      desiredOrientation.changeFrame(this.yoDesiredOrientationInWorldViz.getReferenceFrame());
      this.yoDesiredOrientationInWorldViz.set(desiredOrientation);
      this.yoDesiredQuaternionInWorldViz.set(desiredOrientation);

      desiredOrientation.changeFrame(this.yoDesiredQuaternion.getReferenceFrame());
      this.yoDesiredQuaternion.set(desiredOrientation);

      desiredAngularVelocity.changeFrame(this.yoDesiredAngularVelocity.getReferenceFrame());
      this.yoDesiredAngularVelocity.set(desiredAngularVelocity);

      feedForwardAngularAcceleration.changeFrame(this.yoFeedForwardAngularAcceleration.getReferenceFrame());
      this.yoFeedForwardAngularAcceleration.set(feedForwardAngularAcceleration);
   }

   private final FrameOrientation desiredOrientation = new FrameOrientation();
   private final FrameVector desiredAngularVelocity = new FrameVector();
   private final FrameVector feedForwardAngularAcceleration = new FrameVector();
   private final FrameVector controlledAngularAcceleration = new FrameVector();

   public void compute()
   {
      yoDesiredQuaternion.getFrameOrientationIncludingFrame(desiredOrientation);
      yoDesiredAngularVelocity.getFrameTupleIncludingFrame(desiredAngularVelocity);
      yoFeedForwardAngularAcceleration.getFrameTupleIncludingFrame(feedForwardAngularAcceleration);

      orientationControlModule.compute(controlledAngularAcceleration, desiredOrientation, desiredAngularVelocity, feedForwardAngularAcceleration, elevator);
      spatialAccelerationCommand.setAngularAcceleration(chest.getBodyFixedFrame(), elevator.getBodyFixedFrame(), controlledAngularAcceleration);
   }

   public SpatialAccelerationCommand getSpatialAccelerationCommand()
   {
      return spatialAccelerationCommand;
   }

   public RigidBody getChest()
   {
      return chest;
   }
}
