package us.ihmc.behaviors.sequence;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ConditionalReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.referenceFrames.ReferenceFrameSupplier;

public abstract class FrameBasedBehaviorActionData implements BehaviorActionData, ReferenceFrameSupplier
{
   private final ConditionalReferenceFrame conditionalReferenceFrame = new ConditionalReferenceFrame();
   private RigidBodyTransform transformToParent = new RigidBodyTransform();

   public ConditionalReferenceFrame getConditionalReferenceFrame()
   {
      return conditionalReferenceFrame;
   }

   public RigidBodyTransform getTransformToParent()
   {
      return transformToParent;
   }

   public void setTransformToParent(RigidBodyTransform transformToParent)
   {
      this.transformToParent = transformToParent;
   }

   @Override
   public ReferenceFrame get()
   {
      return conditionalReferenceFrame.get();
   }

   public void update(ReferenceFrameLibrary referenceFrameLibrary)
   {
      conditionalReferenceFrame.update(referenceFrameLibrary);
      update();
   }
}
