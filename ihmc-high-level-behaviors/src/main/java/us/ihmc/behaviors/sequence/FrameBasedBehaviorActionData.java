package us.ihmc.behaviors.sequence;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ConditionalReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.referenceFrames.ReferenceFrameSupplier;

public abstract class FrameBasedBehaviorActionData implements BehaviorActionDescription, ReferenceFrameSupplier
{
   private static final RigidBodyTransform ZERO_TRANSFORM_TO_PARENT = new RigidBodyTransform();

   private final ConditionalReferenceFrame conditionalReferenceFrame = new ConditionalReferenceFrame();
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();

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
      this.transformToParent.set(transformToParent);
   }

   @Override
   public ReferenceFrame get()
   {
      return conditionalReferenceFrame.get();
   }

   public void update(ReferenceFrameLibrary referenceFrameLibrary)
   {
      conditionalReferenceFrame.update(referenceFrameLibrary);
      if (conditionalReferenceFrame.hasParentFrame())
         conditionalReferenceFrame.getModifiableReferenceFrame().update(transformToParent -> transformToParent.set(this.transformToParent));
      else
         conditionalReferenceFrame.getModifiableReferenceFrame().update(transformToParent -> transformToParent.set(ZERO_TRANSFORM_TO_PARENT));
      update();
   }
}
