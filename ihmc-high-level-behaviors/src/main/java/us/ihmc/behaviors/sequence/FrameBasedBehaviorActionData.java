package us.ihmc.behaviors.sequence;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.referenceFrames.ReferenceFrameSupplier;

import javax.annotation.Nullable;

public abstract class FrameBasedBehaviorActionData implements BehaviorActionData, ReferenceFrameSupplier
{
   @Nullable
   private transient ReferenceFrameLibrary referenceFrameLibrary;
   @Nullable
   private transient ReferenceFrame referenceFrame;

   private String parentFrameName = ReferenceFrame.getWorldFrame().getName();
   private RigidBodyTransform transformToParent = new RigidBodyTransform();

   @Nullable
   public ReferenceFrameLibrary getReferenceFrameLibrary()
   {
      return referenceFrameLibrary;
   }

   @Override
   public void setReferenceFrameLibrary(@Nullable ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.referenceFrameLibrary = referenceFrameLibrary;
      if (referenceFrameLibrary != null)
         referenceFrameLibrary.add(this);
   }

   @Nullable
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   public String getParentFrameName()
   {
      return parentFrameName;
   }

   public void setParentFrameName(String parentFrameName)
   {
      this.parentFrameName = parentFrameName;
   }

   public RigidBodyTransform getTransformToParent()
   {
      return transformToParent;
   }

   public void setTransformToParent(RigidBodyTransform transformToParent)
   {
      this.transformToParent = transformToParent;
   }

   public void update()
   {
      if (referenceFrameLibrary != null)
      {
         ReferenceFrame parentFrame = referenceFrameLibrary.findFrameByName(parentFrameName).get();

         // Create the frame
         if (parentFrame != null && referenceFrame == null)
         {
            // TODO: changeme
            referenceFrame = new ReferenceFrame("", parentFrame)
            {
               @Override
               protected void updateTransformToParent(RigidBodyTransform transformToParent)
               {

               }
            };
         }

         // Remove the frame if the parent isn't in the reference frame library
         if (parentFrame == null && referenceFrame != null)
         {
            // Let the GC remove the reference frame since it has no parent
            referenceFrame = null;
         }

         // Check if the frame exists but the parent changed
         else if (referenceFrame != null && !referenceFrame.getParent().equals(parentFrame))
         {
            // TODO: changeme
            referenceFrame = new ReferenceFrame("", parentFrame)
            {
               @Override
               protected void updateTransformToParent(RigidBodyTransform transformToParent)
               {

               }
            };
         }
      }
   }

   @Override
   public ReferenceFrame get()
   {
      return referenceFrame;
   }
}
