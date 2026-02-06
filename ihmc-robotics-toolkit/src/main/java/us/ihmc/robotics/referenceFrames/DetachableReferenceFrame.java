package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

import java.util.function.Function;

/**
 * This class provides support for having a reference frame that
 * doesn't always have it's designated parent available in the world
 * frame tree. So we make that designated parent it's own root and
 * place this frame under it, until the designated parent is available
 * again in the world frame tree. This is handled automatically by
 * calling the updated method.
 */
public class DetachableReferenceFrame
{
   private transient final Function<String, ReferenceFrame> frameFunction;

   private final RigidBodyTransformReadOnly transformToParent;
   /** Never null, but does change. */
   private ReferenceFrame referenceFrame;

   public DetachableReferenceFrame(Function<String, ReferenceFrame> frameFunction, RigidBodyTransformReadOnly transformToParent)
   {
      this.transformToParent = transformToParent;
      this.frameFunction = frameFunction;
   }

   public void update(String parentFrameName)
   {
      ReferenceFrame parentFrameInWorld = frameFunction.apply(parentFrameName);

      boolean shouldBeChildOfWorld = parentFrameInWorld != null;

      boolean frameNeedsRecreating = referenceFrame == null;

      if (referenceFrame != null)
      {
         frameNeedsRecreating |= shouldBeChildOfWorld != isChildOfWorld();
         frameNeedsRecreating |= referenceFrame.getParent() != parentFrameInWorld;
      }

      if (frameNeedsRecreating)
      {
         ReferenceFrame parentFrame;
         if (shouldBeChildOfWorld) // Attached to world frame tree
         {
            parentFrame = parentFrameInWorld;
         }
         else // Detached under it's own root
         {
            parentFrame = ReferenceFrameTools.constructARootFrame(parentFrameName);
         }

         referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentFrame,
                                                                                                 transformToParent,
                                                                                                 DetachableReferenceFrame.class.getSimpleName());
      }
      else
      {
         referenceFrame.update(); // Neccessary?
      }
   }

   /**
    * Changes the parent frame of this reference frame while
    * keeping it in the same place w.r.t. a common ancestors.
    *
    * @param parentFrameName The new parent frame's name
    * @param transformToParent Should be the same instance as the read-only one passed in
    *                          the constructor.
    */
   public void changeFrame(String parentFrameName, RigidBodyTransform transformToParent)
   {
      if (referenceFrame != null)
      {
         ReferenceFrame parentFrame = frameFunction.apply(parentFrameName);
         RigidBodyTransform newTransformToParent = new RigidBodyTransform();

         if (parentFrame != null && referenceFrame.getRootFrame() == parentFrame.getRootFrame()) // Attached to world frame tree
         {
            referenceFrame.getTransformToDesiredFrame(newTransformToParent, parentFrame);
            transformToParent.set(newTransformToParent);
         }
         else // switch over to new root
         {
            parentFrame = ReferenceFrameTools.constructARootFrame(parentFrameName);
         }

         referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentFrame, transformToParent);
      }
   }

   public boolean isChildOfWorld()
   {
      return referenceFrame != null && referenceFrame.getRootFrame() == ReferenceFrame.getWorldFrame();
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}
