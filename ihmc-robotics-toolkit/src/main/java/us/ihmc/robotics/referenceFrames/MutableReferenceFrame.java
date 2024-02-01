package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.EuclidCoreMissingTools;

import java.util.function.Consumer;

/**
 * This class is for two reasons:
 * - Having a single field for a transform with a changing transform to parent.
 *   (Otherwise you need another field to store the transformToParent)
 * - Being able to change the parent frame of this reference frame, which
 *   is complicated and has constraints.
 */
public class MutableReferenceFrame
{
   /**
    * This reference is final, meaning even when this class's ReferenceFrame
    * is recreated, this is still the instance that represents the transformToParent.
    */
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();
   private final String frameName;
   private ReferenceFrame referenceFrame;

   public MutableReferenceFrame()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public MutableReferenceFrame(ReferenceFrame parentFrame)
   {
      this(ReferenceFrameMissingTools.computeFrameName(), parentFrame);
   }

   public MutableReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      this.frameName = frameName;
      referenceFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(frameName, parentFrame, transformToParent);
   }

   public void update(Consumer<RigidBodyTransform> transformToParentConsumer)
   {
      transformToParentConsumer.accept(transformToParent);
      referenceFrame.update();
   }

   /**
    * Set the parent frame without changing the transform to parent.
    *
    * Note that this method can cause the reference frame to move w.r.t
    * common ancestors because it doesn't update transformToParent.
    *
    * Warning! Frames that declared this one as the parent or
    * have this above them in the frame tree are going to be
    * broken after this change!
    */
   public void setParentFrame(ReferenceFrame parentFrame)
   {
      // I'm getting a rare bug here, need to leave this printing for a while so I can see which frame is having the issue. - @dcalvert
      if (EuclidCoreMissingTools.hasBeenRemoved(parentFrame))
      {
         LogTools.error("Parent frame has been removed! {}", parentFrame.getName());
      }

      // No reason to do this unless the parent is changing
      if (parentFrame != referenceFrame.getParent())
      {
         // We can't remove these because we need the getTransformToDesiredFrame to work later
         // for the "changeParentFrameWithoutMoving" functionality. Maybe the GC automatically
         // makes this a non-issue though.
         // referenceFrame.remove();
         referenceFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(frameName, parentFrame, transformToParent);
      }
   }

   /**
    * Change the frame that this reference frame is expressed in,
    * keeping it in the same place w.r.t common ancestors.
    *
    * This is called "changeFrame" to match the language of the rest
    * of the Euclid frame API, treating this ReferenceFrame kind of
    * like a FramePose3D.
    *
    * Warning! Frames that declared this one as the parent or
    * have this above them in the frame tree are going to be
    * broken after this change!
    */
   public void changeFrame(ReferenceFrame parentFrame)
   {
      RigidBodyTransform newTransformToParent = new RigidBodyTransform();
      referenceFrame.getTransformToDesiredFrame(newTransformToParent, parentFrame);

      // We can't remove these because we need the getTransformToDesiredFrame to work later
      // for the "changeParentFrameWithoutMoving" functionality. Maybe the GC automatically
      // makes this a non-issue though.
      // referenceFrame.remove();
      transformToParent.set(newTransformToParent);
      referenceFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(frameName, parentFrame, transformToParent);
   }

   public RigidBodyTransform getTransformToParent()
   {
      return transformToParent;
   }

   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }
}
