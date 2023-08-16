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
public class ModifiableReferenceFrame
{
   /**
    * This reference is final, meaning even when this class's ReferenceFrame
    * is recreated, this is still the instance that represents the transformToParent.
    */
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();
   private final String frameName;
   private ReferenceFrame referenceFrame;

   public ModifiableReferenceFrame()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public ModifiableReferenceFrame(ReferenceFrame parentFrame)
   {
      this(ReferenceFrameMissingTools.computeFrameName(), parentFrame);
   }

   public ModifiableReferenceFrame(String frameName, ReferenceFrame parentFrame)
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
    * Warning! Frames that declared this one as the parent or
    * have this above them in the frame tree are going to be
    * broken after this change!
    *
    * Also note that this method will move the reference frame.
    * It doesn't update transformToParent.
    */
   public void changeParentFrame(ReferenceFrame parentFrame)
   {
      // I'm getting a rare bug here, need to leave this printing for a while so I can see which frame is having the issue. - @dcalvert
      if (EuclidCoreMissingTools.hasBeenRemoved(parentFrame))
      {
         LogTools.error("Parent frame has been removed! {}", parentFrame.getName());
      }

      referenceFrame.remove();
      referenceFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent(frameName, parentFrame, transformToParent);
   }

   /**
    * Warning! Frames that declared this one as the parent or
    * have this above them in the frame tree are going to be
    * broken after this change!
    *
    * This method will keep the frame in the same spot.
    */
   public void changeParentFrameWithoutMoving(ReferenceFrame parentFrame)
   {
      RigidBodyTransform newTransformToParent = new RigidBodyTransform();
      referenceFrame.getTransformToDesiredFrame(newTransformToParent, parentFrame);
      referenceFrame.remove();
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
