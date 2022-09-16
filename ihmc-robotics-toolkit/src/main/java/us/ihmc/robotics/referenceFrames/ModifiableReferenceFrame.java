package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class ModifiableReferenceFrame
{
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();
   private final ReferenceFrame referenceFrame;

   public ModifiableReferenceFrame(ReferenceFrame parentFrame)
   {
      referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentFrame, transformToParent);
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
