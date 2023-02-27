package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;

import java.util.function.Consumer;

public class ModifiableReferenceFrame
{
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();
   private final ReferenceFrame referenceFrame;

   public ModifiableReferenceFrame(ReferenceFrame parentFrame)
   {
      referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(parentFrame, transformToParent);
   }

   public void update(Consumer<RigidBodyTransform> transformToParentConsumer)
   {
      transformToParentConsumer.accept(transformToParent);
      referenceFrame.update();
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
