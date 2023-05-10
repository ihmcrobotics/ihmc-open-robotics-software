package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;

import java.util.function.Consumer;

public class ModifiableReferenceFrame
{
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();
   private final String frameName;
   private ReferenceFrame referenceFrame;

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
    */
   public void changeParentFrame(ReferenceFrame parentFrame)
   {
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
