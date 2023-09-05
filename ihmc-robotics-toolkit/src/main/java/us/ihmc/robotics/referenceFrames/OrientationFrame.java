package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class OrientationFrame extends ReferenceFrame
{
   private static int orientationNumber = 0;
   private final FrameQuaternion frameOrientation;

   public OrientationFrame(FrameOrientation3DReadOnly orientation)
   {
      super("Orientation_" + orientationNumber, orientation.getReferenceFrame());

      orientationNumber++;

      frameOrientation = new FrameQuaternion(orientation);
      this.update();
   }

   public void setOrientationAndUpdate(FrameOrientation3DReadOnly orientation)
   {
      this.getParent().checkReferenceFrameMatch(orientation.getReferenceFrame());
      this.frameOrientation.set(orientation);
      this.update();
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.getRotation().set(frameOrientation);
   }
}
