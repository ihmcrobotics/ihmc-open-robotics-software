package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class OrientationFrame extends ReferenceFrame
{
   private static int orientationNumber = 0;
   private final FrameQuaternion frameOrientation;

   public OrientationFrame(FrameQuaternion orientation)
   {
      super("Orientation_" + orientationNumber, orientation.getReferenceFrame());

      orientationNumber++;

      frameOrientation = new FrameQuaternion(orientation);
      this.update();
   }

   public void setOrientationAndUpdate(FrameQuaternion orientation)
   {
      this.parentFrame.checkReferenceFrameMatch(orientation.getReferenceFrame());
      this.frameOrientation.set(orientation);
      this.update();
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.setRotation(frameOrientation);
   }
}
