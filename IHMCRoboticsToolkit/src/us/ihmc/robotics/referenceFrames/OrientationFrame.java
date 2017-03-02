package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.FrameOrientation;

public class OrientationFrame extends ReferenceFrame
{
   private static final long serialVersionUID = 6465943952628127959L;
   private static int orientationNumber = 0;
   private final FrameOrientation frameOrientation;

   public OrientationFrame(FrameOrientation orientation)
   {
      super("Orientation_" + orientationNumber, orientation.getReferenceFrame(), false, false, false);

      orientationNumber++;

      frameOrientation = new FrameOrientation(orientation);
      this.update();
   }

   public void setOrientationAndUpdate(FrameOrientation orientation)
   {
      this.parentFrame.checkReferenceFrameMatch(orientation.getReferenceFrame());
      this.frameOrientation.set(orientation);
      this.update();
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      frameOrientation.getTransform3D(transformToParent);
   }
}
