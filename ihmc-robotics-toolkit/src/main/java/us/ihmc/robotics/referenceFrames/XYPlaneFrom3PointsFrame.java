package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class XYPlaneFrom3PointsFrame extends ReferenceFrame
{
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final FramePoint3D p1 = new FramePoint3D(world);
   private final FramePoint3D p2 = new FramePoint3D(world);
   private final FramePoint3D p3 = new FramePoint3D(world);

   private final RotationMatrix rotation = new RotationMatrix();
   private final Vector3D eX = new Vector3D();
   private final Vector3D eY = new Vector3D();
   private final Vector3D eZ = new Vector3D();

   public XYPlaneFrom3PointsFrame(ReferenceFrame parentFrame, String name)
   {
      super(name, parentFrame, false, true);
   }

   public void setPoints(FramePoint3D p1, FramePoint3D p2, FramePoint3D p3)
   {
      this.p1.setIncludingFrame(p1);
      this.p2.setIncludingFrame(p2);
      this.p3.setIncludingFrame(p3);

      this.p1.changeFrame(parentFrame);
      this.p2.changeFrame(parentFrame);
      this.p3.changeFrame(parentFrame);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      eX.sub(p2, p1);
      eX.normalize();
      eY.sub(p3, p1); // temp only
      eZ.cross(eX, eY);
      eZ.normalize();
      eY.cross(eZ, eX);

      rotation.setColumns(eX, eY, eZ);

      transformToParent.setRotationAndZeroTranslation(rotation);
      transformToParent.setTranslation(p1.getX(), p1.getY(), p1.getZ());
   }
}
