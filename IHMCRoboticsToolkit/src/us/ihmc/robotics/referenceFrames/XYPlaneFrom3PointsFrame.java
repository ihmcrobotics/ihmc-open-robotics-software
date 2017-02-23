package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;

public class XYPlaneFrom3PointsFrame extends ReferenceFrame
{
   private static final long serialVersionUID = 8615028266128433328L;

   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();

   private final FramePoint p1 = new FramePoint(world);
   private final FramePoint p2 = new FramePoint(world);
   private final FramePoint p3 = new FramePoint(world);

   private final RotationMatrix rotation = new RotationMatrix();
   private final Vector3D eX = new Vector3D();
   private final Vector3D eY = new Vector3D();
   private final Vector3D eZ = new Vector3D();

   public XYPlaneFrom3PointsFrame(ReferenceFrame parentFrame, String name)
   {
      super(name, parentFrame, false, false, true);
   }

   public void setPoints(FramePoint p1, FramePoint p2, FramePoint p3)
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
      eX.sub(p2.getPoint(), p1.getPoint());
      eX.normalize();
      eY.sub(p3.getPoint(), p1.getPoint()); // temp only
      eZ.cross(eX, eY);
      eZ.normalize();
      eY.cross(eZ, eX);

      rotation.setColumns(eX, eY, eZ);

      transformToParent.setRotationAndZeroTranslation(rotation);
      transformToParent.setTranslation(p1.getX(), p1.getY(), p1.getZ());
   }
}
