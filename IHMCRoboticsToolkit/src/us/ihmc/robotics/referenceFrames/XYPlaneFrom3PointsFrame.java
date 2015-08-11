package us.ihmc.robotics.referenceFrames;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

public class XYPlaneFrom3PointsFrame extends ReferenceFrame
{
   private static final long serialVersionUID = 8615028266128433328L;
   
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   
   private final FramePoint p1 = new FramePoint(world);
   private final FramePoint p2 = new FramePoint(world);
   private final FramePoint p3 = new FramePoint(world);
   
   private final Matrix3d rotation = new Matrix3d();
   private final Vector3d eX = new Vector3d();
   private final Vector3d eY = new Vector3d();
   private final Vector3d eZ = new Vector3d();
   
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
   
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      eX.sub(p2.getPoint(), p1.getPoint());
      eX.normalize();
      eY.sub(p3.getPoint(), p1.getPoint()); // temp only
      eZ.cross(eX, eY);
      eZ.normalize();
      eY.cross(eZ, eX);
      
      rotation.setColumn(0, eX);
      rotation.setColumn(1, eY);
      rotation.setColumn(2, eZ);
      
      transformToParent.setRotationAndZeroTranslation(rotation);
      transformToParent.setTranslation(p1.getVectorCopy());
   }
}
