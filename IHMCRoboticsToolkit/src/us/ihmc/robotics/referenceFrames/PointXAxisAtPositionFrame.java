package us.ihmc.robotics.referenceFrames;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

public class PointXAxisAtPositionFrame extends ReferenceFrame
{
   private static final long serialVersionUID = -3583775854419464525L;
   private final FramePoint positionToPointAt;
   private final Vector3d xAxis = new Vector3d();
   private final Vector3d yAxis = new Vector3d();
   private final Vector3d zAxis = new Vector3d(0.0, 0.0, 1.0);
   private final Matrix3d rotationMatrix = new Matrix3d();

   public PointXAxisAtPositionFrame(String name, ReferenceFrame originFrame)
   {
      super(name, originFrame);
      this.positionToPointAt = new FramePoint(originFrame);
   }

   public void setPositionToPointAt(FramePoint positionToPointAt)
   {
      positionToPointAt.changeFrame(parentFrame);
      this.positionToPointAt.set(positionToPointAt);
   }
   
   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      positionToPointAt.get(xAxis);
      xAxis.setZ(0.0);
      xAxis.normalize();
      yAxis.cross(zAxis, xAxis);

      rotationMatrix.setColumn(0, xAxis);
      rotationMatrix.setColumn(1, yAxis);
      rotationMatrix.setColumn(2, zAxis);
      transformToParent.setRotation(rotationMatrix);
   }
}
