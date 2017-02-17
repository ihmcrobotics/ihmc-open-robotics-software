package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;

public class PointXAxisAtPositionFrame extends ReferenceFrame
{
   private static final long serialVersionUID = -3583775854419464525L;
   private final FramePoint positionToPointAt;
   private final Vector3D xAxis = new Vector3D();
   private final Vector3D yAxis = new Vector3D();
   private final Vector3D zAxis = new Vector3D(0.0, 0.0, 1.0);
   private final RotationMatrix rotationMatrix = new RotationMatrix();

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

      rotationMatrix.setColumns(xAxis, yAxis, zAxis);
      transformToParent.setRotation(rotationMatrix);
   }
}
