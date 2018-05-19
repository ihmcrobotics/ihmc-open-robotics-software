package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class PointXAxisAtPositionFrame extends ReferenceFrame
{
   private final FramePoint3D positionToPointAt;
   private final Vector3D xAxis = new Vector3D();
   private final Vector3D yAxis = new Vector3D();
   private final Vector3D zAxis = new Vector3D(0.0, 0.0, 1.0);
   private final RotationMatrix rotationMatrix = new RotationMatrix();

   public PointXAxisAtPositionFrame(String name, ReferenceFrame originFrame)
   {
      super(name, originFrame);
      this.positionToPointAt = new FramePoint3D(originFrame);
   }

   public void setPositionToPointAt(FramePoint3D positionToPointAt)
   {
      positionToPointAt.changeFrame(parentFrame);
      this.positionToPointAt.set(positionToPointAt);
   }
   
   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      xAxis.set(positionToPointAt);
      xAxis.setZ(0.0);
      xAxis.normalize();
      yAxis.cross(zAxis, xAxis);

      rotationMatrix.setColumns(xAxis, yAxis, zAxis);
      transformToParent.setRotation(rotationMatrix);
   }
}
