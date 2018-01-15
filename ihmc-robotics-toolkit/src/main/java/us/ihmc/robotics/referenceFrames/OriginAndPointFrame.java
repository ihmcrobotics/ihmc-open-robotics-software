package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class OriginAndPointFrame extends ReferenceFrame
{
   private final FramePoint3D origin;
   private final FramePoint3D positionToPointAt;
   private final Vector3D xAxis = new Vector3D();
   private final Vector3D yAxis = new Vector3D();
   private final Vector3D zAxis = new Vector3D();
   private final RotationMatrix rotationMatrix = new RotationMatrix();
   private final Vector3D originVector = new Vector3D();

   public OriginAndPointFrame(String name, ReferenceFrame parentFrame)
   {
      super(name, parentFrame);
      this.origin = new FramePoint3D(parentFrame);
      this.positionToPointAt = new FramePoint3D(parentFrame);
   }

   public void setOriginAndPositionToPointAt(FramePoint3D origin, FramePoint3D positionToPointAt)
   {
      origin.changeFrame(parentFrame);
      this.origin.set(origin);

      positionToPointAt.changeFrame(parentFrame);
      this.positionToPointAt.set(positionToPointAt);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      originVector.set(origin.getPoint());
      xAxis.set(positionToPointAt);
      xAxis.sub(originVector);
      double norm = xAxis.length();
      if (norm > 1e-12)
      {
         xAxis.scale(1.0 / norm);
      }
      else
      {
         xAxis.set(1.0, 0.0, 0.0);
      }
      zAxis.set(0.0, 0.0, 1.0);
      yAxis.cross(zAxis, xAxis);
      yAxis.normalize();
      zAxis.cross(xAxis, yAxis);

      rotationMatrix.setColumns(xAxis, yAxis, zAxis);

      transformToParent.setRotationAndZeroTranslation(rotationMatrix);
      transformToParent.setTranslation(originVector);
   }
}
