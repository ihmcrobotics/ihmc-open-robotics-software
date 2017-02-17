package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;

public class OriginAndPointFrame extends ReferenceFrame
{
   private static final long serialVersionUID = -3583775854419464525L;
   private final FramePoint origin;
   private final FramePoint positionToPointAt;
   private final Vector3D xAxis = new Vector3D();
   private final Vector3D yAxis = new Vector3D();
   private final Vector3D zAxis = new Vector3D();
   private final RotationMatrix rotationMatrix = new RotationMatrix();
   private final Vector3D originVector = new Vector3D();

   public OriginAndPointFrame(String name, ReferenceFrame parentFrame)
   {
      super(name, parentFrame);
      this.origin = new FramePoint(parentFrame);
      this.positionToPointAt = new FramePoint(parentFrame);
   }

   public void setOriginAndPositionToPointAt(FramePoint origin, FramePoint positionToPointAt)
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
      positionToPointAt.get(xAxis);
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
