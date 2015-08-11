package us.ihmc.robotics.referenceFrames;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationFunctions;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

public class OriginAndPointFrame extends ReferenceFrame
{
   private static final long serialVersionUID = -3583775854419464525L;
   private final FramePoint origin;
   private final FramePoint positionToPointAt;
   private final Vector3d xAxis = new Vector3d();
   private final Vector3d yAxis = new Vector3d();
   private final Vector3d zAxis = new Vector3d();
   private final Matrix3d rotationMatrix = new Matrix3d();
   private final Vector3d originVector = new Vector3d();

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

      rotationMatrix.setColumn(0, xAxis);
      rotationMatrix.setColumn(1, yAxis);
      rotationMatrix.setColumn(2, zAxis);

      RotationFunctions.assertProper(rotationMatrix);

      transformToParent.setRotationAndZeroTranslation(rotationMatrix);
      transformToParent.setTranslation(originVector);
   }
}
