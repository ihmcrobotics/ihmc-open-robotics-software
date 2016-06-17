package us.ihmc.robotics.geometry.transformables;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.interfaces.GeometryObject;

public class TransformableMatrix3d extends Matrix3d implements GeometryObject<TransformableMatrix3d>
{
   private static final long serialVersionUID = -4556753983259636892L;

   private final Matrix3d temporaryMatrix = new Matrix3d();

   public TransformableMatrix3d(Matrix3d matrix3d)
   {
      super(matrix3d);
   }

   public TransformableMatrix3d()
   {
      super();
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transform.getRotation(temporaryMatrix);
      this.mul(temporaryMatrix, this);
      this.mulTransposeRight(this, temporaryMatrix);
   }

   @Override
   public void setToZero()
   {
      this.setZero();
   }

   @Override
   public void set(TransformableMatrix3d other)
   {
      super.set(other);
   }

   @Override
   public void setToNaN()
   {
      super.setIdentity();
      super.mul(Double.NaN);
   }

   @Override
   public boolean containsNaN()
   {
      for (int i = 0; i<3; i++)
      {
         for (int j = 0; j<3; j++)
         {
            if (Double.isNaN(getElement(i,  j))) return true;
         }
      }

      return false;
   }

   @Override
   public boolean epsilonEquals(TransformableMatrix3d other, double epsilon)
   {      
      for (int i = 0; i<3; i++)
      {
         for (int j = 0; j<3; j++)
         {
            if (Math.abs(this.getElement(i, j) - other.getElement(i,  j)) > epsilon) return false;
         }
      }

      return true;
   }

}
