package us.ihmc.robotics.geometry.transformables;

import javax.vecmath.Matrix3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class TransformableMatrix3d extends Matrix3d implements TransformableDataObject
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
      transform.get(temporaryMatrix);
      this.mul(temporaryMatrix, this);
      this.mulTransposeRight(this, temporaryMatrix);
   }
}
