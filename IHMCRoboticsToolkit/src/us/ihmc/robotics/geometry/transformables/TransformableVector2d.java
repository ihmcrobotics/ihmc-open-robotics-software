package us.ihmc.robotics.geometry.transformables;

import javax.vecmath.Tuple2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class TransformableVector2d extends Vector2d implements TransformableDataObject
{
   private static final long serialVersionUID = -7311591841945085693L;

   protected static final double epsilon = 1e-10;

   private final Vector3d temporaryTransformedVector = new Vector3d();

   public TransformableVector2d(Tuple2d tuple)
   {
      super(tuple);
   }

   public TransformableVector2d()
   {
      super();
   }

   public TransformableVector2d(double x, double y)
   {
      super(x, y);
   }

   public TransformableVector2d(double[] vector)
   {
      super(vector);
   }

   @Override
   public void transform(RigidBodyTransform transform)
   {
      applyTransform(transform, true);
   }

   public void applyTransform(RigidBodyTransform transform, boolean requireTransformInPlane)
   {
      temporaryTransformedVector.set(this.x, this.y, 0.0);
      transform.transform(temporaryTransformedVector);

      if (requireTransformInPlane)
         checkIsTransformationInPlane(temporaryTransformedVector);

      this.set(temporaryTransformedVector.x, temporaryTransformedVector.y);
   }

   private void checkIsTransformationInPlane(Vector3d transformedVector)
   {
      if (Math.abs(transformedVector.z) > epsilon)
         throw new RuntimeException("Cannot transform FramePoint2d to a plane with a different surface normal");
   }

}
