package us.ihmc.robotics.geometry.transformables;

import javax.vecmath.Tuple2d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.interfaces.GeometryObject;

public class TransformableVector2d extends Vector2d implements GeometryObject<TransformableVector2d>
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
   public void applyTransform(RigidBodyTransform transform)
   {
      applyTransform(transform, true);
   }

   public void applyTransform(RigidBodyTransform transform, boolean requireTransformInPlane)
   {
      temporaryTransformedVector.set(this.getX(), this.getY(), 0.0);
      transform.transform(temporaryTransformedVector);

      if (requireTransformInPlane)
         checkIsTransformationInPlane(temporaryTransformedVector);

      this.set(temporaryTransformedVector.getX(), temporaryTransformedVector.getY());
   }

   private void checkIsTransformationInPlane(Vector3d transformedVector)
   {
      if (Math.abs(transformedVector.getZ()) > epsilon)
         throw new RuntimeException("Cannot transform FramePoint2d to a plane with a different surface normal");
   }
   
   @Override
   public void set(TransformableVector2d other)
   {
      super.set(other);
   }

   @Override
   public void setToZero()
   {
      super.set(0.0, 0.0);
   }

   @Override
   public void setToNaN()
   {
      super.set(Double.NaN, Double.NaN);
   }

   @Override
   public boolean containsNaN()
   {
      if (Double.isNaN(getX()))
         return true;
      if (Double.isNaN(getY()))
         return true;

      return false;
   }

   @Override
   public boolean epsilonEquals(TransformableVector2d other, double epsilon)
   {
      // Check one axis at a time for efficiency when false, but end up comparing distance in the end.

      double epsilonSquared = epsilon * epsilon;
      double xDiffSquared = (getX() - other.getX()) * (getX() - other.getX());
      if (xDiffSquared > epsilonSquared)
         return false;

      double yDiffSquared = (getY() - other.getY()) * (getY() - other.getY());
      if ((xDiffSquared + yDiffSquared) > epsilonSquared)
         return false;

      return true;
   }


}
