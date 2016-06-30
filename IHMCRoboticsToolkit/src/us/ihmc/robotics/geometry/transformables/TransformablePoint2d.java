package us.ihmc.robotics.geometry.transformables;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.interfaces.GeometryObject;

public class TransformablePoint2d extends Point2d implements GeometryObject<TransformablePoint2d>
{
   private static final long serialVersionUID = -7311591841945085693L;

   protected static final double epsilon = 1e-10;

   private final Point3d temporaryTransformedPoint = new Point3d();
   private final Vector3d temporaryTranslation = new Vector3d();

   public TransformablePoint2d(Tuple2d tuple)
   {
      super(tuple);
   }

   public TransformablePoint2d()
   {
      super();
   }

   public TransformablePoint2d(double x, double y)
   {
      super(x, y);
   }

   public TransformablePoint2d(double[] position)
   {
      super(position);
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      applyTransform(transform, true);
   }

   public void applyTransform(RigidBodyTransform transform, boolean requireTransformInXYPlane)
   {
      temporaryTransformedPoint.set(this.x, this.y, 0.0);
      transform.transform(temporaryTransformedPoint);

      if (requireTransformInXYPlane)
         checkIsTransformationInPlane(transform, temporaryTransformedPoint);

      this.set(temporaryTransformedPoint.x, temporaryTransformedPoint.y);
   }

   private void checkIsTransformationInPlane(RigidBodyTransform transformToNewFrame, Point3d transformedPoint)
   {
      transformToNewFrame.getTranslation(temporaryTranslation);
      if (Math.abs(temporaryTranslation.z - transformedPoint.z) > epsilon)
         throw new RuntimeException("Cannot transform FramePoint2d to a plane with a different surface normal");
   }
   
   @Override
   public void set(TransformablePoint2d other)
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
   public boolean epsilonEquals(TransformablePoint2d other, double epsilon)
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
