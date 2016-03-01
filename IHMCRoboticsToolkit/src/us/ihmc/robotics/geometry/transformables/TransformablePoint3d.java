package us.ihmc.robotics.geometry.transformables;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class TransformablePoint3d extends Point3d implements TransformableDataObject<TransformablePoint3d>
{
   private static final long serialVersionUID = 3215925974643446454L;

   public TransformablePoint3d(Tuple3d tuple)
   {
      super(tuple);
   }

   public TransformablePoint3d()
   {
      super();
   }

   public TransformablePoint3d(double x, double y, double z)
   {
      super(x, y, z);
   }

   public TransformablePoint3d(double[] position)
   {
      super(position);
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transform.transform(this);
   }
   
   @Override
   public void set(TransformablePoint3d other)
   {
      super.set(other);
   }

   @Override
   public void setToZero()
   {
      super.set(0.0, 0.0, 0.0);
   }

   @Override
   public void setToNaN()
   {
      super.set(Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   public boolean containsNaN()
   {
      if (Double.isNaN(getX()))
         return true;
      if (Double.isNaN(getY()))
         return true;
      if (Double.isNaN(getZ()))
         return true;

      return false;
   }

   @Override
   public boolean epsilonEquals(TransformablePoint3d other, double epsilon)
   {
      // Check one axis at a time for efficiency when false, but end up comparing distance in the end.

      double epsilonSquared = epsilon * epsilon;
      double xDiffSquared = (getX() - other.getX()) * (getX() - other.getX());
      if (xDiffSquared > epsilonSquared)
         return false;

      double yDiffSquared = (getY() - other.getY()) * (getY() - other.getY());
      if ((xDiffSquared + yDiffSquared) > epsilonSquared)
         return false;

      double zDiffSquared = (getZ() - other.getZ()) * (getZ() - other.getZ());
      if ((xDiffSquared + yDiffSquared + zDiffSquared) > epsilonSquared)
         return false;

      return true;
   }

}
