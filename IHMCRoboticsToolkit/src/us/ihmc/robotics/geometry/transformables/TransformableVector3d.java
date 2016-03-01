package us.ihmc.robotics.geometry.transformables;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.geometry.RigidBodyTransform;

public class TransformableVector3d extends Vector3d implements Transformable<TransformableVector3d>
{
   private static final long serialVersionUID = 3215925974643446454L;

   public TransformableVector3d(Tuple3d tuple)
   {
      super(tuple);
   }

   public TransformableVector3d()
   {
      super();
   }

   public TransformableVector3d(double x, double y, double z)
   {
      super(x, y, z);
   }

   public TransformableVector3d(double[] position)
   {
      super(position);
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      transform.transform(this);
   }

   @Override
   public void set(TransformableVector3d other)
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
   public boolean epsilonEquals(TransformableVector3d other, double epsilon)
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
