package us.ihmc.robotics.geometry.transformables;

import javax.vecmath.Point3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.interfaces.GeometryObject;
import us.ihmc.robotics.geometry.interfaces.PointInterface;

public class TransformablePoint3d extends Point3d implements GeometryObject<TransformablePoint3d>, PointInterface
{
   private static final long serialVersionUID = 3215925974643446454L;

   public TransformablePoint3d(Tuple3d tuple)
   {
      super(tuple);
   }

   public TransformablePoint3d(Tuple3f tuple)
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

      if (Double.isNaN(getX()) && !Double.isNaN(other.getX()))
         return false;
      if (Double.isNaN(getY()) && !Double.isNaN(other.getY()))
         return false;
      if (Double.isNaN(getZ()) && !Double.isNaN(other.getZ()))
         return false;
      
      if (!Double.isNaN(getX()) && Double.isNaN(other.getX()))
         return false;
      if (!Double.isNaN(getY()) && Double.isNaN(other.getY()))
         return false;
      if (!Double.isNaN(getZ()) && Double.isNaN(other.getZ()))
         return false;
      
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

   @Override
   public void getPoint(Point3d pointToPack)
   {
      this.get(pointToPack);
   }

   @Override
   public void setPoint(PointInterface pointInterface)
   {
      pointInterface.getPoint(this);
   }

   @Override
   public void setPoint(Point3d point)
   {
      this.set(point);
   }

   public double getLength()
   {
      return Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
   }

}
