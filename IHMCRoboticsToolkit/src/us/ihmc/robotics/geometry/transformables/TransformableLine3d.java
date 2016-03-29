package us.ihmc.robotics.geometry.transformables;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.interfaces.GeometryObject;

public class TransformableLine3d implements GeometryObject<TransformableLine3d>
{
   private final TransformablePoint3d origin = new TransformablePoint3d();
   private final TransformableVector3d direction = new TransformableVector3d();

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      origin.applyTransform(transform);
      direction.applyTransform(transform);
   }

   public TransformablePoint3d getOrigin()
   {
      return origin;
   }

   public TransformableVector3d getDirection()
   {
      return direction;
   }
   
   @Override
   public void set(TransformableLine3d other)
   {
      origin.set(other.origin);
      direction.set(other.direction);
   }

   @Override
   public void setToZero()
   {
      origin.set(0.0, 0.0, 0.0);
      direction.set(0.0, 0.0, 0.0);
   }

   @Override
   public void setToNaN()
   {
      origin.set(Double.NaN, Double.NaN, Double.NaN);
      direction.set(Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   public boolean containsNaN()
   {
      if (origin.containsNaN()) return true;
      if (direction.containsNaN()) return true;
      
      return false;
   }

   @Override
   public boolean epsilonEquals(TransformableLine3d other, double epsilon)
   {
      if (!origin.epsilonEquals(other.origin, epsilon)) return false;
      if (!direction.epsilonEquals(other.direction, epsilon)) return false;

      return true;
   }


}
