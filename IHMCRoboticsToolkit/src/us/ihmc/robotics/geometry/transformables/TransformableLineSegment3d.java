package us.ihmc.robotics.geometry.transformables;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.interfaces.GeometryObject;

public class TransformableLineSegment3d implements GeometryObject<TransformableLineSegment3d>
{
   private final TransformablePoint3d startPoint = new TransformablePoint3d();
   private final TransformablePoint3d endPoint = new TransformablePoint3d();
   
   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      startPoint.applyTransform(transform);
      endPoint.applyTransform(transform);
   }
   
   public TransformablePoint3d getStartPoint()
   {
      return startPoint;
   }
   
   public TransformablePoint3d getEndPoint()
   {
      return endPoint;
   }

   @Override
   public void set(TransformableLineSegment3d other)
   {
      startPoint.set(other.startPoint);
      endPoint.set(other.endPoint);
   }

   @Override
   public void setToZero()
   {
      startPoint.set(0.0, 0.0, 0.0);
      endPoint.set(0.0, 0.0, 0.0);
   }

   @Override
   public void setToNaN()
   {
      startPoint.set(Double.NaN, Double.NaN, Double.NaN);
      endPoint.set(Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   public boolean containsNaN()
   {
      if (startPoint.containsNaN()) return true;
      if (endPoint.containsNaN()) return true;
      
      return false;
   }

   @Override
   public boolean epsilonEquals(TransformableLineSegment3d other, double epsilon)
   {
      if (!startPoint.epsilonEquals(other.startPoint, epsilon)) return false;
      if (!endPoint.epsilonEquals(other.endPoint, epsilon)) return false;

      return true;
   }
}
