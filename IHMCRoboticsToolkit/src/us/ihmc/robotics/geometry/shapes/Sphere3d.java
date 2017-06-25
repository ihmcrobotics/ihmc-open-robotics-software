package us.ihmc.robotics.geometry.shapes;

import static us.ihmc.euclid.tools.EuclidCoreTools.*;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.MathTools;

public class Sphere3d extends Shape3d<Sphere3d>
{
   private double radius;

   public Sphere3d()
   {
      this(0.0, 0.0, 0.0, 1.0);
   }

   public Sphere3d(double radius)
   {
      this(0.0, 0.0, 0.0, radius);
   }

   public Sphere3d(Sphere3d sphere3d)
   {
      this(sphere3d.getX(), sphere3d.getY(), sphere3d.getZ(), sphere3d.getRadius());
   }

   public Sphere3d(Point3DReadOnly center, double radius)
   {
      this(center.getX(), center.getY(), center.getZ(), radius);
   }

   public Sphere3d(double x, double y, double z, double radius)
   {
      this.radius = radius;
      setPosition(x, y, z);
   }

   public double getRadius()
   {
      return radius;
   }

   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   @Override
   public void set(Sphere3d sphere3d)
   {
      setPose(sphere3d);
      radius = sphere3d.radius;
   }

   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(double x, double y, double z, double epsilon)
   {
      double radiusWithEpsilon = radius + epsilon;
      return normSquared(x, y, z) <= radiusWithEpsilon * radiusWithEpsilon;
   }

   @Override
   protected double evaluateQuery(double x, double y, double z, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      double distance = Math.sqrt(EuclidCoreTools.normSquared(x, y, z));

      if (closestPointToPack != null)
      {
         if (distance > Epsilons.ONE_TRILLIONTH)
         {
            closestPointToPack.set(x, y, z);
            closestPointToPack.scale(radius / distance);
         }
         else
         {
            closestPointToPack.set(0.0, 0.0, radius);
         }
      }

      if (normalToPack != null)
      {
         if (distance > Epsilons.ONE_TRILLIONTH)
         {
            normalToPack.set(x, y, z);
            normalToPack.scale(1.0 / distance);
         }
         else
         {
            normalToPack.set(0.0, 0.0, 1.0);
         }
      }

      return distance - radius;
   }

   @Override
   public boolean epsilonEquals(Sphere3d other, double epsilon)
   {
      return MathTools.epsilonEquals(radius, other.radius, epsilon) && super.epsilonEqualsPose(other, epsilon);
   }

   @Override
   public void setToZero()
   {
      super.setToZero();
      radius = 0.0;
   }

   @Override
   public void setToNaN()
   {
      super.setToNaN();
      radius = Double.NaN;
   }

   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || Double.isNaN(radius);
   }

   @Override
   public String toString()
   {
      return "radius = " + radius + "\n";
   }
}
