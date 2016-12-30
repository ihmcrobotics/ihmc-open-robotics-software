package us.ihmc.robotics.geometry.shapes;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.math.Epsilons;

public class Sphere3d extends Shape3d<Sphere3d>
{
   private double radius;
   
   private final Vector3d temporaryVector;

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

   public Sphere3d(Point3d center, double radius)
   {
      this(center.x, center.y, center.z, radius);
   }

   public Sphere3d(double x, double y, double z, double radius)
   {
      this.radius = radius;
      setPosition(x, y, z);
      
      temporaryVector = new Vector3d();
   }

   public double getRadius()
   {
      return radius;
   }
   
   public void setRadius(double radius)
   {
      this.radius = radius;
   }
   
   public void setCenter(double x, double y, double z)
   {
      setPosition(x, y, z);
   }

   public void getCenter(Point3d centerToPack)
   {
      getPosition(centerToPack);
   }

   public double getX()
   {
      return getTransformUnsafe().getM03();
   }

   public double getY()
   {
      return getTransformUnsafe().getM13();
   }

   public double getZ()
   {
      return getTransformUnsafe().getM23();
   }

   @Override
   public void set(Sphere3d sphere3d)
   {
      setTransform(sphere3d.getTransformUnsafe());
      radius = sphere3d.radius;
   }

   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(Point3d pointToCheck, double epsilon)
   {
      temporaryVector.set(pointToCheck);
      return temporaryVector.length() <= radius + epsilon;
   }

   @Override
   protected double distanceShapeFrame(Point3d point)
   {
      temporaryVector.set(point);
      return temporaryVector.length() - radius;
   }

   @Override
   protected void orthogonalProjectionShapeFrame(Point3d point)
   {
      temporaryVector.set(point);

      double distance = temporaryVector.length();

      if (distance >= Epsilons.ONE_TRILLIONTH)
      {
         temporaryVector.normalize();
         temporaryVector.scale(radius);
         
         point.set(temporaryVector);
      }
   }

   @Override
   protected boolean checkIfInsideShapeFrame(Point3d pointInWorldToCheck, Point3d closestPointToPack, Vector3d normalToPack)
   {
      boolean isInside = isInsideOrOnSurfaceShapeFrame(pointInWorldToCheck, Epsilons.ONE_TRILLIONTH);

      surfaceNormalAt(pointInWorldToCheck, normalToPack);

      temporaryVector.set(normalToPack);
      temporaryVector.scale(radius);

      closestPointToPack.set(temporaryVector);

      return isInside;
   }

   private void surfaceNormalAt(Point3d pointToCheck, Vector3d normalToPack)
   {
      normalToPack.set(pointToCheck);
      GeometryTools.normalizeSafelyZUp(normalToPack);
   }

   @Override
   public boolean epsilonEquals(Sphere3d other, double epsilon)
   {
      return MathTools.epsilonEquals(radius, other.radius, epsilon);
   }

   @Override
   public void setToZero()
   {
      radius = 0.0;
   }

   @Override
   public void setToNaN()
   {
      radius = Double.NaN;
   }

   @Override
   public boolean containsNaN()
   {
      return Double.isNaN(radius);
   }

   @Override
   public String toString()
   {
      return "radius = " + radius + "\n";
   }
}
