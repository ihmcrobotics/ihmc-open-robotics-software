package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.math.Epsilons;

public class Sphere3d extends Shape3d<Sphere3d>
{
   private double radius;
   
   private final Vector3D temporaryVector;

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
      
      temporaryVector = new Vector3D();
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

   public void getCenter(Point3DBasics centerToPack)
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
   protected boolean isInsideOrOnSurfaceShapeFrame(Point3DReadOnly pointToCheck, double epsilon)
   {
      temporaryVector.set(pointToCheck);
      return temporaryVector.length() <= radius + epsilon;
   }

   @Override
   protected double distanceShapeFrame(Point3DReadOnly point)
   {
      temporaryVector.set(point);
      return temporaryVector.length() - radius;
   }

   @Override
   protected void orthogonalProjectionShapeFrame(Point3DBasics point)
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
   protected boolean checkIfInsideShapeFrame(Point3DReadOnly pointInWorldToCheck, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      boolean isInside = isInsideOrOnSurfaceShapeFrame(pointInWorldToCheck, Epsilons.ONE_TRILLIONTH);

      surfaceNormalAt(pointInWorldToCheck, normalToPack);

      temporaryVector.set(normalToPack);
      temporaryVector.scale(radius);

      closestPointToPack.set(temporaryVector);

      return isInside;
   }

   private void surfaceNormalAt(Point3DReadOnly pointToCheck, Vector3DBasics normalToPack)
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
