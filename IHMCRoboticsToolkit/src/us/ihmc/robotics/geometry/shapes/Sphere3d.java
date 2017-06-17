package us.ihmc.robotics.geometry.shapes;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.GeometryTools;

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

   @Override
   public void set(Sphere3d sphere3d)
   {
      setPose(sphere3d);
      radius = sphere3d.radius;
   }

   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(double x, double y, double z, double epsilon)
   {
      temporaryVector.set(x, y, z);
      return temporaryVector.length() <= radius + epsilon;
   }

   @Override
   protected double distanceShapeFrame(double x, double y, double z)
   {
      temporaryVector.set(x, y, z);
      return temporaryVector.length() - radius;
   }

   @Override
   protected void orthogonalProjectionShapeFrame(double x, double y, double z, Point3DBasics projectionToPack)
   {
      temporaryVector.set(x, y, z);

      double distance = temporaryVector.length();

      if (distance >= Epsilons.ONE_TRILLIONTH)
      {
         temporaryVector.normalize();
         temporaryVector.scale(radius);
         
         projectionToPack.set(temporaryVector);
      }
   }

   @Override
   protected boolean checkIfInsideShapeFrame(double x, double y, double z, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      boolean isInside = isInsideOrOnSurfaceShapeFrame(x, y, z, Epsilons.ONE_TRILLIONTH);

      surfaceNormalAt(x, y, z, normalToPack);

      temporaryVector.set(normalToPack);
      temporaryVector.scale(radius);

      closestPointToPack.set(temporaryVector);

      return isInside;
   }

   private void surfaceNormalAt(double x, double y, double z, Vector3DBasics normalToPack)
   {
      normalToPack.set(x, y, z);
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
