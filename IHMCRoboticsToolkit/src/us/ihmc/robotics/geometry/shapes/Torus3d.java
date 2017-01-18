package us.ihmc.robotics.geometry.shapes;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.math.Epsilons;

/**
 * The torus is built in the XY plane, around the z axis
 */
public class Torus3d extends Shape3d<Torus3d>
{
   private double radius;
   private double tubeRadius;

   private final Point3d temporaryPoint = new Point3d();
   private final Vector3d temporaryVector = new Vector3d();
   
   private final Vector3d originToRadiusTemporaryVector = new Vector3d();
   private final Vector3d tubeCenterToPointTemporaryVector = new Vector3d();

   private static final double SMALLEST_ALLOWABLE_THICKNESS = 5e-3;
   private static final double SMALLEST_ALLOWABLE_RADIUS_MINUS_THICKNESS = 1e-4;

   public Torus3d(Torus3d torus3d)
   {
      this(torus3d.getTransformUnsafe(), torus3d.radius, torus3d.tubeRadius);
   }
   
   public Torus3d()
   {
      radius = 1.0;
      tubeRadius = 0.1;
      
      checkRadiusAndThickness();
   }

   public Torus3d(double radius, double thickness)
   {
      this.radius = radius;
      this.tubeRadius = thickness;
      
      checkRadiusAndThickness(); 
   }

   public Torus3d(RigidBodyTransform transform, double radius, double thickness)
   {
      setTransform(transform);
      this.radius = radius;
      this.tubeRadius = thickness;
      
      checkRadiusAndThickness();
   }

   @Override
   public void set(Torus3d torus3d)
   {
      setTransform(torus3d.getTransformUnsafe());
      this.radius = torus3d.radius;
      this.tubeRadius = torus3d.tubeRadius;
   }
   
   private void checkRadiusAndThickness()
   {
      if (radius - tubeRadius < SMALLEST_ALLOWABLE_RADIUS_MINUS_THICKNESS)
         throw new RuntimeException("Torus3d: checkRadiusAndThickness(): " +
         		"Invalid dimensions: Difference between radius and thickness is too small. Enter new dimensions.");
      
      if (tubeRadius < SMALLEST_ALLOWABLE_THICKNESS)
         throw new RuntimeException("Torus3d: checkRadiusAndThickness(): " +
               "Invalid thickness: Torus is too thin. Enter new dimensions.");
   }

   public double getRadius()
   {
      return radius;
   }

   public void setRadius(double radius)
   {
      this.radius = radius;
   }

   public double getThickness()
   {
      return tubeRadius;
   }

   public void setThickness(double thickness)
   {
      this.tubeRadius = thickness;
   }

   @Override
   public boolean epsilonEquals(Torus3d other, double epsilon)
   {
      return MathTools.epsilonEquals(radius, other.radius, epsilon) && MathTools.epsilonEquals(tubeRadius, other.tubeRadius, epsilon);
   }

   @Override
   public void setToZero()
   {
      radius = 0.0;
      tubeRadius = 0.0;
   }

   @Override
   public void setToNaN()
   {
      radius = Double.NaN;
      tubeRadius = Double.NaN;
   }

   @Override
   public boolean containsNaN()
   {
      return Double.isNaN(radius) || Double.isNaN(tubeRadius);
   }

   @Override
   public String toString()
   {
      return "radius = " + radius + ", thickness = " + tubeRadius + ", transform = " + getTransformUnsafe() + "\n";
   }

   @Override
   protected boolean checkIfInsideShapeFrame(Point3d pointInWorldToCheck, Point3d closestPointToPack, Vector3d normalToPack)
   {
      surfaceNormalAt(normalToPack, pointInWorldToCheck);
      closestPointToPack.set(pointInWorldToCheck);
      orthogonalProjectionShapeFrame(closestPointToPack);

      return isInsideOrOnSurfaceShapeFrame(pointInWorldToCheck, Epsilons.ONE_TEN_MILLIONTH);
   }

   @Override
   protected double distanceShapeFrame(Point3d point)
   {
      temporaryPoint.set(point);
      orthogonalProjectionShapeFrame(temporaryPoint);

      return temporaryPoint.distance(point);
   }

   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(Point3d pointToCheck, double epsilon)
   {
      temporaryVector.set(pointToCheck.getX(), pointToCheck.getY(), 0.0);
      
      if (temporaryVector.length() < Epsilons.ONE_TRILLIONTH)
      {
         return tubeRadius >= radius;
      }
      
      temporaryVector.normalize();
      temporaryVector.scale(radius);
      
      temporaryPoint.set(temporaryVector);
      
      return temporaryPoint.distance(pointToCheck) <= tubeRadius + epsilon;
   }

   private void surfaceNormalAt(Vector3d normalToPack, Point3d pointToCheck)
   {
      computeCompositeVectorsForPoint(originToRadiusTemporaryVector, tubeCenterToPointTemporaryVector, pointToCheck);

      tubeCenterToPointTemporaryVector.normalize();
      normalToPack.set(tubeCenterToPointTemporaryVector);
   }

   @Override
   protected void orthogonalProjectionShapeFrame(Point3d pointToCheckAndPack)
   {
      computeCompositeVectorsForPoint(originToRadiusTemporaryVector, tubeCenterToPointTemporaryVector, pointToCheckAndPack);

      if (tubeCenterToPointTemporaryVector.length() > tubeRadius)
         tubeCenterToPointTemporaryVector.scale(tubeRadius / tubeCenterToPointTemporaryVector.length());
      tubeCenterToPointTemporaryVector.add(originToRadiusTemporaryVector);

      pointToCheckAndPack.set(tubeCenterToPointTemporaryVector);
   }

   /**
    * Compute the vector, R, from the center point of the torus to the radius of the torus in the XY direction of the given point.
    * Computes the vector from the closest point in the middle of the torus tube (at R) to the given point.
    * @param originToRadiusVectorToPack the vector from the center point of the torus to the radius of the torus in the XY direction of the given point. 
    * @param tubeCenterToPointVectorToPack the vector from the closest point in the middle of the torus tube (at R) to the given point.
    * @param pointToCheck the point in world coordinates for which to compute the origin-to-radius and tube-center-to-point vectors.
    */
   protected void computeCompositeVectorsForPoint(Vector3d originToRadiusVectorToPack, Vector3d tubeCenterToPointVectorToPack, Point3d pointToCheck)
   {
      temporaryPoint.set(pointToCheck);

      double pointX = temporaryPoint.getX(), pointY = temporaryPoint.getY(), pointZ = temporaryPoint.getZ();
      originToRadiusVectorToPack.set(pointX, pointY, 0.0);
      double distance = originToRadiusVectorToPack.length();

      if (distance == 0.0)
      {
         if (pointZ == 0.0)
         {
            originToRadiusVectorToPack.setX(radius - tubeRadius);
         }
         else
         {
            originToRadiusVectorToPack.setX(radius);
         }
         
         distance = originToRadiusVectorToPack.length();
      }
      
      originToRadiusVectorToPack.scale(radius / distance);

      temporaryPoint.sub(originToRadiusVectorToPack);
      
      tubeCenterToPointVectorToPack.set(temporaryPoint);
   }
}
