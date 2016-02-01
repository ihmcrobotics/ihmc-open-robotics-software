package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.RigidBodyTransform;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class Torus3d implements Shape3d
{
   private static final double DEFAULT_EPSILON = 1e-7;
   private RigidBodyTransform transform = new RigidBodyTransform();
   private double radius;
   private double thickness;

   private final RigidBodyTransform temporaryTransform = new RigidBodyTransform();
   private final Point3d temporaryPoint = new Point3d();
   
   private final Vector3d originToRadiusTemporaryVector = new Vector3d();
   private final Vector3d tubeCenterToPointTemporaryVector = new Vector3d();

   private static double SMALLEST_ALLOWABLE_THICKNESS = 5e-3;
   private static double SMALLEST_ALLOWABLE_RADIUS_MINUS_THICKNESS = 1e-4;

   public Torus3d(double radius, double thickness)
   {
      this(new RigidBodyTransform(), radius, thickness);
   }

   public Torus3d(RigidBodyTransform transform, double radius, double thickness)
   {
      checkRadiusAndThickness(radius, thickness);
      
      this.transform.set(transform);
      this.radius = radius;
      this.thickness = thickness;
      
   }

   public Torus3d(Torus3d torus3d)
   {
      this(torus3d.transform, torus3d.radius, torus3d.thickness);
   }

   public void set(Torus3d torus3d)
   {
      this.transform.set(transform);
      this.radius = torus3d.radius;
      this.thickness = torus3d.thickness;
   }
   
   private void checkRadiusAndThickness(double radius, double thickness)
   {
      if (radius - thickness < SMALLEST_ALLOWABLE_RADIUS_MINUS_THICKNESS)
         throw new RuntimeException("Torus3d: checkRadiusAndThickness(): " +
         		"Invalid dimensions: Difference between radius and thickness is too small. Enter new dimensions.");
      
      if (thickness < SMALLEST_ALLOWABLE_THICKNESS)
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
      return thickness;
   }

   public void setThickness(double thickness)
   {
      this.thickness = thickness;
   }

   public RigidBodyTransform getTransform()
   {
      return transform;
   }

   public void setTransform(RigidBodyTransform newTransform)
   {
      transform = new RigidBodyTransform(newTransform);
   }

   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   public void applyTransform(RigidBodyTransform transform)
   {
      tempTransform.set(transform);
      tempTransform.multiply(this.transform);
      this.transform.set(tempTransform);
   }

   @Override
   public String toString()
   {
      return "radius = " + radius + ", thickness = " + thickness + ", transform = " + transform + "\n";
   }

   public boolean checkIfInside(Point3d pointInWorldToCheck, Point3d closestPointToPack, Vector3d normalToPack)
   {
      surfaceNormalAt(normalToPack, pointInWorldToCheck);
      closestPointToPack.set(pointInWorldToCheck);
      orthogonalProjection(closestPointToPack);

      return isInsideOrOnSurface(pointInWorldToCheck, DEFAULT_EPSILON);
//      throw new RuntimeException("Implement me!");
   }

   public double distance(Point3d point)
   {
      temporaryPoint.set(point);
      orthogonalProjection(temporaryPoint);

      return temporaryPoint.distance(point);
   }

   public boolean isInsideOrOnSurface(Point3d pointToCheck)
   {
      return isInsideOrOnSurface(pointToCheck, DEFAULT_EPSILON);
   }

   public boolean isInsideOrOnSurface(Point3d pointToCheck, double epsilon)
   {
      // 0. given point to test P (x,y,z)
      // 1. Transform input point into a torus fixed frame. Use transform3d.transform(pointToCheck)
      // 2. Make projection Pxy = (x,y,0)
      // 3. Find vector CP from center of torus C to projection Pxy
      // 4. Find vector CPr by scaling CP to length of toroid radius-thickness
      // 5. Find intersection point on torus inner circle Pic at the end of CPr using C + CPr = Pic
      // 6. Find vector N from Pic to the original test point P using N = P - Pic
      // 7. If ||N||^2 < thickness^2, then inside
      // 8. Surface normal = N - normalized

      computeCompositeVectorsForPoint(originToRadiusTemporaryVector, tubeCenterToPointTemporaryVector, pointToCheck);
      double lengthSquared = tubeCenterToPointTemporaryVector.lengthSquared();

      return lengthSquared < (thickness * thickness + epsilon);
   }

   private void surfaceNormalAt(Vector3d normalToPack, Point3d pointToCheck)
   {
      computeCompositeVectorsForPoint(originToRadiusTemporaryVector, tubeCenterToPointTemporaryVector, pointToCheck);

      tubeCenterToPointTemporaryVector.normalize();
      transform.transform(tubeCenterToPointTemporaryVector);
      normalToPack.set(tubeCenterToPointTemporaryVector);
   }

   public void orthogonalProjection(Point3d pointToCheckAndPack)
   {
      computeCompositeVectorsForPoint(originToRadiusTemporaryVector, tubeCenterToPointTemporaryVector, pointToCheckAndPack);

      if (tubeCenterToPointTemporaryVector.length() > thickness)
         tubeCenterToPointTemporaryVector.scale(thickness / tubeCenterToPointTemporaryVector.length());
      tubeCenterToPointTemporaryVector.add(originToRadiusTemporaryVector);

      transform.transform(tubeCenterToPointTemporaryVector);
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

      temporaryTransform.set(transform);
      temporaryTransform.invert();
      temporaryTransform.transform(temporaryPoint);

      double pointX = temporaryPoint.getX(), pointY = temporaryPoint.getY(), pointZ = temporaryPoint.getZ();
      originToRadiusVectorToPack.set(pointX, pointY, 0.0);
      double distance = originToRadiusVectorToPack.length();

      if (distance == 0.0)
      {
         if (pointZ == 0.0)
            originToRadiusVectorToPack.setX(radius-thickness);
         else
            originToRadiusVectorToPack.setX(radius);
         
         distance = originToRadiusVectorToPack.length();
      }
      
      originToRadiusVectorToPack.scale(radius / distance);

      temporaryPoint.sub(originToRadiusVectorToPack);
      
      tubeCenterToPointVectorToPack.set(temporaryPoint);
   }

}
