package us.ihmc.robotics.geometry.shapes;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.transformables.Pose;

/**
 * The torus is built in the XY plane, around the z axis
 */
public class Torus3d extends Shape3d<Torus3d>
{
   private double radius;
   private double tubeRadius;

   private final Point3D temporaryPoint = new Point3D();
   private final Vector3D temporaryVector = new Vector3D();
   
   private final Vector3D originToRadiusTemporaryVector = new Vector3D();
   private final Vector3D tubeCenterToPointTemporaryVector = new Vector3D();

   private static final double SMALLEST_ALLOWABLE_THICKNESS = 5e-3;
   private static final double SMALLEST_ALLOWABLE_RADIUS_MINUS_THICKNESS = 1e-4;

   public Torus3d(Torus3d torus3d)
   {
      setPose(torus3d);
      this.radius = torus3d.radius;
      this.tubeRadius = torus3d.tubeRadius;
      
      checkRadiusAndThickness();
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
      setPose(transform);
      this.radius = radius;
      this.tubeRadius = thickness;
      
      checkRadiusAndThickness();
   }

   public Torus3d(Pose pose, double radius, double thickness)
   {
      setPose(pose);
      this.radius = radius;
      this.tubeRadius = thickness;
      
      checkRadiusAndThickness();
   }


   @Override
   public void set(Torus3d torus3d)
   {
      setPose(torus3d);
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
   public void applyTransform(Transform transform)
   {
      applyTransformToPose(transform);
   }

   @Override
   public String toString()
   {
      return "radius = " + radius + ", thickness = " + tubeRadius + ", pose = " + getPoseString() + "\n";
   }

   @Override
   protected boolean checkIfInsideShapeFrame(Point3DReadOnly pointInWorldToCheck, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      surfaceNormalAt(normalToPack, pointInWorldToCheck);
      closestPointToPack.set(pointInWorldToCheck);
      orthogonalProjectionShapeFrame(closestPointToPack);

      return isInsideOrOnSurfaceShapeFrame(pointInWorldToCheck, Epsilons.ONE_TEN_MILLIONTH);
   }

   @Override
   protected double distanceShapeFrame(Point3DReadOnly point)
   {
      temporaryPoint.set(point);
      orthogonalProjectionShapeFrame(temporaryPoint);

      return temporaryPoint.distance(point);
   }

   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(Point3DReadOnly pointToCheck, double epsilon)
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

   private void surfaceNormalAt(Vector3DBasics normalToPack, Point3DReadOnly pointToCheck)
   {
      computeCompositeVectorsForPoint(originToRadiusTemporaryVector, tubeCenterToPointTemporaryVector, pointToCheck);

      tubeCenterToPointTemporaryVector.normalize();
      normalToPack.set(tubeCenterToPointTemporaryVector);
   }

   @Override
   protected void orthogonalProjectionShapeFrame(Point3DBasics pointToCheckAndPack)
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
   protected void computeCompositeVectorsForPoint(Vector3DBasics originToRadiusVectorToPack, Vector3DBasics tubeCenterToPointVectorToPack, Point3DReadOnly pointToCheck)
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
