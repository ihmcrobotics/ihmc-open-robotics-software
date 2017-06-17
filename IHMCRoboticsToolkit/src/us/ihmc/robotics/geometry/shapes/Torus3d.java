package us.ihmc.robotics.geometry.shapes;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.MathTools;

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

   public Torus3d(Pose3D pose, double radius, double thickness)
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
         throw new RuntimeException("Torus3d: checkRadiusAndThickness(): "
               + "Invalid dimensions: Difference between radius and thickness is too small. Enter new dimensions.");

      if (tubeRadius < SMALLEST_ALLOWABLE_THICKNESS)
         throw new RuntimeException("Torus3d: checkRadiusAndThickness(): " + "Invalid thickness: Torus is too thin. Enter new dimensions.");
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
      super.setToZero();
      radius = 0.0;
      tubeRadius = 0.0;
   }

   @Override
   public void setToNaN()
   {
      super.setToNaN();
      radius = Double.NaN;
      tubeRadius = Double.NaN;
   }

   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || Double.isNaN(radius) || Double.isNaN(tubeRadius);
   }

   @Override
   public String toString()
   {
      return "radius = " + radius + ", thickness = " + tubeRadius + ", pose = " + getPoseString() + "\n";
   }

   @Override
   protected boolean checkIfInsideShapeFrame(double x, double y, double z, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      surfaceNormalAt(x, y, z, normalToPack);
      closestPointToPack.set(x, y, z);
      orthogonalProjectionShapeFrame(closestPointToPack);

      return isInsideOrOnSurfaceShapeFrame(x, y, z, Epsilons.ONE_TEN_MILLIONTH);
   }

   @Override
   protected double distanceShapeFrame(double x, double y, double z)
   {
      temporaryPoint.set(x, y, z);
      orthogonalProjectionShapeFrame(temporaryPoint);

      return EuclidGeometryTools.distanceBetweenPoint3Ds(x, y, z, temporaryPoint);
   }

   @Override
   protected boolean isInsideOrOnSurfaceShapeFrame(double x, double y, double z, double epsilon)
   {
      temporaryVector.set(x, y, 0.0);

      if (temporaryVector.length() < Epsilons.ONE_TRILLIONTH)
      {
         return tubeRadius >= radius;
      }

      temporaryVector.normalize();
      temporaryVector.scale(radius);

      temporaryPoint.set(temporaryVector);

      return EuclidGeometryTools.distanceBetweenPoint3Ds(x, y, z, temporaryPoint) <= tubeRadius + epsilon;
   }

   private void surfaceNormalAt(double x, double y, double z, Vector3DBasics normalToPack)
   {
      computeCompositeVectorsForPoint(x, y, z, originToRadiusTemporaryVector, tubeCenterToPointTemporaryVector);

      tubeCenterToPointTemporaryVector.normalize();
      normalToPack.set(tubeCenterToPointTemporaryVector);
   }

   @Override
   protected void orthogonalProjectionShapeFrame(double x, double y, double z, Point3DBasics projectionToPack)
   {
      computeCompositeVectorsForPoint(projectionToPack, originToRadiusTemporaryVector, tubeCenterToPointTemporaryVector);

      if (tubeCenterToPointTemporaryVector.length() > tubeRadius)
         tubeCenterToPointTemporaryVector.scale(tubeRadius / tubeCenterToPointTemporaryVector.length());
      tubeCenterToPointTemporaryVector.add(originToRadiusTemporaryVector);

      projectionToPack.set(tubeCenterToPointTemporaryVector);
   }

   /**
    * Compute the vector, R, from the center point of the torus to the radius of the torus in the XY
    * direction of the given point. Computes the vector from the closest point in the middle of the
    * torus tube (at R) to the given point.
    * 
    * @param pointToCheck the point in world coordinates for which to compute the origin-to-radius
    *           and tube-center-to-point vectors.
    * @param originToRadiusVectorToPack the vector from the center point of the torus to the radius
    *           of the torus in the XY direction of the given point.
    * @param tubeCenterToPointVectorToPack the vector from the closest point in the middle of the
    *           torus tube (at R) to the given point.
    */
   protected void computeCompositeVectorsForPoint(Point3DReadOnly pointToCheck, Vector3DBasics originToRadiusVectorToPack,
                                                  Vector3DBasics tubeCenterToPointVectorToPack)
   {
      computeCompositeVectorsForPoint(pointToCheck.getX(), pointToCheck.getY(), pointToCheck.getZ(), originToRadiusVectorToPack, tubeCenterToPointVectorToPack);
   }

   protected void computeCompositeVectorsForPoint(double x, double y, double z, Vector3DBasics originToRadiusVectorToPack,
                                                  Vector3DBasics tubeCenterToPointVectorToPack)
   {
      temporaryPoint.set(x, y, z);

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
