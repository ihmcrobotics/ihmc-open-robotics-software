package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TransformationTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.MathTools;

/**
 * {@code Cylinder3D} represents a cylinder defined by its radius and height.
 * <p>
 * Shape description:
 * <ul>
 * <li>The cylinder's axis is the z-axis.
 * <li>The cylinder's origin is at the center of the bottom face.
 * </ul>
 * </p>
 */
public class Cylinder3D extends Shape3D<Cylinder3D>
{
   /** Radius of the cylinder part. */
   private double radius;
   /** Z coordinate of the top face, the bottom face being at {@code 0.0}. */
   private double height;

   /**
    * Creates a new cylinder with height of {@code 1} and radius of {@code 0.5}.
    */
   public Cylinder3D()
   {
      this(1.0, 0.5);
   }

   /**
    * Creates a new cylinder 3D identical to {@code other}.
    * 
    * @param other the other cylinder to copy. Not modified.
    */
   public Cylinder3D(Cylinder3D other)
   {
      set(other);
   }

   /**
    * Creates a new cylinder 3D and initializes its height and radius.
    * 
    * @param height the cylinder length along the z-axis.
    * @param radius the radius of the cylinder.
    * @throws IllegalArgumentException if either {@code height} or {@code radius} is negative.
    */
   public Cylinder3D(double height, double radius)
   {
      setHeight(height);
      setRadius(radius);
   }

   /**
    * Creates a new cylinder 3D and initializes its pose, height, and radius.
    * 
    * @param pose the position and orientation of this cylinder. Not modified.
    * @param height the cylinder length along the z-axis.
    * @param radius the radius of the cylinder.
    * @throws IllegalArgumentException if either {@code height} or {@code radius} is negative.
    */
   public Cylinder3D(RigidBodyTransform pose, double height, double radius)
   {
      setPose(pose);
      setHeight(height);
      setRadius(radius);
   }

   /**
    * Creates a new cylinder 3D and initializes its pose, height, and radius.
    * 
    * @param pose the position and orientation of this cylinder. Not modified.
    * @param height the cylinder length along the z-axis.
    * @param radius the radius of the cylinder.
    * @throws IllegalArgumentException if either {@code height} or {@code radius} is negative.
    */
   public Cylinder3D(Pose3D pose, double height, double radius)
   {
      setPose(pose);
      setHeight(height);
      setRadius(radius);
   }

   /**
    * Copies the {@code other} cylinder data into {@code this}.
    * 
    * @param other the other cylinder to copy. Not modified.
    */
   @Override
   public void set(Cylinder3D other)
   {
      setPose(other);
      setHeight(height);
      setRadius(radius);
   }

   /**
    * Sets the radius of this cylinder.
    * 
    * @param radius the new radius for this cylinder.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public void setRadius(double radius)
   {
      if (radius < 0.0)
         throw new IllegalArgumentException("The radius of a Cylinder3D cannot be negative: " + radius);
      this.radius = radius;
   }

   /**
    * Sets the height of this cylinder.
    * 
    * @param height the cylinder length along the z-axis.
    * @throws IllegalArgumentException if {@code height} is negative.
    */
   public void setHeight(double height)
   {
      if (height < 0.0)
         throw new IllegalArgumentException("The height of a Cylinder3D cannot be negative: " + height);
      this.height = height;
   }

   /**
    * Gets the radius of this cylinder.
    * 
    * @return the value of the radius.
    */
   public double getRadius()
   {
      return radius;
   }

   /**
    * Gets the height of this cylinder.
    * 
    * @return the value of the height.
    */
   public double getHeight()
   {
      return height;
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || Double.isNaN(height) || Double.isNaN(radius);
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      super.setToNaN();
      height = Double.NaN;
      radius = Double.NaN;
   }

   /** {@inheritDoc} */
   @Override
   public void setToZero()
   {
      super.setToZero();
      height = 0.0;
      radius = 0.0;
   }

   /**
    * Tests separately and on a per component basis if the pose and the size of this cylinder and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    * 
    * @param other the other cylinder which pose and size is to be compared against this cylinder pose and
    *           size. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two cylinders are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Cylinder3D other, double epsilon)
   {
      return MathTools.epsilonEquals(height, other.height, epsilon) && MathTools.epsilonEquals(radius, other.radius, epsilon) && super.epsilonEqualsPose(other, epsilon);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this cylinder.
    * <p>
    * In the case the line and this cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    * 
    * @param line the line expressed in world coordinates that may intersect this cylinder. Not modified.
    * @param firstIntersectionToPack the coordinate in world of the first intersection. Can be
    *           {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *           {@code null}. Modified.
    * @return the number of intersections between the line and this cylinder. It is either equal to 0, 1,
    *         or 2.
    */
   public int intersectionWith(Line3D line, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this cylinder.
    * <p>
    * In the case the line and this cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    * 
    * @param pointOnLine a point expressed in world located on the infinitely long line. Not
    *           modified.
    * @param lineDirection the direction expressed in world of the line. Not modified.
    * @param firstIntersectionToPack the coordinate in world of the first intersection. Can be
    *           {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *           {@code null}. Modified.
    * @return the number of intersections between the line and this cylinder. It is either equal to 0, 1,
    *         or 2.
    */
   public int intersectionWith(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, Point3DBasics firstIntersectionToPack,
                               Point3DBasics secondIntersectionToPack)
   {
      double xLocal = TransformationTools.computeTransformedX(shapePose, true, pointOnLine);
      double yLocal = TransformationTools.computeTransformedY(shapePose, true, pointOnLine);
      double zLocal = TransformationTools.computeTransformedZ(shapePose, true, pointOnLine);

      double dxLocal = TransformationTools.computeTransformedX(shapePose, true, lineDirection);
      double dyLocal = TransformationTools.computeTransformedY(shapePose, true, lineDirection);
      double dzLocal = TransformationTools.computeTransformedZ(shapePose, true, lineDirection);

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(height, radius, xLocal, yLocal, zLocal, dxLocal, dyLocal, dzLocal,
                                                                                             firstIntersectionToPack, secondIntersectionToPack);
      if (firstIntersectionToPack != null && numberOfIntersections >= 1)
         transformToWorld(firstIntersectionToPack);
      if (secondIntersectionToPack != null && numberOfIntersections == 2)
         transformToWorld(secondIntersectionToPack);
      return numberOfIntersections;
   }

   /** {@inheritDoc} */
   @Override
   protected boolean isInsideEpsilonShapeFrame(double x, double y, double z, double epsilon)
   {
      if (z < -epsilon || z > height + epsilon)
         return false;

      double radiusWithEpsilon = radius + epsilon;
      return EuclidCoreTools.normSquared(x, y) <= radiusWithEpsilon * radiusWithEpsilon;
   }

   /** {@inheritDoc} */
   @Override
   protected double evaluateQuery(double x, double y, double z, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack)
   {
      double xyLengthSquared = EuclidCoreTools.normSquared(x, y);

      if (xyLengthSquared <= radius * radius)
      {
         if (z < 0.0)
         { // The query is directly below the cylinder
            if (closestPointOnSurfaceToPack != null)
               closestPointOnSurfaceToPack.set(x, y, 0.0);
            if (normalToPack != null)
               normalToPack.set(0.0, 0.0, -1.0);
            return -z;
         }

         if (z > height)
         { // The query is directly above the cylinder
            if (closestPointOnSurfaceToPack != null)
               closestPointOnSurfaceToPack.set(x, y, height);
            if (normalToPack != null)
               normalToPack.set(0.0, 0.0, 1.0);
            return z - height;
         }

         // The query is inside the cylinder
         double xyLength = Math.sqrt(xyLengthSquared);
         double dz = Math.min(z, height - z);
         double dr = radius - xyLength;

         if (dz < dr)
         {
            if (z == dz)
            { // Closer to the bottom face
               if (closestPointOnSurfaceToPack != null)
                  closestPointOnSurfaceToPack.set(x, y, 0.0);
               if (normalToPack != null)
                  normalToPack.set(0.0, 0.0, -1.0);
               return -z;
            }
            else
            { // Closer to the top face
               if (closestPointOnSurfaceToPack != null)
                  closestPointOnSurfaceToPack.set(x, y, height);
               if (normalToPack != null)
                  normalToPack.set(0.0, 0.0, 1.0);
               return z - height;
            }
         }
         else
         { // Closer to the cylinder part
            if (closestPointOnSurfaceToPack != null)
            {
               double xyScale = radius / xyLength;
               closestPointOnSurfaceToPack.set(x * xyScale, y * xyScale, z);
            }

            if (normalToPack != null)
            {
               normalToPack.set(x, y, 0.0);
               normalToPack.scale(1.0 / xyLength);
            }
            return xyLength - radius;
         }
      }
      else
      { // The projection of the query onto the xy-plane is outside of the cylinder
         double xyLength = Math.sqrt(xyLengthSquared);
         double xyLengthInv = 1.0 / xyLength;

         double xyClosestScale = radius * xyLengthInv;
         double xClosest = x * xyClosestScale;
         double yClosest = y * xyClosestScale;
         double zClosest = z;

         if (z < 0.0)
            zClosest = 0.0;
         else if (z > height)
            zClosest = height;

         if (zClosest != z)
         { // Closest point is on the circle adjacent to the cylinder and top or bottom face.

            double dx = x - xClosest;
            double dy = y - yClosest;
            double dz = z - zClosest;

            double distance = Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));

            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            }

            if (normalToPack != null)
            {
               normalToPack.set(dx, dy, dz);
               normalToPack.scale(1.0 / distance);
            }

            return distance;
         }
         else
         { // Closest point is on the cylinder.
            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            }

            if (normalToPack != null)
            {
               normalToPack.set(x * xyLengthInv, y * xyLengthInv, 0.0);
            }

            return xyLength - radius;
         }
      }
   }

   /**
    * Provides a {@code String} representation of this cylinder 3D as follows:<br>
    * Cylinder 3D: height = h, radius = r, pose = <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this cylinder 3D.
    */
   @Override
   public String toString()
   {
      return "Cylinder 3D: height = " + height + ", radius = " + radius + ", pose=\n" + getPoseString();
   }
}
