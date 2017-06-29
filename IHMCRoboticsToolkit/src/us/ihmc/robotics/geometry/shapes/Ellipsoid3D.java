package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TransformationTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * {@code Ellipsoid3D} represents a 3D ellipsoid defined by its three main radii and with its origin
 * at its center.
 */
public class Ellipsoid3D extends Shape3D<Ellipsoid3D>
{
   /** The three radii of this ellipsoid. */
   private final Vector3D radii = new Vector3D();

   /**
    * Creates a new ellipsoid 3D with its 3 radii initialized to {@code 1}.
    */
   public Ellipsoid3D()
   {
      this(1.0, 1.0, 1.0);
   }

   /**
    * Creates a new ellipsoid 3D identical to {@code other}.
    * 
    * @param other the other ellipsoid to copy. Not modified.
    */
   public Ellipsoid3D(Ellipsoid3D other)
   {
      set(other);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its radii.
    * 
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public Ellipsoid3D(double radiusX, double radiusY, double radiusZ)
   {
      setRadii(radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    * 
    * @param pose the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public Ellipsoid3D(Pose3D pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    * 
    * @param pose the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public Ellipsoid3D(RigidBodyTransform pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose, radiusX, radiusY, radiusZ);
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || radii.containsNaN();
   }

   /**
    * Tests separately and on a per component basis if the pose and the radii of this ellipsoid and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    * 
    * @param other the other ellipsoid which pose and radii is to be compared against this ellipsoid
    *           pose and radii. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two ellipsoids are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Ellipsoid3D other, double epsilon)
   {
      return radii.epsilonEquals(other.radii, epsilon) && super.epsilonEqualsPose(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   protected double evaluateQuery(double x, double y, double z, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      double sumOfSquares = EuclidCoreTools.normSquared(x / radii.getX(), y / radii.getY(), z / radii.getZ());
      double scaleFactor = 1.0 / Math.sqrt(sumOfSquares);

      if (sumOfSquares > 1.0e-10)
      {
         if (closestPointToPack != null)
         {
            closestPointToPack.set(x, y, z);
            closestPointToPack.scale(scaleFactor);
         }

         if (normalToPack != null)
         {
            double xScale = 1.0 / (radii.getX() * radii.getX());
            double yScale = 1.0 / (radii.getY() * radii.getY());
            double zScale = 1.0 / (radii.getZ() * radii.getZ());

            normalToPack.set(x, y, z);
            normalToPack.scale(xScale, yScale, zScale);
            normalToPack.normalize();
         }

         return Math.sqrt(EuclidCoreTools.normSquared(x, y, z)) * (1.0 - scaleFactor);
      }
      else
      {
         if (closestPointToPack != null)
         {
            closestPointToPack.set(0.0, 0.0, radii.getZ());
         }

         if (normalToPack != null)
         {
            normalToPack.set(0.0, 0.0, 1.0);
         }

         return z - radii.getZ();
      }
   }

   /**
    * Packs the 3 radii of this ellipsoid in the given tuple.
    * 
    * @param radiiToPack the tuple in which the radii are stored. Modified.
    */
   public void getRadii(Tuple3DBasics radiiToPack)
   {
      radiiToPack.set(radii);
   }

   /**
    * Gets the radius along the x-axis.
    * 
    * @return the x radius.
    */
   public double getRadiusX()
   {
      return radii.getX();
   }

   /**
    * Gets the radius along the y-axis.
    * 
    * @return the y radius.
    */
   public double getRadiusY()
   {
      return radii.getY();
   }

   /**
    * Gets the radius along the z-axis.
    * 
    * @return the z radius.
    */
   public double getRadiusZ()
   {
      return radii.getZ();
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this ellipsoid.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    * 
    * @param line the line expressed in world coordinates that may intersect this ellipsoid. Not
    *           modified.
    * @param firstIntersectionToPack the coordinate in world of the first intersection. Can be
    *           {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *           {@code null}. Modified.
    * @return the number of intersections between the line and this ellipsoid. It is either equal to
    *         0, 1, or 2.
    */
   public int intersectionWith(Line3D line, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this ellipsoid.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
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
    * @return the number of intersections between the line and this ellipsoid. It is either equal to
    *         0, 1, or 2.
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

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radii.getX(), radii.getY(), radii.getZ(), xLocal, yLocal, zLocal,
                                                                                              dxLocal, dyLocal, dzLocal, firstIntersectionToPack,
                                                                                              secondIntersectionToPack);
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
      double scaledX = x / (radii.getX() + epsilon);
      double scaledY = y / (radii.getY() + epsilon);
      double scaledZ = z / (radii.getZ() + epsilon);

      return EuclidCoreTools.normSquared(scaledX, scaledY, scaledZ) <= 1.0;
   }

   /**
    * Copies the {@code other} ellipsoid data into {@code this}.
    * 
    * @param other the other ellipsoid to copy. Not modified.
    */
   @Override
   public void set(Ellipsoid3D other)
   {
      setPose(other);
      radii.set(other.radii);
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      super.setToNaN();
      radii.setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   public void setToZero()
   {
      super.setToZero();
      radii.setToZero();
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    * 
    * @param pose the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public void set(Pose3D pose, double radiusX, double radiusY, double radiusZ)
   {
      setPose(pose);
      setRadii(radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    * 
    * @param pose the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public void set(RigidBodyTransform pose, double radiusX, double radiusY, double radiusZ)
   {
      setPose(pose);
      setRadii(radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the radii of this ellipsoid.
    * 
    * @param radii tuple holding the 3 radii of the ellipsoid.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public void setRadii(Tuple3DReadOnly radii)
   {
      setRadiusX(radii.getX());
      setRadiusY(radii.getY());
      setRadiusZ(radii.getZ());
   }

   /**
    * Sets the radii of this ellipsoid.
    * 
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public void setRadii(double radiusX, double radiusY, double radiusZ)
   {
      setRadiusX(radiusX);
      setRadiusY(radiusY);
      setRadiusZ(radiusZ);
   }

   /**
    * Sets the radius along the x-axis for this ellipsoid.
    * 
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @throws IllegalArgumentException if the argument is negative.
    */
   public void setRadiusX(double radiusX)
   {
      if (radiusX < 0.0)
         throw new IllegalArgumentException("The x-radius of an Ellipsoid3D cannot be negative: " + radiusX);
      radii.setX(radiusX);
   }

   /**
    * Sets the radius along the y-axis for this ellipsoid.
    * 
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @throws IllegalArgumentException if the argument is negative.
    */
   public void setRadiusY(double radiusY)
   {
      if (radiusY < 0.0)
         throw new IllegalArgumentException("The y-radius of an Ellipsoid3D cannot be negative: " + radiusY);
      radii.setY(radiusY);
   }

   /**
    * Sets the radius along the z-axis for this ellipsoid.
    * 
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if the argument is negative.
    */
   public void setRadiusZ(double radiusZ)
   {
      if (radiusZ < 0.0)
         throw new IllegalArgumentException("The z-radius of an Ellipsoid3D cannot be negative: " + radiusZ);
      radii.setZ(radiusZ);
   }

   /**
    * Provides a {@code String} representation of this ellipsoid 3D as follows:<br>
    * Ellipsoid 3D: radii = (rx, ry, rz), pose = <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this ellipsoid 3D.
    */
   @Override
   public String toString()
   {
      return "Ellipsoid 3D: radii = " + radii + ", pose =\n" + getPoseString();
   }
}
