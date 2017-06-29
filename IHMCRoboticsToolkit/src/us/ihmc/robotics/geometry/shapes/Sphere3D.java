package us.ihmc.robotics.geometry.shapes;

import static us.ihmc.euclid.tools.EuclidCoreTools.*;

import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TransformationTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.MathTools;

/**
 * {@code Sphere3D} represents a 3D sphere defined by its radius and with its origin at its center.
 */
public class Sphere3D extends Shape3D<Sphere3D>
{
   /** The radius of this sphere. */
   private double radius;

   /**
    * Creates a new sphere 3D with a radius of {@code 1}.
    */
   public Sphere3D()
   {
      this(1.0);
   }

   /**
    * Creates a new sphere 3D identical to {@code other}
    * 
    * @param other the other sphere to copy. Not modified.
    */
   public Sphere3D(Sphere3D other)
   {
      set(other);
   }

   /**
    * Creates a new sphere 3D and initializes its radius.
    * 
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public Sphere3D(double radius)
   {
      setRadius(radius);
   }

   /**
    * Creates a new sphere 3D and initializes its position and radius.
    * 
    * @param center the coordinates of this sphere. Not modified.
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public Sphere3D(Point3DReadOnly center, double radius)
   {
      this(center.getX(), center.getY(), center.getZ(), radius);
   }

   /**
    * Creates a new sphere 3D and initializes its position and radius.
    * 
    * @param centerX the x-coordinate of this sphere. Not modified.
    * @param centerY the y-coordinate of this sphere. Not modified.
    * @param centerZ the z-coordinate of this sphere. Not modified.
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public Sphere3D(double centerX, double centerY, double centerZ, double radius)
   {
      setPosition(centerX, centerY, centerZ);
      setRadius(radius);
   }

   /**
    * Gets the radius of this sphere.
    * 
    * @return the value of the radius.
    */
   public double getRadius()
   {
      return radius;
   }

   /**
    * Sets the radius of this sphere.
    * 
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public void setRadius(double radius)
   {
      if (radius < 0.0)
         throw new IllegalArgumentException("The radius of a Sphere 3D cannot be negative.");
      this.radius = radius;
   }

   /**
    * Copies the {@code other} sphere data into {@code this}.
    * 
    * @param other the other sphere to copy. Not modified.
    */
   @Override
   public void set(Sphere3D other)
   {
      setPose(other);
      setRadius(other.radius);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this sphere.
    * <p>
    * In the case the line and this sphere do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    * 
    * @param line the line expressed in world coordinates that may intersect this sphere. Not
    *           modified.
    * @param firstIntersectionToPack the coordinate in world of the first intersection. Can be
    *           {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *           {@code null}. Modified.
    * @return the number of intersections between the line and this sphere. It is either equal to 0,
    *         1, or 2.
    */
   public int intersectionWith(Line3D line, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this sphere.
    * <p>
    * In the case the line and this sphere do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    * 
    * @param pointOnLine a point expressed in world located on the infinitely long line. Not
    *           modified.
    * @param lineDirection the direction expressed in world of the line. Not modified.s
    * @param firstIntersectionToPack the coordinate in world of the first intersection. Can be
    *           {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *           {@code null}. Modified.
    * @return the number of intersections between the line and this sphere. It is either equal to 0,
    *         1, or 2.
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

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(radius, radius, radius, xLocal, yLocal, zLocal, dxLocal, dyLocal,
                                                                                              dzLocal, firstIntersectionToPack, secondIntersectionToPack);
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
      double radiusWithEpsilon = radius + epsilon;
      return normSquared(x, y, z) <= radiusWithEpsilon * radiusWithEpsilon;
   }

   /** {@inheritDoc} */
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

   /**
    * Tests separately and on a per component basis if the pose and the radius of this sphere and
    * {@code other}'s pose and radius are equal to an {@code epsilon}.
    * 
    * @param other the other sphere which pose and radius is to be compared against this radius pose
    *           and radius. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two spheres are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Sphere3D other, double epsilon)
   {
      return MathTools.epsilonEquals(radius, other.radius, epsilon) && super.epsilonEqualsPose(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public void setToZero()
   {
      super.setToZero();
      radius = 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      super.setToNaN();
      radius = Double.NaN;
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || Double.isNaN(radius);
   }

   /**
    * Provides a {@code String} representation of this sphere 3D as follows:<br>
    * Sphere 3D: radius = r, pose = <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this box 3D.
    */
   @Override
   public String toString()
   {
      return "Sphere 3D: radius = " + radius + ", pose=\n" + getPoseString();
   }
}
