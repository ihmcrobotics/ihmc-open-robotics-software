package us.ihmc.robotics.geometry.shapes;

import static us.ihmc.euclid.tools.EuclidCoreTools.*;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.robotics.MathTools;

/**
 * {@code Torus3D} represents a torus in the XY-plane.
 * <p>
 * Shape description:
 * <ul>
 * <li>The origin is located at the centroid or center of the torus.
 * <li>The axis of revolution is the z-axis.
 * <li>The torus is defined by two radii: {@link #tubeRadius} that represents the radius of the
 * tube, and {@link #radius} that is the radius for the center of the torus to the center of the
 * tube.
 * </ul>
 * </p>
 */
public class Torus3D extends Shape3D<Torus3D>
{
   private static final double MIN_TUBE_RADIUS = 1.0e-4;
   private static final double MIN_INNER_RADIUS = 1.0e-4;

   /** It is the radius for the center of the torus to the center of the tube. */
   private double radius;
   /** Represents the radius of the tube */
   private double tubeRadius;

   /**
    * Creates a new torus 3D with a radius of {@code 1}, and tube radius of {@code 0.1}.
    */
   public Torus3D()
   {
      this(1.0, 0.1);
   }

   /**
    * Creates a new torus 3D identical to {@code other}.
    * 
    * @param other the other torus to copy. Not modified.
    */
   public Torus3D(Torus3D other)
   {
      set(other);
   }

   /**
    * Creates a new torus 3D and initializes its radii.
    * 
    * @param radius radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code tubeRadius} is less than {@value #MIN_TUBE_RADIUS}
    *            or if the resulting inner radius is less than {@value #MIN_INNER_RADIUS}.
    */
   public Torus3D(double radius, double tubeRadius)
   {
      setRadii(radius, tubeRadius);
   }

   /**
    * Creates a new torus 3D and initializes its pose and radii.
    * 
    * @param pose the position and orientation of this torus. Not modified.
    * @param radius radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code tubeRadius} is less than {@value #MIN_TUBE_RADIUS}
    *            or if the resulting inner radius is less than {@value #MIN_INNER_RADIUS}.
    */
   public Torus3D(RigidBodyTransform pose, double radius, double tubeRadius)
   {
      set(pose, radius, tubeRadius);
   }

   /**
    * Creates a new torus 3D and initializes its pose and radii.
    * 
    * @param pose the position and orientation of this torus. Not modified.
    * @param radius radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code tubeRadius} is less than {@value #MIN_TUBE_RADIUS}
    *            or if the resulting inner radius is less than {@value #MIN_INNER_RADIUS}.
    */
   public Torus3D(Pose3D pose, double radius, double thickness)
   {
      set(pose, radius, tubeRadius);
   }

   /**
    * Copies the {@code other} torus data into {@code this}.
    * 
    * @param other the other torus to copy. Not modified.
    */
   @Override
   public void set(Torus3D other)
   {
      setPose(other);
      this.radius = other.radius;
      this.tubeRadius = other.tubeRadius;
   }

   /**
    * Sets the pose and radii of this torus 3D.
    * 
    * @param pose the position and orientation of this torus. Not modified.
    * @param radius radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code tubeRadius} is less than {@value #MIN_TUBE_RADIUS}
    *            or if the resulting inner radius is less than {@value #MIN_INNER_RADIUS}.
    */
   public void set(Pose3D pose, double radius, double tubeRadius)
   {
      setPose(pose);
      setRadii(radius, tubeRadius);
   }

   /**
    * Sets the pose and radii of this torus 3D.
    * 
    * @param pose the position and orientation of this torus. Not modified.
    * @param radius radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code tubeRadius} is less than {@value #MIN_TUBE_RADIUS}
    *            or if the resulting inner radius is less than {@value #MIN_INNER_RADIUS}.
    */
   public void set(RigidBodyTransform pose, double radius, double tubeRadius)
   {
      setPose(pose);
      setRadii(radius, tubeRadius);
   }

   /**
    * Sets the radii of this torus 3D.
    * 
    * @param radius radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code tubeRadius} is less than {@value #MIN_TUBE_RADIUS}
    *            or if the resulting inner radius is less than {@value #MIN_INNER_RADIUS}.
    */
   public void setRadii(double radius, double tubeRadius)
   {
      this.radius = radius;
      this.tubeRadius = tubeRadius;

      if (radius - tubeRadius < MIN_INNER_RADIUS)
         throw new IllegalArgumentException("Invalid dimensions: Difference between radius and tube radius is too small, difference: " + (radius - tubeRadius)
               + ", minimum allowed: " + MIN_INNER_RADIUS);

      if (tubeRadius < MIN_TUBE_RADIUS)
         throw new IllegalArgumentException("Invalid tube radius: Torus is too thin: " + tubeRadius + ", minimum allowed: " + MIN_TUBE_RADIUS);
   }

   /**
    * Gets the radius from the torus center to the tube center.
    * 
    * @return this torus main radius.
    */
   public double getRadius()
   {
      return radius;
   }

   /**
    * Gets the radius of the tube of this torus.
    * 
    * @return the radius of the tube.
    */
   public double getTubeRadius()
   {
      return tubeRadius;
   }

   /**
    * Tests separately and on a per component basis if the pose and the radii of this torus and
    * {@code other}'s pose and radii are equal to an {@code epsilon}.
    * 
    * @param other the other torus which pose and radii is to be compared against this torus pose
    *           and radii. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two toruses are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Torus3D other, double epsilon)
   {
      return MathTools.epsilonEquals(radius, other.radius, epsilon) && MathTools.epsilonEquals(tubeRadius, other.tubeRadius, epsilon)
            && super.epsilonEqualsPose(other, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   public void setToZero()
   {
      super.setToZero();
      radius = 0.0;
      tubeRadius = 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      super.setToNaN();
      radius = Double.NaN;
      tubeRadius = Double.NaN;
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || Double.isNaN(radius) || Double.isNaN(tubeRadius);
   }

   /**
    * Provides a {@code String} representation of this torus 3D as follows:<br>
    * Torus 3D: radius = R, tube radius = r, pose = <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this torus 3D.
    */
   @Override
   public String toString()
   {
      return "Torus: radius = " + radius + ", tube radius = " + tubeRadius + ", pose =\n" + getPoseString();
   }

   /** {@inheritDoc} */
   @Override
   protected boolean isInsideEpsilonShapeFrame(double x, double y, double z, double epsilon)
   {
      double xyLengthSquared = EuclidCoreTools.normSquared(x, y);

      double outerRadius = radius + tubeRadius + epsilon;
      double innerRadius = radius - tubeRadius - epsilon;

      if (xyLengthSquared > outerRadius * outerRadius || xyLengthSquared < innerRadius * innerRadius)
         return false;

      double xyScale = radius / Math.sqrt(xyLengthSquared);

      double closestTubeCenterX = x * xyScale;
      double closestTubeCenterY = y * xyScale;

      return EuclidGeometryTools.distanceBetweenPoint3Ds(x, y, z, closestTubeCenterX, closestTubeCenterY, 0.0) <= tubeRadius + epsilon;
   }

   /** {@inheritDoc} */
   @Override
   protected double evaluateQuery(double x, double y, double z, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      double xyLengthSquared = normSquared(x, y);

      if (xyLengthSquared < 1.0e-12)
      {
         double xzLength = Math.sqrt(normSquared(radius, z));

         if (closestPointToPack != null)
         {
            double scale = tubeRadius / xzLength;
            double closestTubeCenterX = radius;
            double closestTubeCenterY = 0.0;

            double tubeCenterToSurfaceX = -radius * scale;
            double tubeCenterToSurfaceY = 0.0;
            double tubeCenterToSurfaceZ = z * scale;

            closestPointToPack.set(closestTubeCenterX, closestTubeCenterY, 0.0);
            closestPointToPack.add(tubeCenterToSurfaceX, tubeCenterToSurfaceY, tubeCenterToSurfaceZ);
         }

         if (normalToPack != null)
         {
            normalToPack.set(-radius, 0.0, z);
            normalToPack.scale(1.0 / xzLength);
         }

         return xzLength - tubeRadius;
      }
      else
      {
         double xyScale = radius / Math.sqrt(xyLengthSquared);

         double closestTubeCenterX = x * xyScale;
         double closestTubeCenterY = y * xyScale;

         double dx = x - closestTubeCenterX;
         double dy = y - closestTubeCenterY;
         double dz = z;

         double distance = Math.sqrt(normSquared(dx, dy, dz));

         double distanceInv = 1.0 / distance;

         if (closestPointToPack != null)
         {
            closestPointToPack.set(dx, dy, dz);
            closestPointToPack.scale(tubeRadius * distanceInv);
            closestPointToPack.add(closestTubeCenterX, closestTubeCenterY, 0.0);
         }

         if (normalToPack != null)
         {
            normalToPack.set(dx, dy, dz);
            normalToPack.scale(distanceInv);
         }

         return distance - tubeRadius;
      }
   }
}
