package us.ihmc.robotics.physics;

import java.util.Random;

import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.primitives.*;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

// FIXME Finish up a release of Euclid and remove this class.
@Deprecated
public class EuclidShapeMissingRandomTools
{
   private EuclidShapeMissingRandomTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Generates a random box 3D.
    * <ul>
    * <li>{@code size}<sub>i</sub> &in; [{@code minSize}; {@code maxSize}].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * 
    * @param random   the random generator to use.
    * @param position the position of the box's center. Not modified.
    * @return the random box 3D.
    * @throws RuntimeException if {@code minSize > maxSize}.
    */
   public static Box3D nextBox3D(Random random, Tuple3DReadOnly position)
   {
      return nextBox3D(random, position, 0.0, 1.0);
   }

   /**
    * Generates a random box 3D.
    * <ul>
    * <li>{@code size}<sub>i</sub> &in; [{@code minSize}; {@code maxSize}].
    * </ul>
    * 
    * @param random   the random generator to use.
    * @param position the position of the box's center. Not modified.
    * @param minSize  the minimum value for each component of the box size.
    * @param maxSize  the maximum value for each component of the box size.
    * @return the random box 3D.
    * @throws RuntimeException if {@code minSize > maxSize}.
    */
   public static Box3D nextBox3D(Random random, Tuple3DReadOnly position, double minSize, double maxSize)
   {
      RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      pose.getTranslation().set(position);
      return new Box3D(pose,
                       EuclidCoreRandomTools.nextDouble(random, minSize, maxSize),
                       EuclidCoreRandomTools.nextDouble(random, minSize, maxSize),
                       EuclidCoreRandomTools.nextDouble(random, minSize, maxSize));
   }

   /**
    * Generates a random capsule 3D.
    * <ul>
    * <li>{@code length} &in; [0.0; 1.0].
    * <li>{@code radius} &in; [0.0; 1.0].
    * <li>{@code axis} is generated using
    * {@link EuclidCoreRandomTools#nextVector3DWithFixedLength(Random, double)} with a length of 1.0.
    * </ul>
    * 
    * @param random   the random generator to use.
    * @param position the position of the generated capsule's center. Not modified.
    * @return the random capsule 3D.
    * @throws RuntimeException if {@code minLength > maxLength} or {@code minRadius > maxRadius}.
    */
   public static Capsule3D nextCapsule3D(Random random, Tuple3DReadOnly position)
   {
      return nextCapsule3D(random, position, 0.0, 1.0, 0.0, 1.0);
   }

   /**
    * Generates a random capsule 3D.
    * <ul>
    * <li>{@code length} &in; [{@code minLength}; {@code maxLength}].
    * <li>{@code radius} &in; [{@code minRadius}; {@code maxRadius}].
    * <li>{@code axis} is generated using
    * {@link EuclidCoreRandomTools#nextVector3DWithFixedLength(Random, double)} with a length of 1.0.
    * </ul>
    * 
    * @param random    the random generator to use.
    * @param position  the position of the generated capsule's center. Not modified.
    * @param minLength the minimum value for the length.
    * @param maxLength the maximum value for the length.
    * @param minRadius the minimum value for the radius.
    * @param maxRadius the maximum value for the radius.
    * @return the random capsule 3D.
    * @throws RuntimeException if {@code minLength > maxLength} or {@code minRadius > maxRadius}.
    */
   public static Capsule3D nextCapsule3D(Random random, Tuple3DReadOnly position, double minLength, double maxLength, double minRadius, double maxRadius)
   {
      return new Capsule3D(new Point3D(position),
                           EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0),
                           EuclidCoreRandomTools.nextDouble(random, minLength, maxLength),
                           EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   /**
    * Generates a random cylinder 3D.
    * <ul>
    * <li>{@code length} &in; [0.0; 1.0].
    * <li>{@code radius} &in; [0.0; 1.0].
    * <li>{@code axis} is generated using
    * {@link EuclidCoreRandomTools#nextVector3DWithFixedLength(Random, double)} with a length of 1.0.
    * </ul>
    * 
    * @param random   the random generator to use.
    * @param position the position of the cylinder's center. Not modified.
    * @return the random cylinder 3D.
    * @throws RuntimeException if {@code minLength > maxLength} or {@code minRadius > maxRadius}.
    */
   public static Cylinder3D nextCylinder3D(Random random, Tuple3DReadOnly position)
   {
      return nextCylinder3D(random, position, 0.0, 1.0, 0.0, 1.0);
   }

   /**
    * Generates a random cylinder 3D.
    * <ul>
    * <li>{@code length} &in; [{@code minLength}; {@code maxLength}].
    * <li>{@code radius} &in; [{@code minRadius}; {@code maxRadius}].
    * <li>{@code axis} is generated using
    * {@link EuclidCoreRandomTools#nextVector3DWithFixedLength(Random, double)} with a length of 1.0.
    * </ul>
    * 
    * @param random    the random generator to use.
    * @param position  the position of the cylinder's center. Not modified.
    * @param minLength the minimum value for the length.
    * @param maxLength the maximum value for the length.
    * @param minRadius the minimum value for the radius.
    * @param maxRadius the maximum value for the radius.
    * @return the random cylinder 3D.
    * @throws RuntimeException if {@code minLength > maxLength} or {@code minRadius > maxRadius}.
    */
   public static Cylinder3D nextCylinder3D(Random random, Tuple3DReadOnly position, double minLength, double maxLength, double minRadius, double maxRadius)
   {
      return new Cylinder3D(new Point3D(position),
                            EuclidCoreRandomTools.nextVector3D(random),
                            EuclidCoreRandomTools.nextDouble(random, minLength, maxLength),
                            EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   /**
    * Generates a random ellipsoid 3D.
    * <ul>
    * <li>{@code radii}<sub>i</sub> &in; [{@code minSize}; {@code maxSize}].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * 
    * @param random   the random generator to use.
    * @param position the position of the ellipsoid's center. Not modified.
    * @return the random ellipsoid 3D.
    * @throws RuntimeException if {@code minRadius > maxRadius}.
    */
   public static Ellipsoid3D nextEllipsoid3D(Random random, Tuple3DReadOnly position)
   {
      return nextEllipsoid3D(random, position, 0.0, 1.0);
   }

   /**
    * Generates a random ellipsoid 3D.
    * <ul>
    * <li>{@code radii}<sub>i</sub> &in; [{@code minSize}; {@code maxSize}].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * 
    * @param random    the random generator to use.
    * @param position  the position of the ellipsoid's center. Not modified.
    * @param minRadius the minimum value for each component of the ellipsoid radius.
    * @param maxRadius the maximum value for each component of the ellipsoid radius.
    * @return the random ellipsoid 3D.
    * @throws RuntimeException if {@code minRadius > maxRadius}.
    */
   public static Ellipsoid3D nextEllipsoid3D(Random random, Tuple3DReadOnly position, double minRadius, double maxRadius)
   {
      RigidBodyTransform pose = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      pose.getTranslation().set(position);
      return new Ellipsoid3D(pose,
                             EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius),
                             EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius),
                             EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   /**
    * Generates a random ramp 3D.
    * <ul>
    * <li>{@code size}<sub>i</sub> &in; [0.0; 1.0].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * 
    * @param random   the random generator to use.
    * @param position the position of the ramp's centroid. Not modified.
    * @return the random ramp 3D.
    * @throws RuntimeException if {@code minSize > maxSize}.
    */
   public static Ramp3D nextRamp3D(Random random, Tuple3DReadOnly position)
   {
      return nextRamp3D(random, position, 0.0, 1.0);
   }

   /**
    * Generates a random ramp 3D.
    * <ul>
    * <li>{@code size}<sub>i</sub> &in; [{@code minSize}; {@code maxSize}].
    * <li>{@code orientation.getAngle()} &in; [-<i>pi</i>; <i>pi</i>].
    * </ul>
    * 
    * @param random   the random generator to use.
    * @param position the position of the ramp's centroid. Not modified.
    * @param minSize  the minimum value for each component of the ramp size.
    * @param maxSize  the maximum value for each component of the ramp size.
    * @return the random ramp 3D.
    * @throws RuntimeException if {@code minSize > maxSize}.
    */
   public static Ramp3D nextRamp3D(Random random, Tuple3DReadOnly position, double minSize, double maxSize)
   {
      Ramp3D next = EuclidShapeRandomTools.nextRamp3D(random, minSize, maxSize);
      Point3D centroid = new Point3D(2.0 / 3.0 * next.getSizeX(), 0.0, 1.0 / 3.0 * next.getSizeZ());
      next.transformToWorld(centroid);
      next.getPose().getTranslation().sub(centroid);
      next.getPose().getTranslation().add(position);
      return next;
   }

   /**
    * Generates a random sphere 3D.
    * <ul>
    * <li>{@code radius} &in; [0.0; 1.0].
    * </ul>
    * 
    * @param random   the random generator to use.
    * @param position the position of the sphere's center. Not modified.
    * @return the random sphere 3D.
    * @throws RuntimeException if {@code minRadius > maxRadius}.
    */
   public static Sphere3D nextSphere3D(Random random, Tuple3DReadOnly position)
   {
      return nextSphere3D(random, position, 0.0, 1.0);
   }

   /**
    * Generates a random sphere 3D.
    * <ul>
    * <li>{@code radius} &in; [{@code minRadius}; {@code maxRadius}].
    * </ul>
    * 
    * @param random    the random generator to use.
    * @param position  the position of the sphere's center. Not modified.
    * @param minRadius the minimum value for the radius.
    * @param maxRadius the maximum value for the radius.
    * @return the random sphere 3D.
    * @throws RuntimeException if {@code minRadius > maxRadius}.
    */
   public static Sphere3D nextSphere3D(Random random, Tuple3DReadOnly position, double minRadius, double maxRadius)
   {
      return new Sphere3D(new Point3D(position), EuclidCoreRandomTools.nextDouble(random, minRadius, maxRadius));
   }

   /**
    * Generates a random convex polytope 3D.
    * <p>
    * The convex polytope is generated using {@link EuclidShapeRandomTools#nextConvexPolytope3D(Random)} and then translated
    * such that its centroid is at the given position.
    * 
    * @param random   the random generator to use.
    * @param position the position of the polytope's centroid. Not modified.
    * @return the random convex polytope 3D.
    */
   public static ConvexPolytope3D nextConvexPolytope3D(Random random, Tuple3DReadOnly position)
   {
      ConvexPolytope3D next = EuclidShapeRandomTools.nextConvexPolytope3D(random);
      Vector3D translation = new Vector3D();
      translation.sub(position, next.getCentroid());
      next.applyTransform(new RigidBodyTransform(new Quaternion(), translation));
      return next;
   }

   /**
    * Generates a random convex shape 3D.
    * <p>
    * The shape is generated by picking at random one of the following generators:
    * <ul>
    * <li>{@link #nextBox3D(Random, Tuple3DReadOnly)}.
    * <li>{@link #nextCapsule3D(Random, Tuple3DReadOnly)}.
    * <li>{@link #nextConvexPolytope3D(Random, Tuple3DReadOnly)}.
    * <li>{@link #nextCylinder3D(Random, Tuple3DReadOnly)}.
    * <li>{@link #nextEllipsoid3D(Random, Tuple3DReadOnly)}.
    * <li>In case of {@link PointShape3D}, the resulting shape is not random because fully constrained
    * by the given position.
    * <li>{@link #nextRamp3D(Random, Tuple3DReadOnly)}.
    * <li>{@link #nextSphere3D(Random, Tuple3DReadOnly)}.
    * </ul>
    * </p>
    * <p>
    * This generator differs from {@link EuclidShapeRandomTools#nextShape3D(Random)} by excluding {@link Torus3D} that is a
    * concave shape.
    * </p>
    * 
    * @param random   the random generator to use.
    * @param position the position of the shape's centroid. Not modified.
    * @return the random convex shape 3D.
    */
   public static Shape3DBasics nextConvexShape3D(Random random, Tuple3DReadOnly position)
   {
      switch (random.nextInt(8))
      {
         case 0:
            return nextBox3D(random, position);
         case 1:
            return nextCapsule3D(random, position);
         case 2:
            return nextConvexPolytope3D(random, position);
         case 3:
            return nextCylinder3D(random, position);
         case 4:
            return nextEllipsoid3D(random, position);
         case 5:
            return new PointShape3D(position);
         case 6:
            return nextRamp3D(random, position);
         case 7:
            return nextSphere3D(random, position);
         default:
            throw new RuntimeException("Unexpected state.");
      }
   }
}
