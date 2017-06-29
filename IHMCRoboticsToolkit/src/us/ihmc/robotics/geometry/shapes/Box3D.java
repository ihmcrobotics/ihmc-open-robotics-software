package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TransformationTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.MathTools;

/**
 * {@code Box3D} represents an axis-aligned box with a length, a width, and a height.
 * <p>
 * Its origin is located at its centroid.
 * </p>
 */
public class Box3D extends Shape3D<Box3D>
{
   /** Internal point used to enable garbage free operations. */
   private final Point3D temporaryPoint = new Point3D();

   /**
    * Represents half the length, width, and height of this box. Note that this field automatically
    * updated from {@link #size}.
    */
   private final Size3D halfSize = new Size3D();
   /**
    * Represents the length, width, and height of this box. When changed, it automatically updates
    * the field {@link #halfSize}.
    */
   private final Size3D size = new Size3D()
   {
      private static final long serialVersionUID = 3115155959997000188L;

      @Override
      public final void setX(double x)
      {
         super.setX(x);
         halfSize.setX(0.5 * x);
      }

      @Override
      public final void setY(double y)
      {
         super.setY(y);
         halfSize.setY(0.5 * y);
      }

      @Override
      public final void setZ(double z)
      {
         super.setZ(z);
         halfSize.setZ(0.5 * z);
      }
   };

   /**
    * Creates a 1-by-1-by-1 box 3D.
    */
   public Box3D()
   {
      this(1.0, 1.0, 1.0);
   }

   /**
    * Creates a new box 3D identical to {@code other}.
    * 
    * @param other the other box to copy. Not modified.
    */
   public Box3D(Box3D other)
   {
      set(other);
   }

   /**
    * Creates a new box 3D and initializes its size.
    * 
    * @param length the size of this box along the x-axis.
    * @param width the size of this box along the y-axis.
    * @param height the size of this box along the z-axis.
    */
   public Box3D(double length, double width, double height)
   {
      setSize(length, width, height);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    * 
    * @param position the position of this box. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param length the size of this box along the x-axis.
    * @param width the size of this box along the y-axis.
    * @param height the size of this box along the z-axis.
    */
   public Box3D(Point3DReadOnly position, QuaternionReadOnly orientation, double length, double width, double height)
   {
      setPose(position, orientation);
      setSize(length, width, height);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    * 
    * @param pose the position and orientation of this box. Not modified.
    * @param length the size of this box along the x-axis.
    * @param width the size of this box along the y-axis.
    * @param height the size of this box along the z-axis.
    */
   public Box3D(Pose3D pose, double length, double width, double height)
   {
      setPose(pose);
      setSize(length, width, height);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    * 
    * @param pose the position and orientation of this box. Not modified.
    * @param length the size of this box along the x-axis.
    * @param width the size of this box along the y-axis.
    * @param height the size of this box along the z-axis.
    */
   public Box3D(RigidBodyTransform pose, double length, double width, double height)
   {
      setPose(pose);
      setSize(length, width, height);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    * 
    * @param pose the position and orientation of this box. Not modified.
    * @param size the size of this box along in order the x, y, and z axes.
    */
   public Box3D(RigidBodyTransform pose, double[] size)
   {
      this(pose, size[0], size[1], size[2]);
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return super.containsNaN() || size.containsNaN();
   }

   /**
    * Tests separately and on a per component basis if the pose and the size of this box and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    * 
    * @param other the other box which pose and size is to be compared against this box pose and
    *           size. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two boxes are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Box3D other, double epsilon)
   {
      return super.epsilonEqualsPose(other, epsilon) && size.epsilonEquals(other.size, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   protected double evaluateQuery(double x, double y, double z, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      boolean isInside = isInsideEpsilonShapeFrame(x, y, z, 0.0);

      if (isInside)
      {
         double dx = Math.abs(Math.abs(x) - halfSize.getX());
         double dy = Math.abs(Math.abs(y) - halfSize.getY());
         double dz = Math.abs(Math.abs(z) - halfSize.getZ());

         if (closestPointToPack != null)
         {
            closestPointToPack.set(x, y, z);

            if (dx < dy)
            {
               if (dx < dz)
                  closestPointToPack.setX(Math.copySign(halfSize.getX(), x));
               else
                  closestPointToPack.setZ(Math.copySign(halfSize.getZ(), z));
            }
            else
            {
               if (dy < dz)
                  closestPointToPack.setY(Math.copySign(halfSize.getY(), y));
               else
                  closestPointToPack.setZ(Math.copySign(halfSize.getZ(), z));
            }
         }

         if (normalToPack != null)
         {
            normalToPack.setToZero();

            if (dx < dy)
            {
               if (dx < dz)
                  normalToPack.setX(Math.copySign(1.0, x));
               else
                  normalToPack.setZ(Math.copySign(1.0, z));
            }
            else
            {
               if (dy < dz)
                  normalToPack.setY(Math.copySign(1.0, y));
               else
                  normalToPack.setZ(Math.copySign(1.0, z));
            }
         }

         return -EuclidCoreTools.min(dx, dy, dz);
      }
      else
      {

         double xClamped = MathTools.clamp(x, halfSize.getX());
         double yClamped = MathTools.clamp(y, halfSize.getY());
         double zClamped = MathTools.clamp(z, halfSize.getZ());

         double dx = x - xClamped;
         double dy = y - yClamped;
         double dz = z - zClamped;

         double distance = Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));

         if (closestPointToPack != null)
         {
            closestPointToPack.set(xClamped, yClamped, zClamped);
         }

         if (normalToPack != null)
         {
            normalToPack.set(dx, dy, dz);
            normalToPack.scale(1.0 / distance);
         }

         return distance;
      }
   }

   /**
    * Computes and packs the bounding box 3D that fully contains this box 3D.
    * <p>
    * This method does not take the size of this box to convert it into a bounding box, it actually
    * considers the pose of this box 3D such that if it has a non-zero orientation the bounding box
    * will be bigger than this box.
    * </p>
    * 
    * @param boundingBoxToPack the bounding box to pack. Modified.
    */
   public void getBoundingBox3D(BoundingBox3D boundingBoxToPack)
   {
      boundingBoxToPack.setToNaN();

      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
      {
         getVertex(vertexIndex, temporaryPoint);
         boundingBoxToPack.updateToIncludePoint(temporaryPoint);
      }
   }

   /**
    * Convenience method that packs the center of the box.
    * <p>
    * This method is equivalent to {@link #getPosition(Tuple3DBasics)}.
    * </p>
    * 
    * @param centerToPack the point in which the coordinates of the center of this box are stored.
    *           Modified.
    */
   public void getCenter(Point3DBasics centerToPack)
   {
      getPosition(centerToPack);
   }

   /**
    * Gets this box length, i.e. the size along the x-axis.
    * 
    * @return this box length.
    */
   public double getLength()
   {
      return size.getLength();
   }

   /**
    * Gets this box width, i.e. the size along the y-axis.
    * 
    * @return this box width.
    */
   public double getWidth()
   {
      return size.getWidth();
   }

   /**
    * Gets this box height, i.e. the size along the z-axis.
    * 
    * @return this box height.
    */
   public double getHeight()
   {
      return size.getHeight();
   }

   /**
    * Gets this box size along the x-axis, i.e. its length.
    * 
    * @return this box size along the x-axis.
    */
   public double getSizeX()
   {
      return size.getX();
   }

   /**
    * Gets this box size along the y-axis, i.e. its width.
    * 
    * @return this box size along the y-axis.
    */
   public double getSizeY()
   {
      return size.getY();
   }

   /**
    * Gets this box size along the z-axis, i.e. its height.
    * 
    * @return this box size along the z-axis.
    */
   public double getSizeZ()
   {
      return size.getZ();
   }

   /**
    * Packs the world coordinates of one of this box vertices.
    * 
    * @param vertexIndex the index in [0, 7] of the vertex to pack.
    * @param vertexToPack point in which the coordinates of the vertex are stored. Modified.
    * @throws IndexOutOfBoundsException if {@code vertexIndex} is not in [0, 7].
    */
   public void getVertex(int vertexIndex, Point3DBasics vertexToPack)
   {
      if (vertexIndex < 0 || vertexIndex >= 8)
         throw new IndexOutOfBoundsException("The vertex index has to be in [0, 7], was: " + vertexIndex);

      vertexToPack.setX((vertexIndex & 1) == 0 ? size.getX() : -size.getX());
      vertexToPack.setY((vertexIndex & 2) == 0 ? size.getY() : -size.getY());
      vertexToPack.setZ((vertexIndex & 4) == 0 ? size.getZ() : -size.getZ());
      vertexToPack.scale(0.5);
      transformToWorld(vertexToPack);
   }

   /**
    * Gets the 8 vertices, expressed in world, of this box as an array.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @return an array of 8 {@code Point3D} with this box vertices.
    */
   public Point3D[] getVertices()
   {
      Point3D[] vertices = new Point3D[8];
      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         getVertex(vertexIndex, vertices[vertexIndex] = new Point3D());
      return vertices;
   }

   /**
    * Pack the coordinates in world of the 8 vertices of this box in the given array.
    * 
    * @param verticesToPack the array in which the coordinates are stored. Modified.
    * @throws IllegalArgumentException if the length of the given array is different than 8.
    * @throws NullPointerException if any of the 8 first elements of the given array is
    *            {@code null}.
    */
   public void getVertices(Point3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 8)
         throw new IllegalArgumentException("Array is too small, has to be at least 8 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         getVertex(vertexIndex, verticesToPack[vertexIndex]);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    * 
    * @param line the line expressed in world coordinates that may intersect this box. Not modified.
    * @param firstIntersectionToPack the coordinate in world of the first intersection. Can be
    *           {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *           {@code null}. Modified.
    * @return the number of intersections between the line and this box. It is either equal to 0, 1,
    *         or 2.
    */
   public int intersectionWith(Line3D line, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this box.
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
    * @return the number of intersections between the line and this box. It is either equal to 0, 1,
    *         or 2.
    */
   public int intersectionWith(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, Point3DBasics firstIntersectionToPack,
                               Point3DBasics secondIntersectionToPack)
   {
      double minX = -halfSize.getX();
      double minY = -halfSize.getY();
      double minZ = -halfSize.getZ();
      double maxX = halfSize.getX();
      double maxY = halfSize.getY();
      double maxZ = halfSize.getZ();

      double xLocal = TransformationTools.computeTransformedX(shapePose, true, pointOnLine);
      double yLocal = TransformationTools.computeTransformedY(shapePose, true, pointOnLine);
      double zLocal = TransformationTools.computeTransformedZ(shapePose, true, pointOnLine);

      double dxLocal = TransformationTools.computeTransformedX(shapePose, true, lineDirection);
      double dyLocal = TransformationTools.computeTransformedY(shapePose, true, lineDirection);
      double dzLocal = TransformationTools.computeTransformedZ(shapePose, true, lineDirection);

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D(minX, minY, minZ, maxX, maxY, maxZ, xLocal, yLocal, zLocal,
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
      return Math.abs(x) <= halfSize.getX() + epsilon && Math.abs(y) <= halfSize.getY() + epsilon && Math.abs(z) <= halfSize.getZ() + epsilon;
   }

   /**
    * Applies the given scale factor to the size of this box.
    * 
    * @param scale the scale factor to use.
    * @throws IllegalArgumentException if {@code scale} is negative.
    */
   public void scale(double scale)
   {
      if (scale < 0.0)
         throw new IllegalArgumentException("Cannot apply a negative scale: " + scale);
      size.scale(scale);
   }

   /**
    * Copies the {@code other} box data into {@code this}.
    * 
    * @param other the other box to copy. Not modified.
    */
   @Override
   public void set(Box3D other)
   {
      setPose(other);
      size.set(other.size);
   }

   /**
    * Sets the size of this box.
    * 
    * @param length the size of this box along the x-axis.
    * @param width the size of this box along the y-axis.
    * @param height the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of the three arguments is negative.
    */
   public void setSize(double length, double width, double height)
   {
      if (length < 0.0)
         throw new IllegalArgumentException("A Box3D cannot have a negative length: " + length);
      if (width < 0.0)
         throw new IllegalArgumentException("A Box3D cannot have a negative width: " + width);
      if (height < 0.0)
         throw new IllegalArgumentException("A Box3D cannot have a negative height: " + height);

      size.setLengthWidthHeight(length, width, height);
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      super.setToNaN();
      size.setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   public void setToZero()
   {
      super.setToZero();
      size.setToZero();
   }

   /**
    * Provides a {@code String} representation of this box 3D as follows:<br>
    * Box 3D: size = (length, width, height), pose = <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this box 3D.
    */
   @Override
   public String toString()
   {
      return "Box 3D: size = " + size + ", pose =\n" + getPoseString();
   }
}
