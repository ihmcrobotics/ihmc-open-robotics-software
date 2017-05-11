package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonTools.EmptyPolygonException;
import us.ihmc.robotics.geometry.ConvexPolygonTools.OutdatedPolygonException;

/**
 * Describes a planar convex polygon defined in the XY-plane.
 * <p>
 * The vertices of a convex polygon are clockwise and are all different.
 * </p>
 * <p>
 * This implementation of convex polygon is designed for garbage free operations.
 * </p>
 */
public class ConvexPolygon2d implements GeometryObject<ConvexPolygon2d>
{
   /**
    * Field for future expansion of {@code ConvexPolygon2d} to enable having the vertices in
    * clockwise or counter-clockwise ordered.
    */
   private final boolean clockwiseOrdered = true;
   /**
    * The current size or number of vertices for this convex polygon.
    */
   private int numberOfVertices = 0;
   /**
    * The internal memory of {@code ConvexPolygon2d}.
    * <p>
    * New vertices can be added to this polygon, after which the method {@link #update()} has to be
    * called to ensure that this polygon is convex.
    * </p>
    * <p>
    * Note that this list is used as a buffer to recycle the memory and can thus be greater than the
    * actual size of this polygon.
    * </p>
    * <p>
    * The vertices composing this polygon are located in the index range [0,
    * {@link #numberOfVertices}[ in this list.
    * </p>
    */
   private final List<Point2D> clockwiseOrderedVertices = new ArrayList<Point2D>();
   /**
    * The smallest axis-aligned bounding box that contains all this polygon's vertices.
    * <p>
    * It is updated in the method {@link #updateBoundingBox()} which is itself called in
    * {@link #update()}.
    * </p>
    */
   private final BoundingBox2D boundingBox = new BoundingBox2D();
   /**
    * The centroid of this polygon which is located at the center of mass of this polygon when
    * considered as a physical object with constant thickness and density.
    * <p>
    * It is updated in the method {@link #updateCentroidAndArea()} which is itself called in
    * {@link #update()}.
    * </p>
    */
   private final Point2D centroid = new Point2D();
   /**
    * The area of this convex polygon.
    * <p>
    * It is updated in the method {@link #updateCentroidAndArea()} which is itself called in
    * {@link #update()}.
    * </p>
    * <p>
    * When a polygon is empty, i.e. has no vertices, the area is equal to {@link Double#NaN}.
    * </p>
    */
   private double area;
   /**
    * This field is used to know whether the method {@link #update()} has been called since the last
    * time the vertices of this polygon have been modified.
    * <p>
    * Most operations with a polygon require the polygon to be up-to-date.
    * </p>
    */
   private boolean isUpToDate = false;

   /** Index of the vertex with the lowest x-coordinate. */
   private int minX_index = 0;
   /** Index of the vertex with the highest x-coordinate. */
   private int maxX_index = 0;
   /** Index of the vertex with the lowest y-coordinate. */
   private int minY_index = 0;
   /** Index of the vertex with the highest y-coordinate. */
   private int maxY_index = 0;
   /**
    * Index of the vertex with the lowest x-coordinate. If the lowest x-coordinate exists in more
    * than one vertex in the list, it is the index of the vertex with the highest y-coordinate out
    * of the candidates.
    * <p>
    * Note that the method {@link #update()} will always position this vertex at the index 0, so it
    * does not need to be updated.
    * </p>
    */
   private final int minXmaxY_index = 0;
   /**
    * Index of the vertex with the lowest x-coordinate. If the lowest x-coordinate exists in more
    * than one vertex in the list, it is the index of the vertex with the lowest y-coordinate out of
    * the candidates.
    */
   private int minXminY_index = 0;
   /**
    * Index of the vertex with the highest x-coordinate. If the highest x-coordinate exists in more
    * than one vertex in the list, it is the index of the vertex with the lowest y-coordinate out of
    * the candidates.
    */
   private int maxXminY_index = 0;
   /**
    * Index of the vertex with the highest x-coordinate. If the highest x-coordinate exists in more
    * than one vertex in the list, it is the index of the vertex with the highest y-coordinate out
    * of the candidates.
    */
   private int maxXmaxY_index = 0;

   /**
    * Creates an empty convex polygon.
    */
   public ConvexPolygon2d()
   {
      numberOfVertices = 0;
      update();
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of the points from the
    * given {@code vertices} list that are in the index range [0, {@code numberOfVertices}[.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    * 
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the list. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public ConvexPolygon2d(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      setAndUpdate(vertices, numberOfVertices);
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of the points from the
    * given {@code vertices} list.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    * 
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    */
   public ConvexPolygon2d(List<? extends Point2DReadOnly> vertices)
   {
      this(vertices, vertices.size());
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of the points from the
    * given {@code vertices} array that are in the index range [0, {@code numberOfVertices}[.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    * 
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given array of vertices.
    */
   public ConvexPolygon2d(Point2DReadOnly[] vertices, int numberOfVertices)
   {
      setAndUpdate(vertices, numberOfVertices);
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of the points from the
    * given {@code vertices} array.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    * 
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    */
   public ConvexPolygon2d(Point2DReadOnly[] vertices)
   {
      this(vertices, vertices.length);
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of the points from the
    * given {@code vertices} array that are in the index range [0, {@code numberOfVertices}[.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    * 
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Each row
    *           contains one point whereas the (at least) two columns contains in order the
    *           coordinates x and y. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given array of vertices.
    */
   public ConvexPolygon2d(double[][] vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * Creates a new convex polygon such that it represents the convex hull of the points from the
    * given {@code vertices} array.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    * 
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Each row
    *           contains one point whereas the (at least) two columns contains in order the
    *           coordinates x and y. Not modified.
    */
   public ConvexPolygon2d(double[][] vertices)
   {
      this(vertices, vertices.length);
   }

   /**
    * Copy constructor.
    * 
    * @param otherPolygon the other convex polygon to copy. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygon's vertices were edited.
    */
   public ConvexPolygon2d(ConvexPolygon2d otherPolygon)
   {
      setAndUpdate(otherPolygon);
   }

   /**
    * Creates a new convex polygon by combining two other convex polygons. The result is the
    * smallest convex hull that contains both polygons.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    *
    * @param firstPolygon the first convex polygon to combine. Not modified.
    * @param secondPolygon the second convex polygon to combine. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygons' vertices were edited.
    */
   public ConvexPolygon2d(ConvexPolygon2d firstPolygon, ConvexPolygon2d secondPolygon)
   {
      setAndUpdate(firstPolygon, secondPolygon);
   }

   /**
    * Generates a random convex polygon given the maximum absolute coordinate value of its vertices
    * and the size of the point cloud from which it is generated.
    * 
    * @param random the random generator to use.
    * @param maxAbsoluteXY the maximum absolute value for each coordinate of the vertices.
    * @param numberOfPossiblePoints the size of the point cloud to generate that is used for
    *           computing the random convex polygon. The size of the resulting convex polygon will
    *           be less than {@code numberOfPossiblePoints}.
    * @return the random convex polygon.
    * @throws RuntimeException if {@code maxAbsoluteXY < 0}.
    */
   public static ConvexPolygon2d generateRandomConvexPolygon2d(Random random, double maxAbsoluteXY, int numberOfPossiblePoints)
   {
      List<Point2D> vertices = EuclidGeometryRandomTools.generateRandomPointCloud2D(random, 0.0, maxAbsoluteXY, numberOfPossiblePoints);
      return new ConvexPolygon2d(vertices);
   }

   /**
    * After calling this method, the polygon has no vertex, area, or centroid.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    */
   public void clear()
   {
      numberOfVertices = 0;
      area = Double.NaN;
      centroid.set(Double.NaN, Double.NaN);
      isUpToDate = false;
   }

   /**
    * After calling this method, the polygon has no vertex, area, or centroid.
    * <p>
    * Use only when an empty polygon is desired.
    * </p>
    */
   public void clearAndUpdate()
   {
      clear();
      isUpToDate = true;
   }

   /**
    * Clears this polygon, adds a single vertex at (0, 0), and updates it.
    */
   @Override
   public void setToZero()
   {
      clear();
      addVertex(0.0, 0.0);
      update();
   }

   /**
    * Clears this polygon, adds a single vertex at ({@link Double#NaN}, {@link Double#NaN}), and
    * updates it.
    */
   @Override
   public void setToNaN()
   {
      clear();
      addVertex(Double.NaN, Double.NaN);
      update();
   }

   /**
    * Tests if any of this polygon's vertices contains a {@link Double#NaN}.
    * 
    * @return {@code true} if at least one vertex contains {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   @Override
   public boolean containsNaN()
   {
      update();

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D point = clockwiseOrderedVertices.get(i);

         if (point.containsNaN())
            return true;
      }

      return false;
   }

   /**
    * Add a vertex to this polygon.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    * 
    * @param vertex the new vertex. Not modified.
    */
   public void addVertex(Point2DReadOnly vertex)
   {
      addVertex(vertex.getX(), vertex.getY());
   }

   /**
    * Add a vertex to this polygon.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    * 
    * @param x the x-coordinate of the new vertex.
    * @param y the y-coordinate of the new vertex.
    */
   public void addVertex(double x, double y)
   {
      isUpToDate = false;
      setOrCreate(x, y, numberOfVertices);
      numberOfVertices++;
   }

   /**
    * Adds the N first vertices from the given list to this polygon, where N is equal to
    * {@code numberOfVertices}.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    * 
    * @param vertices the list containing the vertices to add to this polygon. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the list. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public void addVertices(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > vertices.size())
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + vertices.size() + "].");

      for (int i = 0; i < numberOfVertices; i++)
         addVertex(vertices.get(i));
   }

   /**
    * Adds the N first vertices from the given array to this polygon, where N is equal to
    * {@code numberOfVertices}.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    * 
    * @param vertices the array containing the vertices to add to this polygon. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given array of vertices.
    */
   public void addVertices(Point2DReadOnly[] vertices, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > vertices.length)
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + vertices.length + "].");

      for (int i = 0; i < numberOfVertices; i++)
         addVertex(vertices[i]);
   }

   /**
    * Adds the N first vertices from the given array to this polygon, where N is equal to
    * {@code numberOfVertices}.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    * 
    * @param vertices the array containing the vertices to add to this polygon. Each row contains
    *           one point whereas the (at least) two columns contains in order the coordinates x and
    *           y. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given array of vertices.
    */
   public void addVertices(double[][] vertices, int numberOfVertices)
   {
      if (numberOfVertices < 0 || numberOfVertices > vertices.length)
         throw new IllegalArgumentException("Illegal numberOfVertices: " + numberOfVertices + ", expected a value in ] 0, " + vertices.length + "].");

      for (int i = 0; i < numberOfVertices; i++)
         addVertex(vertices[i][0], vertices[i][1]);
   }

   /**
    * Adds new vertices to this polygon from another convex polygon.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    * 
    * @param otherPolygon the other convex polygon that is used to add new vertices to this polygon.
    *           Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygon's vertices were edited.
    */
   public void addVertices(ConvexPolygon2d otherPolygon)
   {
      for (int i = 0; i < otherPolygon.getNumberOfVertices(); i++)
         addVertex(otherPolygon.getVertex(i));
   }

   /**
    * Removes the vertex of this polygon positioned at the index {@code indexOfVertexToRemove}.
    * <p>
    * Note that this polygon is marked as being out-of-date. The method {@link #update()} has to be
    * called afterward before being able to perform operations with this polygon.
    * </p>
    * 
    * @param indexOfVertexToRemove the index of the vertex to remove.
    * @throws EmptyPolygonException if this polygon is empty before calling this method.
    * @throws IndexOutOfBoundsException if the given index is either negative or greater or equal
    *            than the polygon's number of vertices.
    */
   public void removeVertex(int indexOfVertexToRemove)
   {
      checkNonEmpty();
      checkIndexInBoundaries(indexOfVertexToRemove);

      if (indexOfVertexToRemove == numberOfVertices - 1)
      {
         numberOfVertices--;
         return;
      }
      isUpToDate = false;
      Collections.swap(clockwiseOrderedVertices, indexOfVertexToRemove, numberOfVertices - 1);
      numberOfVertices--;
   }

   /**
    * Method for internal use only.
    * <p>
    * Sets the {@code i}<sup>th</sup> point in {@link #clockwiseOrderedVertices} to the given point.
    * The list is extended if needed.
    * </p>
    * 
    * @param x the x-coordinate of the point to copy. Not modified.
    * @param y the y-coordinate of the point to copy. Not modified.
    * @param i the position in the list {@link #clockwiseOrderedVertices} to copy the given point.
    */
   private void setOrCreate(double x, double y, int i)
   {
      while (i >= clockwiseOrderedVertices.size())
         clockwiseOrderedVertices.add(new Point2D());
      clockwiseOrderedVertices.get(i).set(x, y);
   }

   /**
    * Updates the vertices so they represent a clockwise convex polygon.
    * <p>
    * Call this method after editing the vertices of this polygon.
    * </p>
    * <p>
    * Note that this also updates centroid, area and the bounding box of this polygon.
    * </p>
    */
   public void update()
   {
      if (isUpToDate)
         return;

      numberOfVertices = EuclidGeometryPolygonTools.inPlaceGiftWrapConvexHull2D(clockwiseOrderedVertices, numberOfVertices);
      isUpToDate = true;

      updateCentroidAndArea();
      updateBoundingBox();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(List, int)}.
    * <li>{@link #update()}.
    * </ol>
    * 
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the list. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    */
   public void setAndUpdate(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(Point2DReadOnly[], int)}.
    * <li>{@link #update()}.
    * </ol>
    * 
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given array of vertices.
    */
   public void setAndUpdate(Point2DReadOnly[] vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(double[][], int)}.
    * <li>{@link #update()}.
    * </ol>
    * 
    * @param vertices the 2D point cloud from which the convex hull is to be computed. Each row
    *           contains one point whereas the (at least) two columns contains in order the
    *           coordinates x and y. Not modified.
    * @param numberOfVertices specifies the number of relevant points in the array. Only the points
    *           &in; [0; {@code numberOfVertices}[ are processed.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given array of vertices.
    */
   public void setAndUpdate(double[][] vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(ConvexPolygon2d)}.
    * <li>{@link #update()}.
    * </ol>
    * 
    * @param otherPolygon the other convex polygon to copy. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygon's vertices were edited.
    */
   @Override
   public void set(ConvexPolygon2d other)
   {
      setAndUpdate(other);
   }

   /**
    * This method does:
    * <ol>
    * <li>{@link #clear()}.
    * <li>{@link #addVertices(ConvexPolygon2d)}.
    * <li>{@link #update()}.
    * </ol>
    * 
    * @param otherPolygon the other convex polygon to copy. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygon's vertices were edited.
    */
   // TODO There is no need to call update() there, instead update everything from the other polygon to make it faster.
   public void setAndUpdate(ConvexPolygon2d otherPolygon)
   {
      clear();
      addVertices(otherPolygon);
      update();
   }

   /**
    * Sets this polygon such that it represents the smallest convex hull that contains both
    * polygons.
    * <p>
    * Note that the resulting polygon is ready to be used for any operations, no need to call
    * {@link #update()}.
    * </p>
    * 
    * @param firstPolygon the first convex polygon to combine. Not modified.
    * @param secondPolygon the second convex polygon to combine. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time the
    *            other polygons' vertices were edited.
    */
   // TODO: Make this more efficient by finding the rotating calipers, as in the intersection method.
   public void setAndUpdate(ConvexPolygon2d firstPolygon, ConvexPolygon2d secondPolygon)
   {
      clear();
      addVertices(firstPolygon);
      addVertices(secondPolygon);
      update();
   }

   /**
    * Updates the bounding box properties.
    */
   private void updateBoundingBox()
   {
      minX_index = 0;
      maxX_index = 0;
      minY_index = 0;
      maxY_index = 0;
      minXminY_index = 0;
      maxXmaxY_index = 0;
      maxXminY_index = 0;

      if (!isEmpty())
      {
         Point2D firstVertex = getVertexUnsafe(0);
         double minX = firstVertex.getX();
         double minY = firstVertex.getY();
         double maxX = firstVertex.getX();
         double maxY = firstVertex.getY();

         Point2D p;
         for (int i = 1; i < numberOfVertices; i++)
         {
            p = getVertexUnsafe(i);

            if (p.getX() < minX)
            {
               minX = p.getX();
               minX_index = i;
               minXminY_index = i;
            }
            else if (p.getX() > maxX)
            {
               maxX = p.getX();
               maxX_index = i;
               maxXmaxY_index = i;
               maxXminY_index = i;
            }
            else if (p.getX() == getVertex(minXminY_index).getX() && p.getY() < getVertex(minXminY_index).getY())
            {
               minXminY_index = i;
            }
            else if (p.getX() == getVertex(maxXminY_index).getX()) // any case: getVertex(maxXmaxY_index).x == getVertex(maxXminY_index).x
            {
               if (p.getY() < getVertex(maxXminY_index).getY())
               {
                  maxXminY_index = i;
               }
               else if (p.getY() > getVertex(maxXmaxY_index).getY())
               {
                  maxXmaxY_index = i;
               }
            }

            if (p.getY() <= minY)
            {
               minY = p.getY();
               minY_index = i;
            }
            else if (p.getY() >= maxY)
            {
               maxY = p.getY();
               maxY_index = i;
            }
         }
         boundingBox.set(minX, minY, maxX, maxY);
      }
      else
      {
         boundingBox.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
      }
   }

   /**
    * Compute centroid and area of this polygon. Formula taken from
    * <a href="http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/">here</a>.
    */
   private void updateCentroidAndArea()
   {
      area = EuclidGeometryPolygonTools.computeConvexPolyong2DArea(clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered, centroid);
   }

   /**
    * Gets the value of this polygon area.
    * 
    * @return the are of this polygon.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public double getArea()
   {
      checkIfUpToDate();
      return area;
   }

   /**
    * Gets this polygon's centroid coordinates.
    * <p>
    * The centroid is not necessarily equal to the average of this polygon's vertices.
    * </p>
    * <p>
    * When viewing a polygon as a physical object with constant density and thickness, the centroid
    * is equivalent to the polygon's center of mass.
    * </p>
    * 
    * @param centroidToPack the point in which the coordinates of the centroid are stored. Modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public void getCentroid(Point2DBasics centroidToPack)
   {
      checkIfUpToDate();
      centroidToPack.set(this.centroid);
   }

   /**
    * Gets the read-only reference to the polygon's centroid.
    * <p>
    * The centroid is not necessarily equal to the average of this polygon's vertices.
    * </p>
    * <p>
    * When viewing a polygon as a physical object with constant density and thickness, the centroid
    * is equivalent to the polygon's center of mass.
    * </p>
    * 
    * @return the read-only reference to this polygon's centroid.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public Point2DReadOnly getCentroid()
   {
      checkIfUpToDate();
      return centroid;
   }

   /**
    * Gets the internal reference to this polygon's axis-aligned bounding box.
    * 
    * @return this polygon's bounding box.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public BoundingBox2D getBoundingBox()
   {
      checkIfUpToDate();
      return boundingBox;
   }

   /**
    * Gets the size along the x-axis of this polygon's bounding box.
    * 
    * @return the range on the x-axis of the bounding box.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public double getBoundingBoxRangeX()
   {
      checkIfUpToDate();
      return boundingBox.getMaxPoint().getX() - boundingBox.getMinPoint().getX();
   }

   /**
    * Gets the size along the y-axis of this polygon's bounding box.
    * 
    * @return the range on the y-axis of the bounding box.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public double getBoundingBoxRangeY()
   {
      checkIfUpToDate();
      return boundingBox.getMaxPoint().getY() - boundingBox.getMinPoint().getY();
   }

   /**
    * Gets a copy of this polygon's axis-aligned bounding box.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @return the copy of this polygon's bounding box.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public BoundingBox2D getBoundingBoxCopy()
   {
      checkIfUpToDate();
      BoundingBox2D ret = new BoundingBox2D(boundingBox);

      return ret;
   }

   /**
    * Packs this polygon's axis-aligned bounding box in the given {@code boundingBoxToPack}.
    * 
    * @param boundingBoxToPack the bounding box that is set to this polygon's bounding box.
    *           Modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public void getBoundingBox(BoundingBox2D boundingBoxToPack)
   {
      checkIfUpToDate();
      boundingBoxToPack.set(boundingBox);
   }

   /**
    * Gets the read-only reference to the {@code index}<sup>th</sup> vertex of this polygon.
    * <p>
    * Note that this polygon's vertices are clockwise ordered and that the first vertex has the
    * lowest x-coordinate.
    * </p>
    * 
    * @param index the index of the vertex in the clockwise ordered list.
    * @return the read-only reference to the vertex.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public Point2DReadOnly getVertex(int index)
   {
      checkIfUpToDate();
      return getVertexUnsafe(index);
   }

   /**
    * Same as {@link #getVertex(int)} but without checking if the polygon has been updated.
    * <p>
    * For internal use only.
    * </p>
    */
   private Point2D getVertexUnsafe(int vertexIndex)
   {
      checkNonEmpty();
      checkIndexInBoundaries(vertexIndex);
      return clockwiseOrderedVertices.get(vertexIndex);
   }

   /**
    * Gets the read-only reference to the vertex located after the {@code index}<sup>th</sup> vertex
    * of this polygon.
    * <p>
    * Note that this polygon's vertices are clockwise ordered and that the first vertex has the
    * lowest x-coordinate.
    * </p>
    * 
    * @param index the index of the vertex in the clockwise ordered list.
    * @return the read-only reference to the next vertex.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public Point2DReadOnly getNextVertex(int index)
   {
      return getVertex(getNextVertexIndex(index));
   }

   /**
    * Gets the read-only reference to the vertex located before the {@code index}<sup>th</sup>
    * vertex of this polygon.
    * <p>
    * Note that this polygon's vertices are clockwise ordered and that the first vertex has the
    * lowest x-coordinate.
    * </p>
    * 
    * @param index the index of the vertex in the clockwise ordered list.
    * @return the read-only reference to the previous vertex.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public Point2DReadOnly getPreviousVertex(int index)
   {
      return getVertex(getPreviousVertexIndex(index));
   }

   /**
    * Gets the read-only reference to the {@code index}<sup>th</sup> vertex of this polygon.
    * <p>
    * This method calculates a new index to emulate a counter-clockwise ordering of this polygon's
    * vertices. The first vertex has the lowest x-coordinate.
    * </p>
    * 
    * @param index the index of the vertex in the counter-clockwise ordered list.
    * @return the read-only reference to the vertex.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public Point2DReadOnly getVertexCCW(int index)
   {
      checkIfUpToDate();
      checkNonEmpty();
      checkIndexInBoundaries(index);
      return clockwiseOrderedVertices.get(numberOfVertices - 1 - index);
   }

   /**
    * Gets the read-only reference to the vertex located after the {@code index}<sup>th</sup> vertex
    * of this polygon.
    * <p>
    * This method calculates a new index to emulate a counter-clockwise ordering of this polygon's
    * vertices. The first vertex has the lowest x-coordinate.
    * </p>
    * 
    * @param index the index of the vertex in the counter-clockwise ordered list.
    * @return the read-only reference to the next vertex.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public Point2DReadOnly getNextVertexCCW(int index)
   {
      return getVertexCCW(getNextVertexIndex(index));
   }

   /**
    * Gets the read-only reference to the vertex located before the {@code index}<sup>th</sup>
    * vertex of this polygon.
    * <p>
    * This method calculates a new index to emulate a counter-clockwise ordering of this polygon's
    * vertices. The first vertex has the lowest x-coordinate.
    * </p>
    * 
    * @param index the index of the vertex in the counter-clockwise ordered list.
    * @return the read-only reference to the previous vertex.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public Point2DReadOnly getPreviousVertexCCW(int index)
   {
      return getVertexCCW(getPreviousVertexIndex(index));
   }

   /**
    * Gets the index of the vertex located after the vertex at the index {@code currentVertexIndex}
    * in the list of vertices.
    * <p>
    * Note that this polygon's vertices are clockwise ordered and that the first vertex has the
    * lowest x-coordinate.
    * </p>
    * 
    * @param currentVertexIndex the current vertex index.
    * @return the next vertex index.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public int getNextVertexIndex(int currentVertexIndex)
   {
      checkIfUpToDate();
      checkIndexInBoundaries(currentVertexIndex);
      return getNextVertexIndexUnsafe(currentVertexIndex);
   }

   /**
    * Same as {@link #getNextVertexIndex(int)} but without checking if the polygon has been updated.
    * <p>
    * For internal use only.
    * </p>
    * 
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   private int getNextVertexIndexUnsafe(int currentVertexIndex)
   {
      checkNonEmpty();

      if (currentVertexIndex < numberOfVertices - 1)
         return currentVertexIndex + 1;
      else
         return 0;
   }

   /**
    * Gets the index of the vertex located before the vertex at the index {@code currentVertexIndex}
    * in the list of vertices.
    * <p>
    * Note that this polygon's vertices are clockwise ordered and that the first vertex has the
    * lowest x-coordinate.
    * </p>
    * 
    * @param currentVertexIndex the current vertex index.
    * @return the before vertex index.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public int getPreviousVertexIndex(int currentVertexIndex)
   {
      checkIfUpToDate();
      checkIndexInBoundaries(currentVertexIndex);
      return getPreviousVertexIndexUnsafe(currentVertexIndex);
   }

   /**
    * Same as {@link #getPreviousVertexIndex(int)} but without checking if the polygon has been
    * updated.
    * <p>
    * For internal use only.
    * </p>
    * 
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   private int getPreviousVertexIndexUnsafe(int currentVertexIndex)
   {
      checkNonEmpty();

      if (currentVertexIndex < 1)
         return numberOfVertices - 1;
      else
         return currentVertexIndex - 1;
   }

   /**
    * Gets the number of vertices composing this convex polygon.
    * 
    * @return this polygon's size.
    */
   public int getNumberOfVertices()
   {
      return numberOfVertices;
   }

   /**
    * Scale this convex polygon about its centroid.
    * <p>
    * The polygon centroid remains unchanged.
    * </p>
    * 
    * @param scaleFactor the scale factor to apply to this polygon. A value of {@code 1.0} does
    *           nothing.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public void scale(double scaleFactor)
   {
      scale(centroid, scaleFactor);
   }

   /**
    * Scale this convex polygon about {@code pointToScaleAbout}.
    * <p>
    * This method effectively modifies the vertices of this polygon such that the scale is applied
    * on the distance between each vertex and the given {@code pointToScaleAbout}.
    * </p>
    * <p>
    * If {@code pointToScaleAbout} is equal to a vertex of this polygon, the coordinates of this
    * vertex will remain unmodified.
    * </p>
    * 
    * @param scaleFactor the scale factor to apply to this polygon. A value of {@code 1.0} does
    *           nothing.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public void scale(Point2DReadOnly pointToScaleAbout, double scaleFactor)
   {
      checkIfUpToDate();
      isUpToDate = false;

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = getVertexUnsafe(i);
         vertex.sub(pointToScaleAbout);
         vertex.scale(scaleFactor);
         vertex.add(pointToScaleAbout);
      }

      update();
   }

   /**
    * Translates this polygon.
    * 
    * @param translation the translation to apply to this polygon's vertices. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public void translate(Tuple2DReadOnly translation)
   {
      translate(translation.getX(), translation.getY());
   }

   /**
    * Translated this polygon.
    * 
    * @param x the translation along the x-axis to apply to each of this polygon's vertices.
    * @param y the translation along the y-axis to apply to each of this polygon's vertices.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public void translate(double x, double y)
   {
      checkIfUpToDate();

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = getVertexUnsafe(i);
         vertex.add(x, y);
      }

      updateBoundingBox();
      updateCentroidAndArea();
   }

   /**
    * Copies this polygon, translates the copy, and returns it.
    * 
    * @param translation the translation to apply to the copy of this polygon. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public ConvexPolygon2d translateCopy(Tuple2DReadOnly translation)
   {
      ConvexPolygon2d copy = new ConvexPolygon2d(this);
      copy.translate(translation);
      return copy;
   }

   /**
    * Creates and returns a representative {@code String} for this polygon.
    */
   @Override
   public String toString()
   {
      String ret = "";

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = clockwiseOrderedVertices.get(i);
         ret = ret + "(" + vertex.getX() + ", " + vertex.getY() + ")," + "\n";
      }

      return ret;
   }

   /**
    * Transforms this convex polygon using the given homogeneous transformation matrix.
    * 
    * @param transform the transform to apply on the vertices of this convex polygon. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a
    *            transformation in the XY-plane.
    */
   @Override
   public void applyTransform(Transform transform)
   {
      checkIfUpToDate();
      isUpToDate = false;

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = getVertexUnsafe(i);
         vertex.applyTransform(transform);
      }
      update();
   }

   /**
    * Transforms this convex polygon using the given homogeneous transformation matrix and project
    * the result onto the XY-plane.
    * 
    * @param transform the transform to apply on the vertices of this convex polygon. Not modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      checkIfUpToDate();
      isUpToDate = false;

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = getVertexUnsafe(i);
         vertex.applyTransform(transform, false);
      }
      update();
   }

   /**
    * Creates a copy of this polygon, transforms it using {@link #applyTransform(Transform)}, and
    * returns the copy.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param transform the transform to apply on the vertices of the copy of this convex polygon.
    *           Not modified.
    * @param the copy of this transformed.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a
    *            transformation in the XY-plane.
    */
   public ConvexPolygon2d applyTransformCopy(Transform transform)
   {
      ConvexPolygon2d copy = new ConvexPolygon2d(this);
      copy.applyTransform(transform);
      return copy;
   }

   /**
    * Creates a copy of this polygon, transforms it using
    * {@link #applyTransformAndProjectToXYPlane(Transform)}, and returns the copy.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * 
    * @param transform the transform to apply on the vertices of the copy of this convex polygon.
    *           Not modified.
    * @param the copy of this transformed.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public ConvexPolygon2d applyTransformAndProjectToXYPlaneCopy(Transform transform)
   {
      ConvexPolygon2d copy = new ConvexPolygon2d(this);
      copy.applyTransformAndProjectToXYPlane(transform);
      return copy;
   }

   /**
    * Gets the highest x-coordinate value of the vertices composing this polygon.
    * 
    * @return the maximum x-coordinate.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public double getMaxX()
   {
      return getVertex(maxX_index).getX();
   }

   /**
    * Gets the lowest x-coordinate value of the vertices composing this polygon.
    * 
    * @return the minimum x-coordinate.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public double getMinX()
   {
      return getVertex(minX_index).getX();
   }

   /**
    * Gets the highest y-coordinate value of the vertices composing this polygon.
    * 
    * @return the maximum y-coordinate.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public double getMaxY()
   {
      return getVertex(maxY_index).getY();
   }

   /**
    * Gets the lowest y-coordinate value of the vertices composing this polygon.
    * 
    * @return the minimum y-coordinate.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public double getMinY()
   {
      return getVertex(minY_index).getY();
   }

   /**
    * Gets the index of the vertex with the lowest x-coordinate.
    * 
    * @return the index of the vertex located at the minimum x.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public int getMinXIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return minX_index;
   }

   /**
    * Gets the index of the vertex with the highest x-coordinate.
    * 
    * @return the index of the vertex located at the maximum x.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public int getMaxXIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return maxX_index;
   }

   /**
    * Gets the index of the vertex with the lowest y-coordinate.
    * 
    * @return the index of the vertex located at the minimum y.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public int getMinYIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return minY_index;
   }

   /**
    * Gets the index of the vertex with the highest y-coordinate.
    * 
    * @return the index of the vertex located at the maximum y.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public int getMaxYIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return maxY_index;
   }

   /**
    * Gets the index of the vertex with the lowest x-coordinate. If the lowest x-coordinate exists
    * in more than one vertex in the list, it is the index of the vertex with the highest
    * y-coordinate out of the candidates.
    * 
    * @return the index of the vertex located at the minimum x and maximum y.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public int getMinXMaxYIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return minXmaxY_index;
   }

   /**
    * Gets the index of the vertex with the lowest x-coordinate. If the lowest x-coordinate exists
    * in more than one vertex in the list, it is the index of the vertex with the lowest
    * y-coordinate out of the candidates.
    * 
    * @return the index of the vertex located at the minimum x and minimum y.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public int getMinXMinYIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return minXminY_index;
   }

   /**
    * Gets the index of the vertex with the highest x-coordinate. If the highest x-coordinate exists
    * in more than one vertex in the list, it is the index of the vertex with the highest
    * y-coordinate out of the candidates.
    * 
    * @return the index of the vertex located at the maximum x and maximum y.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public int getMaxXMaxYIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return maxXmaxY_index;
   }

   /**
    * Gets the index of the vertex with the highest x-coordinate. If the highest x-coordinate exists
    * in more than one vertex in the list, it is the index of the vertex with the lowest
    * y-coordinate out of the candidates.
    * 
    * @return the index of the vertex located at the maximum x and minimum y.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public int getMaxXMinYIndex()
   {
      checkIfUpToDate();
      checkNonEmpty();
      return maxXminY_index;
   }

   /**
    * Adds a subset of this polygon's vertices into the given list.
    * <p>
    * The subset consists of the vertices from the vertex at {@code startIndexInclusive} to the
    * vertex {@code endIndexInclusive} while going from start to end in a clockwise order.
    * </p>
    * 
    * @param startIndexInclusive the index of the first vertex to add.
    * @param endIndexInclusive the index of the last vertex to add.
    * @param pointListToPack the list into which the vertices are to be added.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   protected void getPointsInClockwiseOrder(int startIndexInclusive, int endIndexInclusive, List<Point2DReadOnly> pointListToPack)
   {
      checkIfUpToDate();
      int index = startIndexInclusive;

      while (true)
      {
         pointListToPack.add(getVertex(index));

         if (index == endIndexInclusive)
            break;
         index = getNextVertexIndex(index);
      }
   }

   /**
    * Adds a subset of this polygon's vertices into the given polygon.
    * <p>
    * The subset consists of the vertices from the vertex at {@code startIndexInclusive} to the
    * vertex {@code endIndexInclusive} while going from start to end in a clockwise order.
    * </p>
    * 
    * @param startIndexInclusive the index of the first vertex to add.
    * @param endIndexInclusive the index of the last vertex to add.
    * @param polygonToPack the polygon into which the vertices are to be added.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   protected void getVerticesInClockwiseOrder(int startIndexInclusive, int endIndexInclusive, ConvexPolygon2d polygonToPack)
   {
      checkIfUpToDate();
      int index = startIndexInclusive;

      while (true)
      {
         polygonToPack.addVertex(getVertex(index));

         if (index == endIndexInclusive)
            break;
         index = getNextVertexIndex(index);
      }
   }

   /**
    * Tests on a per-component basis on every vertices if this convex polygon is equal to
    * {@code other} with the tolerance {@code epsilon}.
    * <p>
    * The method returns {@code false} if the two polygons have different size.
    * </p>
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal, {@code false} otherwise.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   @Override
   public boolean epsilonEquals(ConvexPolygon2d other, double epsilon)
   {
      checkIfUpToDate();

      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;

      for (int i = 0; i < other.getNumberOfVertices(); i++)
      {
         if (!other.getVertex(i).epsilonEquals(getVertex(i), epsilon))
            return false;
      }

      return true;
   }

   /**
    * Tests whether this polygon is empty, i.e. it has no vertices.
    * 
    * @return {@code true} if this polygon is empty, {@code false} otherwise.
    */
   public boolean isEmpty()
   {
      return numberOfVertices == 0;
   }

   /**
    * Tests whether this polygon has been updated via {@link #update()} since last time its vertices
    * have been modified.
    * 
    * @return {@code true} if this polygon is up-to-date and operations can be used, {@code false}
    *         otherwise.
    */
   public boolean isUpToDate()
   {
      return isUpToDate;
   }

   /**
    * Checks if this polygon has been updated via {@link #update()} since last time its vertices
    * have been modified.
    * 
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public void checkIfUpToDate()
   {
      if (!isUpToDate)
         throw new OutdatedPolygonException("Call the method ConvexPolygon2d.update() before doing any other calculation!");
   }

   /**
    * Checks if this polygon is empty, i.e. it has no vertices.
    * 
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public void checkNonEmpty()
   {
      if (isEmpty())
         throw new EmptyPolygonException("This polygon has no vertex. Add vertices with addVertex() or setAndUpdate() methods.");
   }

   /**
    * Checks if the given index is contained in the range [0, {@link #numberOfVertices}[.
    * 
    * @param index the index to check.
    * @throws IndexOutOfBoundsException if the given index is either negative or greater or equal
    *            than the polygon's number of vertices.
    */
   public void checkIndexInBoundaries(int index)
   {
      if (index < 0)
         throw new IndexOutOfBoundsException("vertexIndex < 0");
      if (index >= numberOfVertices)
         throw new IndexOutOfBoundsException("vertexIndex >= numberOfVertices. numberOfVertices = " + numberOfVertices);
   }

   /**
    * Tests if the given point is inside this polygon or exactly on and edge/vertex of this polygon.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code numberOfVertices == 0}, this method returns {@code false}.
    * <li>if {@code numberOfVertices == 1}, this method returns whether the query and the single
    * vertex are exactly equal.
    * <li>if {@code numberOfVertices == 2}, this method returns whether the query is exactly on the
    * polygons single edge.
    * </ul>
    * 
    * @param x the x-coordinate of the query.
    * @param y the y-coordinate of the query.
    * @return {@code true} if the query is inside this polygon, {@code false} otherwise.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public boolean isPointInside(double x, double y)
   {
      return isPointInside(x, y, 0.0);
   }

   /**
    * Determines if the point is inside this convex polygon given the tolerance {@code epsilon}.
    * <p>
    * The sign of {@code epsilon} is equivalent to performing the test against the polygon shrunk by
    * {@code Math.abs(epsilon)} if {@code epsilon < 0.0}, or against the polygon enlarged by
    * {@code epsilon} if {@code epsilon > 0.0}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code numberOfVertices == 0}, this method returns {@code false}.
    * <li>if {@code numberOfVertices == 1}, this method returns {@code false} if {@code epsilon < 0}
    * or if the query is at a distance from the polygon's only vertex that is greater than
    * {@code epsilon}, returns {@code true} otherwise.
    * <li>if {@code numberOfVertices == 2}, this method returns {@code false} if {@code epsilon < 0}
    * or if the query is at a distance from the polygon's only edge that is greater than
    * {@code epsilon}, returns {@code true} otherwise.
    * </ul>
    * 
    * 
    * @param x the x-coordinate of the query.
    * @param y the y-coordinate of the query.
    * @param epsilon the tolerance to use during the test.
    * @return {@code true} if the query is considered to be inside the polygon, {@code false}
    *         otherwise.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public boolean isPointInside(double x, double y, double epsilon)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(x, y, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered, epsilon);
   }

   /**
    * Tests if the given point is inside this polygon or exactly on and edge/vertex of this polygon.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code numberOfVertices == 0}, this method returns {@code false}.
    * <li>if {@code numberOfVertices == 1}, this method returns whether the query and the single
    * vertex are exactly equal.
    * <li>if {@code numberOfVertices == 2}, this method returns whether the query is exactly on the
    * polygons single edge.
    * </ul>
    * 
    * @param point the query. Not modified.
    * @return {@code true} if the query is inside this polygon, {@code false} otherwise.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public boolean isPointInside(Point2DReadOnly point)
   {
      return isPointInside(point, 0.0);
   }

   /**
    * Determines if the point is inside this convex polygon given the tolerance {@code epsilon}.
    * <p>
    * The sign of {@code epsilon} is equivalent to performing the test against the polygon shrunk by
    * {@code Math.abs(epsilon)} if {@code epsilon < 0.0}, or against the polygon enlarged by
    * {@code epsilon} if {@code epsilon > 0.0}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code numberOfVertices == 0}, this method returns {@code false}.
    * <li>if {@code numberOfVertices == 1}, this method returns {@code false} if {@code epsilon < 0}
    * or if the query is at a distance from the polygon's only vertex that is greater than
    * {@code epsilon}, returns {@code true} otherwise.
    * <li>if {@code numberOfVertices == 2}, this method returns {@code false} if {@code epsilon < 0}
    * or if the query is at a distance from the polygon's only edge that is greater than
    * {@code epsilon}, returns {@code true} otherwise.
    * </ul>
    * 
    * 
    * @param point the query. Not modified.
    * @param epsilon the tolerance to use during the test.
    * @return {@code true} if the query is considered to be inside the polygon, {@code false}
    *         otherwise.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public boolean isPointInside(Point2DReadOnly point, double epsilon)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(point, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered, epsilon);
   }

   /**
    * Computes the coordinates of the closest point to the ray that belongs to this convex polygon.
    * <p>
    * WARNING: This methods assumes that the ray does not intersect with the polygon. Such scenario
    * should be handled with {@link #intersectionWithRay(Line2D, Point2DBasics, Point2DBasics)}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code false}.
    * <li>If the ray is parallel to the closest edge, the closest point to the ray origin is chosen.
    * </ul>
    * </p>
    * 
    * @param ray the ray to find the closest point to. Not modified.
    * @param closestPointToPack the point in which the coordinates of the closest point are stored.
    *           Modified.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public boolean getClosestPointWithRay(Line2D ray, Point2DBasics closestPointToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestPointToNonInterectingRay2D(ray.getPoint(), ray.getDirection(), clockwiseOrderedVertices, numberOfVertices,
                                                                          clockwiseOrdered, closestPointToPack);
   }

   /**
    * Computes the coordinates of the closest point to the ray that belongs to this convex polygon.
    * <p>
    * WARNING: This methods assumes that the ray does not intersect with the polygon. Such scenario
    * should be handled with {@link #intersectionWithRay(Line2D, Point2DBasics, Point2DBasics)}.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code null}.
    * <li>If the ray is parallel to the closest edge, the closest point to the ray origin is chosen.
    * </ul>
    * </p>
    * 
    * @param ray the ray to find the closest point to. Not modified.
    * @return the coordinates of the closest point if the method succeeds, {@code null} otherwise.
    * @throws IllegalArgumentException if {@code numberOfVertices} is negative or greater than the
    *            size of the given list of vertices.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public Point2D getClosestPointWithRay(Line2D ray)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestPointToNonInterectingRay2D(ray.getPoint(), ray.getDirection(), clockwiseOrderedVertices, numberOfVertices,
                                                                          clockwiseOrdered);
   }

   /**
    * Calculates the minimum distance between the point and this polygon.
    * <p>
    * Note that if the point is inside this polygon, this method returns 0.0.
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @return the value of the distance between the point and this polygon.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public double distance(Point2DReadOnly point)
   {
      return Math.max(0.0, signedDistance(point));
   }

   /**
    * Returns minimum distance between the point and this polygon.
    * <p>
    * The return value is negative if the point is inside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@link Double#NaN}.
    * <li>If the polygon has exactly one vertex, the returned value is positive and is equal to the
    * distance between the query and the polygon's vertex.
    * <li>If the polygon has exactly two vertices, the returned value is positive and is equal to
    * the distance and the line segment defined by the polygon's two vertices.
    * </ul>
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @return the distance between the query and the polygon, it is negative if the point is inside
    *         the polygon.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public double signedDistance(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.signedDistanceFromPoint2DToConvexPolygon2D(point, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D convex polygon.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code false}.
    * <li>If the polygon has exactly one vertex, the result is the polygon only vertex, this method
    * returns {@code true}.
    * <li>If the query is inside the polygon, the method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to project on this polygon. Modified.
    * @return whether the method succeeded or not.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public boolean orthogonalProjection(Point2DBasics pointToProject)
   {
      return orthogonalProjection(pointToProject, pointToProject);
   }

   /**
    * Computes the orthogonal projection of a 2D point this 2D convex polygon.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code false}.
    * <li>If the polygon has exactly one vertex, the result is the polygon only vertex, this method
    * returns {@code true}.
    * <li>If the query is inside the polygon, the method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the coordinate of the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the convex polygon is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public boolean orthogonalProjection(Point2DReadOnly pointToProject, Point2DBasics projectionToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.orthogonalProjectionOnConvexPolygon2D(pointToProject, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered,
                                                                              projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D convex polygon.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code null}.
    * <li>If the polygon has exactly one vertex, the result is the polygon only vertex.
    * <li>If the query is inside the polygon, the method fails and returns {@code null}.
    * </ul>
    * </p>
    *
    * @param pointToProject the coordinate of the point to compute the projection of. Not modified.
    * @return the coordinates of the projection, or {@code null} if the method failed.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public Point2D orthogonalProjectionCopy(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.orthogonalProjectionOnConvexPolygon2D(point, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the index of the
    * first vertex that is in the line-of-sight.
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code -1}.
    * <li>The observer is inside the polygon, this method fails and returns {@code -1}.
    * <li>The polygon has exactly one vertex, this method returns {@code 0} if the observer is
    * different from the polygon's vertex, or returns {@code -1} if the observer is equal to the
    * polygon's vertex.
    * </ul>
    * </p>
    * 
    * @param observer the coordinates of the observer. Not modified.
    * @return the index of the first vertex that is in the line-of-sight, {@code -1} if this method
    *         fails.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public int lineOfSightStartIndex(Point2DReadOnly observer)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.lineOfSightStartIndex(observer, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the index of the
    * last vertex that is in the line-of-sight.
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code -1}.
    * <li>The observer is inside the polygon, this method fails and returns {@code -1}.
    * <li>The polygon has exactly one vertex, this method returns {@code 0} if the observer is
    * different from the polygon's vertex, or returns {@code -1} if the observer is equal to the
    * polygon's vertex.
    * </ul>
    * </p>
    * 
    * @param observer the coordinates of the observer. Not modified.
    * @return the index of the last vertex that is in the line-of-sight, {@code -1} if this method
    *         fails.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public int lineOfSightEndIndex(Point2DReadOnly observer)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.lineOfSightEndIndex(observer, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the indices of the
    * first and last vertices that are in the line-of-sight.
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code -1}.
    * <li>The observer is inside the polygon, this method fails and returns {@code -1}.
    * <li>The polygon has exactly one vertex, this method returns {@code 0} if the observer is
    * different from the polygon's vertex, or returns {@code -1} if the observer is equal to the
    * polygon's vertex.
    * </ul>
    * </p>
    * 
    * @param observer the coordinates of the observer. Not modified.
    * @return the indices in order of the first and last vertices that are in the line-of-sight,
    *         {@code null} if this method fails.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public int[] lineOfSightIndices(Point2DReadOnly observer)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.lineOfSightIndices(observer, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the first vertex
    * that is in the line-of-sight.
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code false}.
    * <li>The observer is inside the polygon, this method fails and returns {@code false}.
    * <li>The polygon has exactly one vertex, this method succeeds and packs the vertex coordinates
    * if the observer is different from the polygon's vertex, or returns {@code false} if the
    * observer is equal to the polygon's vertex.
    * </ul>
    * </p>
    * 
    * @param observer the coordinates of the observer. Not modified.
    * @param startVertexToPack point in which the coordinates of the first vertex in the
    *           line-of-sight are stored. Modified.
    * @return whether the method succeeded or not.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public boolean lineOfSightStartVertex(Point2DReadOnly observer, Point2DBasics startVertexToPack)
   {
      int lineOfSightStartIndex = lineOfSightStartIndex(observer);

      if (lineOfSightStartIndex == -1)
         return false;

      startVertexToPack.set(getVertexUnsafe(lineOfSightStartIndex));

      return true;
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the last vertex
    * that is in the line-of-sight.
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code false}.
    * <li>The observer is inside the polygon, this method fails and returns {@code false}.
    * <li>The polygon has exactly one vertex, this method succeeds and packs the vertex coordinates
    * if the observer is different from the polygon's vertex, or returns {@code false} if the
    * observer is equal to the polygon's vertex.
    * </ul>
    * </p>
    * 
    * @param observer the coordinates of the observer. Not modified.
    * @param endVertexToPack point in which the coordinates of the last vertex in the line-of-sight
    *           are stored. Modified.
    * @return whether the method succeeded or not.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public boolean lineOfSightEndVertex(Point2DReadOnly observer, Point2DBasics endVertexToPack)
   {
      checkIfUpToDate();
      int lineOfSightEndIndex = lineOfSightEndIndex(observer);

      if (lineOfSightEndIndex == -1)
         return false;

      endVertexToPack.set(getVertexUnsafe(lineOfSightEndIndex));

      return true;
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the first vertex
    * that is in the line-of-sight.
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code null}.
    * <li>The observer is inside the polygon, this method fails and returns {@code null}.
    * <li>The polygon has exactly one vertex, this method succeeds and returns the vertex
    * coordinates if the observer is different from the polygon's vertex, or returns {@code null} if
    * the observer is equal to the polygon's vertex.
    * </ul>
    * </p>
    * 
    * @param observer the coordinates of the observer. Not modified.
    * @return the coordinates of the first vertex in the line-of-sight or {@code null} if this
    *         method failed. Modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public Point2D lineOfSightStartVertexCopy(Point2DReadOnly observer)
   {
      Point2D startVertex = new Point2D();
      boolean success = lineOfSightStartVertex(observer, startVertex);
      return success ? startVertex : null;
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the last vertex
    * that is in the line-of-sight.
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code null}.
    * <li>The observer is inside the polygon, this method fails and returns {@code null}.
    * <li>The polygon has exactly one vertex, this method succeeds and returns the vertex
    * coordinates if the observer is different from the polygon's vertex, or returns {@code null} if
    * the observer is equal to the polygon's vertex.
    * </ul>
    * </p>
    * 
    * @param observer the coordinates of the observer. Not modified.
    * @return the coordinates of the last vertex in the line-of-sight or {@code null} if this method
    *         failed. Modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public Point2D lineOfSightEndVertexCopy(Point2DReadOnly observer)
   {
      Point2D endVertex = new Point2D();
      boolean success = lineOfSightEndVertex(observer, endVertex);
      return success ? endVertex : null;
   }

   /**
    * From the point of view of an observer located outside the polygon, only a continuous subset of
    * the polygon's edges can be seen defining a line-of-sight. This method finds the first and last
    * vertices that is in the line-of-sight.
    * <p>
    * WARNING: This method assumes that the given observer is located outside the polygon.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>The polygon has no vertices, this method fails and returns {@code null}.
    * <li>The observer is inside the polygon, this method fails and returns {@code null}.
    * <li>The polygon has exactly one vertex, this method succeeds and returns the vertex
    * coordinates if the observer is different from the polygon's vertex, or returns {@code null} if
    * the observer is equal to the polygon's vertex.
    * </ul>
    * </p>
    * 
    * @param observer the coordinates of the observer. Not modified.
    * @return the coordinates in order of the first and last vertices in the line-of-sight or
    *         {@code null} if this method failed. Modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public Point2D[] lineOfSightVertices(Point2DReadOnly observer)
   {
      Point2D startVertex = lineOfSightStartVertexCopy(observer);
      Point2D endVertex = lineOfSightEndVertexCopy(observer);
      if (startVertex == null || endVertex == null)
         return null;
      else
         return new Point2D[] {startVertex, endVertex};
   }

   /**
    * Determines whether an observer can see the outside of the given edge of this convex polygon.
    * <p>
    * The edge is defined by its start {@code this.getVertex(edgeIndex)} and its end
    * {@code this.getNextVertex(edgeIndex)}.
    * </p>
    * 
    * 
    * @param edgeIndex the vertex index of the start of the edge.
    * @param observer the coordinates of the observer. Not modified.
    * @return {@code true} if the observer can see the outside of the edge, {@code false} if the
    *         observer cannot see the outside or is lying on the edge.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public boolean canObserverSeeEdge(int edgeIndex, Point2DReadOnly observer)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.canObserverSeeEdge(edgeIndex, observer, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   /**
    * Tests if the given point lies on an edge of this convex polygon.
    * 
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is considered to be on an edge of this polygon,
    *         {@code false} otherwise.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public boolean pointIsOnPerimeter(Point2DReadOnly point)
   {
      return Math.abs(signedDistance(point)) < 1.0E-10;
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and this
    * convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments remain unmodified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of
    * the only intersection are stored in {@code firstIntersectionToPack}.
    * {@code secondIntersectionToPack} remains unmodified.
    * </ul>
    * </p>
    * 
    * @param line the line that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public int intersectionWith(Line2D line, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLine2DAndConvexPolygon2D(line.getPoint(), line.getDirection(), clockwiseOrderedVertices,
                                                                                    numberOfVertices, clockwiseOrdered, firstIntersectionToPack,
                                                                                    secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line 2D and this
    * convex polygon 2D.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections and
    * returns {@code null}.
    * <li>If no intersections exist, this method returns {@code null}.
    * </ul>
    * </p>
    * 
    * @param line the line that may intersect this polygon. Not modified.
    * @return the coordinates of the intersections between the line and the polygon, or {@code null}
    *         if they do not intersect.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public Point2D[] intersectionWith(Line2D line)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLine2DAndConvexPolygon2D(line.getPoint(), line.getDirection(), clockwiseOrderedVertices,
                                                                                    numberOfVertices, clockwiseOrdered);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and this
    * convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments might be modified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of
    * the only intersection are stored in {@code firstIntersectionToPack}.
    * {@code secondIntersectionToPack} might be modified.
    * </ul>
    * </p>
    * 
    * @param ray the ray that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the ray and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the ray and the polygon.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public int intersectionWithRay(Line2D ray, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenRay2DAndConvexPolygon2D(ray.getPoint(), ray.getDirection(), clockwiseOrderedVertices,
                                                                                   numberOfVertices, clockwiseOrdered, firstIntersectionToPack,
                                                                                   secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given ray 2D and this
    * convex polygon 2D.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections and
    * returns {@code null}.
    * <li>If no intersections exist, this method returns {@code null}.
    * </ul>
    * </p>
    * 
    * @param ray the ray that may intersect this polygon. Not modified.
    * @return the intersections between the ray and the polygon.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public Point2D[] intersectionWithRay(Line2D ray)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenRay2DAndConvexPolygon2D(ray.getPoint(), ray.getDirection(), clockwiseOrderedVertices,
                                                                                   numberOfVertices, clockwiseOrdered);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments remain unmodified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of
    * the only intersection are stored in {@code firstIntersectionToPack}.
    * {@code secondIntersectionToPack} remains unmodified.
    * <li>If the line segment is collinear to an edge:
    * <ul>
    * <li>The edge entirely contains the line segment: this method finds two intersections which are
    * the endpoints of the line segment.
    * <li>The line segment entirely contains the edge: this method finds two intersections which are
    * the vertices of the edge.
    * <li>The edge and the line segment partially overlap: this method finds two intersections which
    * the polygon's vertex that on the line segment and the line segment's endpoint that is on the
    * polygon's edge.
    * </ul>
    * </ul>
    * </p>
    * 
    * @param lineSegment the line segment that may intersect this polygon. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection
    *           between the line and the convex polygon. Can be {@code null}. Modified.
    * @return the number of intersections between the line and the polygon.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public int intersectionWith(LineSegment2D lineSegment2d, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLineSegment2DAndConvexPolygon2D(lineSegment2d.getFirstEndpoint(), lineSegment2d.getSecondEndpoint(),
                                                                                           clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered,
                                                                                           firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections and
    * this method returns {@code null}.
    * <li>If no intersections exist, this method returns {@code null}.
    * <li>If the line segment is collinear to an edge:
    * <ul>
    * <li>The edge entirely contains the line segment: this method finds two intersections which are
    * the endpoints of the line segment.
    * <li>The line segment entirely contains the edge: this method finds two intersections which are
    * the vertices of the edge.
    * <li>The edge and the line segment partially overlap: this method finds two intersections which
    * the polygon's vertex that on the line segment and the line segment's endpoint that is on the
    * polygon's edge.
    * </ul>
    * </ul>
    * </p>
    * 
    * @param lineSegment the line segment that may intersect this polygon. Not modified.
    * @return the intersections between the line segment and the polygon.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public Point2D[] intersectionWith(LineSegment2D lineSegment2d)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLineSegment2DAndConvexPolygon2D(lineSegment2d.getFirstEndpoint(), lineSegment2d.getSecondEndpoint(),
                                                                                           clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   /**
    * Finds the index of the closest edge to the query.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has one or no vertices, this method fails and returns {@code -1}.
    * </ul>
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @return the index of the closest edge to the query.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public int getClosestEdgeIndex(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestEdgeIndexToPoint2D(point, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   /**
    * Finds the index of the closest edge to the query.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has one or no vertices, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @param closestEdgeToPack the line segment used to store the result. Not modified.
    * @return whether this method succeeded or not.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public boolean getClosestEdge(Point2DReadOnly point, LineSegment2D closestEdgeToPack)
   {
      int edgeIndex = getClosestEdgeIndex(point);
      if (edgeIndex == -1)
         return false;
      getEdge(edgeIndex, closestEdgeToPack);
      return true;
   }

   /**
    * Finds the index of the closest edge to the query.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has one or no vertices, this method fails and returns {@code null}.
    * </ul>
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @return the line segment representing the closest edge or {@code null} if this method failed.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public LineSegment2D getClosestEdgeCopy(Point2DReadOnly point)
   {
      LineSegment2D closestEdge = new LineSegment2D();
      if (getClosestEdge(point, closestEdge))
         return closestEdge;
      else
         return null;
   }

   /**
    * Finds the index of the closest vertex to the query.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code -1}.
    * </ul>
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @return the index of the closest vertex to the query.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public int getClosestVertexIndex(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestVertexIndexToPoint2D(point, clockwiseOrderedVertices, numberOfVertices);
   }

   /**
    * Finds the index of the closest vertex to the query.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @param vertexToPack point used to store the result. Modified.
    * @return whether this method succeeded or not.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public boolean getClosestVertex(Point2DReadOnly point, Point2DBasics vertexToPack)
   {
      int vertexIndex = getClosestVertexIndex(point);
      if (vertexIndex == -1)
         return false;
      vertexToPack.set(getVertex(vertexIndex));
      return true;
   }

   /**
    * Finds the index of the closest vertex to the query.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code null}.
    * </ul>
    * </p>
    * 
    * @param point the coordinates of the query. Not modified.
    * @return the coordinates of the closest vertex, or {@code null} if this method failed.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public Point2D getClosestVertexCopy(Point2DReadOnly point)
   {
      int vertexIndex = getClosestVertexIndex(point);
      if (vertexIndex == -1)
         return null;
      return new Point2D(getVertex(vertexIndex));
   }

   /**
    * Finds the index of the closest vertex to the given line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code -1}.
    * </ul>
    * </p>
    * 
    * @param line the query. Not modified.
    * @return the index of the closest vertex to the query.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public int getClosestVertexIndex(Line2D line)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestVertexIndexToLine2D(line.getPoint(), line.getDirection(), clockwiseOrderedVertices, numberOfVertices);
   }

   /**
    * Finds the index of the closest vertex to the given line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param line the query. Not modified.
    * @param vertexToPack point used to store the result. Modified.
    * @return whether this method succeeded or not.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public boolean getClosestVertex(Line2D line, Point2DBasics vertexToPack)
   {
      int vertexIndex = getClosestVertexIndex(line);
      if (vertexIndex == -1)
         return false;
      vertexToPack.set(getVertex(vertexIndex));
      return true;
   }

   /**
    * Finds the index of the closest vertex to the given line.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method fails and returns {@code false}.
    * </ul>
    * </p>
    * 
    * @param line the query. Not modified.
    * @return the coordinates of the closest vertex or {@code null} if this method failed.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   public Point2D getClosestVertexCopy(Line2D line)
   {
      int vertexIndex = getClosestVertexIndex(line);
      if (vertexIndex == -1)
         return null;
      return new Point2D(getVertex(vertexIndex));
   }

   /**
    * Packs the endpoints of an edge of this polygon into {@code edgeToPack}.
    * 
    * @param edgeIndex index of the vertex that starts the edge.
    * @param edgeToPack line segment used to store the edge endpoints. Modified.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's vertices were edited.
    * @throws IndexOutOfBoundsException if the given {@code index} is negative or greater or equal
    *            than this polygon's number of vertices.
    * @throws EmptyPolygonException if this polygon is empty when calling this method.
    */
   public void getEdge(int edgeIndex, LineSegment2D edgeToPack)
   {
      edgeToPack.set(getVertex(edgeIndex), getNextVertex(edgeIndex));
   }
}
