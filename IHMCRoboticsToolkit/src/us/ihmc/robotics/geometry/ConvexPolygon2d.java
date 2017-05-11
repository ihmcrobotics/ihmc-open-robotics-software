package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

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
      isUpToDate = false;
      setOrCreate(vertex, numberOfVertices);
      numberOfVertices++;
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

      isUpToDate = false;
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

      isUpToDate = false;
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

      isUpToDate = false;
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
    */
   public void addVertices(ConvexPolygon2d otherPolygon)
   {
      isUpToDate = false;
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
    * @param point the coordinates to copy. Not modified.
    * @param i the position in the list {@link #clockwiseOrderedVertices} to copy the given point.
    */
   private void setOrCreate(Point2DReadOnly point, int i)
   {
      setOrCreate(point.getX(), point.getY(), i);
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

      if (hasAtLeastOneVertex())
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
      area = 0.0;
      centroid.set(0.0, 0.0);

      if (hasAtLeastThreeVertices())
      {
         double Cx = 0.0;
         double Cy = 0.0;

         // Order counterclockwise to make the area positive
         for (int i = numberOfVertices - 1; i >= 0; i--)
         {
            Point2D ci = getVertexUnsafe(i);
            Point2D ciMinus1 = getPreviousVertexUnsafe(i);

            double weight = (ci.getX() * ciMinus1.getY() - ciMinus1.getX() * ci.getY());

            Cx += (ci.getX() + ciMinus1.getX()) * weight;
            Cy += (ci.getY() + ciMinus1.getY()) * weight;

            area += weight;
         }

         area *= 0.5;

         if (area < 1.0e-5)
         {
            centroid.set(getVertex(0));
         }
         else
         {
            Cx *= 1.0 / (6.0 * area);
            Cy *= 1.0 / (6.0 * area);

            centroid.set(Cx, Cy);
         }
      }
      else if (hasAtLeastOneVertex())
      {
         for (int i = 0; i < numberOfVertices; i++)
         {
            centroid.add(getVertex(i));
         }

         centroid.scale(1.0 / numberOfVertices);
      }
      else
      {
         area = Double.NaN;
         centroid.set(Double.NaN, Double.NaN);
      }
   }

   /**
    * Gets the value of this polygon area.
    * 
    * @return the are of this polygon.
    * @throws OutdatedPolygonException if {@link #update()} has not been called since last time this
    *            polygon's were edited.
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
    *            polygon's were edited.
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
    *            polygon's were edited.
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
    *            polygon's were edited.
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
    *            polygon's were edited.
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
    *            polygon's were edited.
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
    *            polygon's were edited.
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
    *            polygon's were edited.
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
    *            polygon's were edited.
    */
   public Point2DReadOnly getVertex(int index)
   {
      checkIfUpToDate();
      return getVertexUnsafe(index);
   }

   /**
    * Same as getVertex(vertexIndex) but without checking if the polygon has been updated. Be
    * careful when using it!
    */
   protected Point2D getVertexUnsafe(int vertexIndex)
   {
      checkNonEmpty();
      checkIndexInBoundaries(vertexIndex);
      return clockwiseOrderedVertices.get(vertexIndex);
   }

   /** Return the next vertex from a clockwise ordered list */
   public Point2DReadOnly getNextVertex(int index)
   {
      return getVertex(getNextVertexIndex(index));
   }

   protected Point2D getNextVertexUnsafe(int index)
   {
      return getVertexUnsafe(getNextVertexIndexUnsafe(index));
   }

   /** Return the previous vertex from a clockwise ordered list */
   public Point2DReadOnly getPreviousVertex(int index)
   {
      return getVertex(getPreviousVertexIndex(index));
   }

   protected Point2D getPreviousVertexUnsafe(int index)
   {
      return getVertexUnsafe(getPreviousVertexIndexUnsafe(index));
   }

   /** Return the vertex from a counter clockwise ordered list */
   public Point2DReadOnly getVertexCCW(int vertexIndex)
   {
      checkIfUpToDate();
      checkNonEmpty();
      checkIndexInBoundaries(vertexIndex);
      return clockwiseOrderedVertices.get(numberOfVertices - 1 - vertexIndex);
   }

   /** Return the next vertex from a counter clockwise ordered list */
   public Point2DReadOnly getNextVertexCCW(int index)
   {
      return getVertexCCW(getNextVertexIndex(index));
   }

   /** Return the previous vertex from a counter clockwise ordered list */
   public Point2DReadOnly getPreviousVertexCCW(int index)
   {
      return getVertexCCW(getPreviousVertexIndex(index));
   }

   public int getNextVertexIndex(int currentVertexIndex)
   {
      checkIfUpToDate();
      checkIndexInBoundaries(currentVertexIndex);
      return getNextVertexIndexUnsafe(currentVertexIndex);
   }

   protected int getNextVertexIndexUnsafe(int currentVertexIndex)
   {
      checkNonEmpty();

      if (currentVertexIndex < numberOfVertices - 1)
         return currentVertexIndex + 1;
      else
         return 0;
   }

   public int getPreviousVertexIndex(int currentVertexIndex)
   {
      checkIfUpToDate();
      checkIndexInBoundaries(currentVertexIndex);
      return getPreviousVertexIndexUnsafe(currentVertexIndex);
   }

   protected int getPreviousVertexIndexUnsafe(int currentVertexIndex)
   {
      checkNonEmpty();

      if (currentVertexIndex < 1)
         return numberOfVertices - 1;
      else
         return currentVertexIndex - 1;
   }

   public int getNumberOfVertices()
   {
      return numberOfVertices;
   }

   /**
    * Scale this convex polygon about its centroid, i.e. once scaled the polygon centroid remains
    * unchanged.
    * 
    * @param scaleFactor
    */
   public void scale(double scaleFactor)
   {
      scale(centroid, scaleFactor);
   }

   /**
    * Scale this convex polygon about pointToScaleAbout.
    * 
    * @param scaleFactor
    */
   public void scale(Point2DReadOnly pointToScaleAbout, double scaleFactor)
   {
      checkIfUpToDate();
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
    */
   public void translate(Tuple2DReadOnly translation)
   {
      translate(translation.getX(), translation.getY());
   }

   public void translate(double x, double y)
   {
      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = getVertexUnsafe(i);
         vertex.add(x, y);
      }

      updateBoundingBox();
      updateCentroidAndArea();
   }

   @Override
   public String toString()
   {
      String ret = "";
      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = clockwiseOrderedVertices.get(i);
         ret = ret + "{" + vertex.getX() + ", " + vertex.getY() + "}," + "\n";
      }

      return ret;
   }

   @Override
   public void applyTransform(Transform transform)
   {
      isUpToDate = false;

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = getVertexUnsafe(i);
         vertex.applyTransform(transform);
      }
      update();
   }

   public void applyTransformAndProjectToXYPlane(Transform transform)
   {
      isUpToDate = false;

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = getVertexUnsafe(i);
         vertex.applyTransform(transform, false);
      }
      update();
   }

   public ConvexPolygon2d applyTransformCopy(Transform transform)
   {
      ConvexPolygon2d copy = new ConvexPolygon2d(this);
      copy.applyTransform(transform);
      return copy;
   }

   public ConvexPolygon2d applyTransformAndProjectToXYPlaneCopy(Transform transform)
   {
      ConvexPolygon2d copy = new ConvexPolygon2d(this);
      copy.applyTransformAndProjectToXYPlane(transform);

      return copy;
   }

   public double getMaxX()
   {
      return getVertexUnsafe(maxX_index).getX();
   }

   public double getMinX()
   {
      return getVertexUnsafe(minX_index).getX();
   }

   public double getMaxY()
   {
      return getVertexUnsafe(maxY_index).getY();
   }

   public double getMinY()
   {
      return getVertexUnsafe(minY_index).getY();
   }

   public int getMinXIndex()
   {
      return minX_index;
   }

   public int getMaxXIndex()
   {
      return maxX_index;
   }

   public int getMinYIndex()
   {
      return minY_index;
   }

   public int getMaxYIndex()
   {
      return maxY_index;
   }

   public Point2D getMinXMaxYPointCopy()
   {
      checkIfUpToDate();
      return new Point2D(getVertex(minXmaxY_index));
   }

   public Point2D getMinXMinYPointCopy()
   {
      checkIfUpToDate();
      return new Point2D(getVertex(minXminY_index));
   }

   public Point2D getMaxXMaxYPointCopy()
   {
      checkIfUpToDate();
      return new Point2D(getVertex(maxXmaxY_index));
   }

   public Point2D getMaxXMinYPointCopy()
   {
      checkIfUpToDate();
      return new Point2D(getVertex(maxXminY_index));
   }

   protected void getPointsInClockwiseOrder(int startIndexInclusive, int endIndexInclusive, List<Point2DReadOnly> pointList)
   {
      checkIfUpToDate();
      int index = startIndexInclusive;

      while (true)
      {
         pointList.add(getVertex(index));

         if (index == endIndexInclusive)
            break;
         index = getNextVertexIndex(index);
      }
   }

   protected void addVerticesInClockwiseOrderInPolygon(int startIndexInclusive, int endIndexInclusive, ConvexPolygon2d polygonToPack)
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

   @Override
   public boolean epsilonEquals(ConvexPolygon2d convexPolygon, double threshold)
   {
      checkIfUpToDate();
      if (getNumberOfVertices() != convexPolygon.getNumberOfVertices())
         return false;

      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         if (!convexPolygon.getVertex(i).epsilonEquals(getVertex(i), threshold))
            return false;
      }

      return true;
   }

   public boolean isEmpty()
   {
      return numberOfVertices == 0;
   }

   protected boolean hasAtLeastThreeVertices()
   {
      checkIfUpToDate();
      return numberOfVertices >= 3;
   }

   protected boolean hasAtLeastTwoVertices()
   {
      checkIfUpToDate();
      return numberOfVertices >= 2;
   }

   protected boolean hasAtLeastOneVertex()
   {
      checkIfUpToDate();
      return numberOfVertices >= 1;
   }

   protected boolean hasExactlyOneVertex()
   {
      checkIfUpToDate();
      return numberOfVertices == 1;
   }

   protected boolean hasExactlyTwoVertices()
   {
      checkIfUpToDate();
      return numberOfVertices == 2;
   }

   public boolean isUpToDate()
   {
      return isUpToDate;
   }

   public void checkIfUpToDate()
   {
      if (!isUpToDate)
         throw new OutdatedPolygonException("Call the method ConvexPolygon2d.update() before doing any other calculation!");
   }

   public void checkNonEmpty()
   {
      if (isEmpty())
         throw new EmptyPolygonException("This polygon has no vertex. Add vertices with addVertex() or setAndUpdate() methods.");
   }

   public void checkIndexInBoundaries(int vertexIndex)
   {
      if (vertexIndex < 0)
         throw new IndexOutOfBoundsException("vertexIndex < 0");
      if (vertexIndex >= numberOfVertices)
         throw new IndexOutOfBoundsException("vertexIndex >= numberOfVertices. numberOfVertices = " + numberOfVertices);
   }

   @Override
   public void set(ConvexPolygon2d other)
   {
      setAndUpdate(other);
   }

   @Override
   public void setToZero()
   {
      numberOfVertices = 1;
      if (clockwiseOrderedVertices.isEmpty())
         clockwiseOrderedVertices.add(new Point2D());

      Point2D point = clockwiseOrderedVertices.get(0);
      point.set(0.0, 0.0);

      isUpToDate = false;
      update();
   }

   @Override
   public void setToNaN()
   {
      numberOfVertices = 1;
      if (clockwiseOrderedVertices.isEmpty())
         clockwiseOrderedVertices.add(new Point2D());

      Point2D point = clockwiseOrderedVertices.get(0);
      point.set(Double.NaN, Double.NaN);

      isUpToDate = false;
      update();
   }

   @Override
   public boolean containsNaN()
   {
      update();

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D point = clockwiseOrderedVertices.get(i);

         if (Double.isNaN(point.getX()))
            return true;
         if (Double.isNaN(point.getY()))
            return true;
      }

      return false;
   }

   public boolean isPointInside(double x, double y)
   {
      return isPointInside(x, y, 0.0);
   }

   public boolean isPointInside(double x, double y, double epsilon)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(x, y, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered, epsilon);
   }

   public boolean isPointInside(Point2DReadOnly point)
   {
      return isPointInside(point, 0.0);
   }

   public boolean isPointInside(Point2DReadOnly point, double epsilon)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(point, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered, epsilon);
   }

   public ConvexPolygon2d translateCopy(Tuple2DReadOnly translation)
   {
      ConvexPolygon2d copy = new ConvexPolygon2d(this);
      copy.translate(translation);
      return copy;
   }

   public LineSegment2D[] getIntersectingEdgesCopy(Line2D line)
   {
      return ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line, this);
   }

   public boolean getClosestPointWithRay(Point2DBasics pointToPack, Line2D ray)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestPointToNonInterectingRay2D(ray.getPoint(), ray.getDirection(), clockwiseOrderedVertices, numberOfVertices,
                                                                          clockwiseOrdered, pointToPack);
   }

   public Point2D getClosestPointWithRay(Line2D ray)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestPointToNonInterectingRay2D(ray.getPoint(), ray.getDirection(), clockwiseOrderedVertices, numberOfVertices,
                                                                          clockwiseOrdered);
   }

   public double distance(Point2DReadOnly point)
   {
      return Math.max(0.0, signedDistance(point));
   }

   public double signedDistance(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.signedDistanceFromPoint2DToConvexPolygon2D(point, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   public void orthogonalProjection(Point2DBasics point2d)
   {
      orthogonalProjection(point2d, point2d);
   }

   public void orthogonalProjection(Point2DReadOnly point2d, Point2DBasics projectionToPack)
   {
      checkIfUpToDate();
      EuclidGeometryPolygonTools.orthogonalProjectionOnConvexPolygon2D(point2d, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered, projectionToPack);
   }

   public Point2D orthogonalProjectionCopy(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.orthogonalProjectionOnConvexPolygon2D(point, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   public int lineOfSightStartIndex(Point2DReadOnly observer)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.lineOfSightStartIndex(observer, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   public int lineOfSightEndIndex(Point2DReadOnly observer)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.lineOfSightEndIndex(observer, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   public int[] lineOfSightIndices(Point2DReadOnly observer)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.lineOfSightIndices(observer, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   public boolean lineOfSightStartVertex(Point2DReadOnly observer, Point2DBasics startVertexToPack)
   {
      checkIfUpToDate();
      int lineOfSightStartIndex = lineOfSightStartIndex(observer);

      if (lineOfSightStartIndex == -1)
         return false;

      startVertexToPack.set(getVertexUnsafe(lineOfSightStartIndex));

      return true;
   }

   public boolean lineOfSightEndVertex(Point2DReadOnly observer, Point2DBasics endVertexToPack)
   {
      checkIfUpToDate();
      int lineOfSightEndIndex = lineOfSightEndIndex(observer);

      if (lineOfSightEndIndex == -1)
         return false;

      endVertexToPack.set(getVertexUnsafe(lineOfSightEndIndex));

      return true;
   }

   public Point2D lineOfSightStartVertex(Point2DReadOnly observer)
   {
      Point2D startVertex = new Point2D();
      boolean success = lineOfSightStartVertex(observer, startVertex);
      return success ? startVertex : null;
   }

   public Point2D lineOfSightEndVertex(Point2DReadOnly observer)
   {
      Point2D endVertex = new Point2D();
      boolean success = lineOfSightEndVertex(observer, endVertex);
      return success ? endVertex : null;
   }

   public Point2D[] lineOfSightVertices(Point2DReadOnly observer)
   {
      Point2D startVertex = lineOfSightStartVertex(observer);
      Point2D endVertex = lineOfSightEndVertex(observer);
      if (startVertex == null || endVertex == null)
         return null;
      else
         return new Point2D[] {startVertex, endVertex};
   }

   public boolean canObserverSeeEdge(int edgeIndex, Point2DReadOnly observer)
   {
      return EuclidGeometryPolygonTools.canObserverSeeEdge(edgeIndex, observer, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   public boolean pointIsOnPerimeter(Point2DReadOnly point)
   {
      return distance(point) < 1.0E-10;
   }

   public int intersectionWith(Line2D line, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLine2DAndConvexPolygon2D(line.getPoint(), line.getDirection(), clockwiseOrderedVertices,
                                                                                    numberOfVertices, clockwiseOrdered, firstIntersectionToPack,
                                                                                    secondIntersectionToPack);
   }

   public Point2D[] intersectionWith(Line2D line)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLine2DAndConvexPolygon2D(line.getPoint(), line.getDirection(), clockwiseOrderedVertices,
                                                                                    numberOfVertices, clockwiseOrdered);
   }

   public int intersectionWithRay(Line2D ray, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenRay2DAndConvexPolygon2D(ray.getPoint(), ray.getDirection(), clockwiseOrderedVertices,
                                                                                   numberOfVertices, clockwiseOrdered, firstIntersectionToPack,
                                                                                   secondIntersectionToPack);
   }

   public Point2D[] intersectionWithRay(Line2D ray)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenRay2DAndConvexPolygon2D(ray.getPoint(), ray.getDirection(), clockwiseOrderedVertices,
                                                                                   numberOfVertices, clockwiseOrdered);
   }

   public int intersectionWith(LineSegment2D lineSegment2d, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLineSegment2DAndConvexPolygon2D(lineSegment2d.getFirstEndpoint(), lineSegment2d.getSecondEndpoint(),
                                                                                           clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered,
                                                                                           firstIntersectionToPack, secondIntersectionToPack);
   }

   public Point2D[] intersectionWith(LineSegment2D lineSegment2d)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLineSegment2DAndConvexPolygon2D(lineSegment2d.getFirstEndpoint(), lineSegment2d.getSecondEndpoint(),
                                                                                           clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   public int getClosestEdgeIndex(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestEdgeIndexToPoint2D(point, clockwiseOrderedVertices, numberOfVertices, clockwiseOrdered);
   }

   public boolean getClosestEdge(LineSegment2D closestEdgeToPack, Point2DReadOnly point)
   {
      int edgeIndex = getClosestEdgeIndex(point);
      if (edgeIndex == -1)
         return false;
      getEdge(edgeIndex, closestEdgeToPack);
      return true;
   }

   public LineSegment2D getClosestEdgeCopy(Point2DReadOnly point)
   {
      LineSegment2D closestEdge = new LineSegment2D();
      if (getClosestEdge(closestEdge, point))
         return closestEdge;
      else
         return null;
   }

   public int getClosestVertexIndex(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestVertexIndexToPoint2D(point, clockwiseOrderedVertices, numberOfVertices);
   }

   public boolean getClosestVertex(Point2DReadOnly point, Point2DBasics vertexToPack)
   {
      int vertexIndex = getClosestVertexIndex(point);
      if (vertexIndex == -1)
         return false;
      vertexToPack.set(getVertex(vertexIndex));
      return true;
   }

   public Point2D getClosestVertexCopy(Point2DReadOnly point)
   {
      int vertexIndex = getClosestVertexIndex(point);
      if (vertexIndex == -1)
         return null;
      return new Point2D(getVertex(vertexIndex));
   }

   public int getClosestVertexIndex(Line2D line)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestVertexIndexToLine2D(line.getPoint(), line.getDirection(), clockwiseOrderedVertices, numberOfVertices);
   }

   public boolean getClosestVertex(Line2D line, Point2DBasics vertexToPack)
   {
      int vertexIndex = getClosestVertexIndex(line);
      if (vertexIndex == -1)
         return false;
      vertexToPack.set(getVertex(vertexIndex));
      return true;
   }

   public Point2D getClosestVertexCopy(Line2D line)
   {
      int vertexIndex = getClosestVertexIndex(line);
      if (vertexIndex == -1)
         return null;
      return new Point2D(getVertex(vertexIndex));
   }

   public void getEdge(int edgeIndex, LineSegment2D edgeToPack)
   {
      edgeToPack.set(getVertex(edgeIndex), getNextVertex(edgeIndex));
   }

   // TODO: clean up garbage in / implement the following methods
   public ConvexPolygon2d intersectionWith(ConvexPolygon2d convexPolygon)
   {
      checkIfUpToDate();
      ConvexPolygon2d ret = new ConvexPolygon2d();
      boolean success = ConvexPolygonTools.computeIntersectionOfPolygons(this, convexPolygon, ret);
      if (!success)
         ret = null;
      return ret;
   }

   public boolean intersectionWith(ConvexPolygon2d convexPolygon, ConvexPolygon2d intersectionToPack)
   {
      checkIfUpToDate();
      return ConvexPolygonTools.computeIntersectionOfPolygons(this, convexPolygon, intersectionToPack);
   }

   public double distance(Line2D line)
   {
      checkIfUpToDate();
      throw new RuntimeException("Not yet implemented");
   }

   public double distance(LineSegment2D lineSegment)
   {
      checkIfUpToDate();
      throw new RuntimeException("Not yet implemented");
   }

   public double distance(ConvexPolygon2d convexPolygon)
   {
      checkIfUpToDate();
      throw new RuntimeException("Not yet implemented");
   }
}
