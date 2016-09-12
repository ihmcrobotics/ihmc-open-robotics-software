package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Tuple2d;
import javax.vecmath.Vector2d;

import us.ihmc.robotics.geometry.ConvexPolygonTools.EmptyPolygonException;
import us.ihmc.robotics.geometry.ConvexPolygonTools.OutdatedPolygonException;
import us.ihmc.robotics.math.exceptions.UndefinedOperationException;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * <p>Title: ConvexPolygon2d</p>
 *
 * <p>Description: Describes a planar convex polygon.
 * The vertices of this polygon are clockwise and are all different.
 * </p>
 *
 * <p>Copyright: Copyright (c) 2007</p>
 *
 * <p>Company: </p>
 *
 * @author IHMC-Yobotics Biped Team
 * @version 1.0
 */
public class ConvexPolygon2d implements Geometry2d<ConvexPolygon2d>
{
   private final ArrayList<Point2d> clockwiseOrderedListOfPoints = new ArrayList<Point2d>();
   private final BoundingBox2d boundingBox = new BoundingBox2d();
   private final Point2d centroid = new Point2d();
   private int numberOfVertices = 0;
   private boolean isUpToDate = false;
   private double area;

   private int minX_index = 0, maxX_index = 0, minY_index = 0, maxY_index = 0;
   private int minXminY_index = 0, maxXminY_index = 0, maxXmaxY_index = 0;
   private final int minXmaxY_index = 0;

   // some temporary objects to reduce garbage generation:
   private final Point2d tempPoint = new Point2d();
   private final Vector2d tempVector1 = new Vector2d();
   private final Vector2d tempVector2 = new Vector2d();
   private final int[] tempTwoIndices = new int[2];
   private final LineSegment2d tempSegment1 = new LineSegment2d();
   private final LineSegment2d tempSegment2 = new LineSegment2d();

   /**
    * Creates an empty convex polygon.
    */
   public ConvexPolygon2d()
   {
      numberOfVertices = 0;
      update();
   }

   /**
    * Creates an empty convex polygon, adds N new vertices using a list of {@code Point2d}, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param vertices {@code List<Point2d>} the list of points that is used to creates the vertices.
    * @param numberOfVertices int that is used to determine the number of vertices of the polygon.
    * Note the: {@code pointList.size()} can be greater or equal to numberOfVertices.
    */
   public ConvexPolygon2d(List<Point2d> vertices, int numberOfVertices)
   {
      setAndUpdate(vertices, numberOfVertices);
   }

   /**
    * Creates an empty convex polygon, adds N new vertices using a list of {@code Point2d}, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param vertices {@code List<Point2d>} the list of points that is used to creates the vertices.
    * The number of vertices of this polygon will be equal to the size of the point list.
    */
   public ConvexPolygon2d(List<Point2d> vertices)
   {
      this(vertices, vertices.size());
   }

   /**
    * Creates an empty convex polygon, adds N new vertices using an array of {@code Point2d}, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param vertices {@code double[>=numberOfVertices][>=2]} the array of points that is used to creates the vertices. The each row contains one point whereas the (at least) two columns contains the coordinates x and y.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the polygon.
    * Note the: {@code pointListArray.length} can be greater or equal to numberOfVertices.
    */
   public ConvexPolygon2d(double[][] vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * Creates an empty convex polygon, adds N new vertices using an array of {@code Point2d}, updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * @param vertices {@code double[N][>=2]} the array of points that is used to creates the vertices.
    * Each row contains one point whereas the (at least) two columns contains the coordinates x and y.
    * The number of vertices of this polygon will be equal to the length of the point array.
    */
   public ConvexPolygon2d(double[][] vertices)
   {
      this(vertices, vertices.length);
   }

   /**
    * Creates a polygon with the same properties as the other polygon.
    * @param otherPolygon {@code ConvexPolygon2d} the other convex polygon.
    */
   public ConvexPolygon2d(ConvexPolygon2d otherPolygon)
   {
      setAndUpdate(otherPolygon);
   }

   /**
    * Creates a new convex polygon by combining two other convex polygons. The result is the smallest convex hull that contains both polygons.
    * Then it updates the vertices so they are clockwise ordered, and initializes some essential numbers such as the centroid.
    * <p/> TODO: Make this more efficient by finding the rotating calipers, as in the intersection method.<p/>
    *
    * @param firstPolygon {@code ConvexPolygon2d}
    * @param secondPolygon {@code ConvexPolygon2d}
    */
   public ConvexPolygon2d(ConvexPolygon2d firstPolygon, ConvexPolygon2d secondPolygon)
   {
      setAndUpdate(firstPolygon, secondPolygon);
   }

   public static ConvexPolygon2d generateRandomConvexPolygon2d(Random random, double maxAbsoluteXY, int numberOfPossiblePoints)
   {
      ArrayList<Point2d> vertices = new ArrayList<Point2d>();

      for (int i = 0; i < numberOfPossiblePoints; i++)
      {
         vertices.add(RandomTools.generateRandomPoint2d(random, maxAbsoluteXY, maxAbsoluteXY));
      }

      ConvexPolygon2d polygonToReturn = new ConvexPolygon2d(vertices);

      return polygonToReturn;
   }

   /**
    * After calling this method, the polygon has no vertex, area, or centroid.
    * Note that calling that method doesn't generate garbage.
    */
   public void clear()
   {
      numberOfVertices = 0;
      area = Double.NaN;
      centroid.set(Double.NaN, Double.NaN);
      isUpToDate = false;
   }

   /**
    * After calling this method, the polygon has no vertex, area, or centroid and isUpToDate = true.
    * Use only when an empty is desired.
    * Note that calling that method doesn't generate garbage.
    */
   public void clearAndUpdate()
   {
      clear();
      isUpToDate = true;
   }

   /**
    * Add a vertex to this polygon.
    * Note that this method recycles memory.
    * @param vertex {@code Point2d} the new vertex.
    */
   public void addVertex(Point2d vertex)
   {
      isUpToDate = false;
      setOrCreate(vertex, numberOfVertices);
      numberOfVertices++;
   }

   /**
    * Add a vertex to this polygon.
    * Note that this method recycles memory.
    * @param x {@code double} first coordinate of the new vertex.
    * @param y {@code double} second coordinate of the new vertex.
    */
   public void addVertex(double x, double y)
   {
      isUpToDate = false;
      setOrCreate(x, y, numberOfVertices);
      numberOfVertices++;
   }

   /**
    * Adds N new vertices to this polygon using a list of {@code Point2d}.
    * Note that this method recycles memory.
    * @param vertices {@code List<Point2d>} the list of new vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices to add to this polygon. Note the: {@code vertices.size()} can be greater or equal to numberOfVertices.
    */
   public void addVertices(List<Point2d> vertices, int numberOfVertices)
   {
      isUpToDate = false;
      for (int i = 0; i < numberOfVertices; i++)
         addVertex(vertices.get(i));
   }

   /**
    * Adds N new vertices to this polygon using an array of {@code Point2d}.
    * Note that this method recycles memory.
    * @param vertices {@code Point2d[]} the list of new vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices to add to this polygon. Note the: {@code vertices.length} can be greater or equal to numberOfVertices.
    */
   public void addVertices(Point2d[] vertices, int numberOfVertices)
   {
      isUpToDate = false;
      for (int i = 0; i < numberOfVertices; i++)
         addVertex(vertices[i]);
   }

   /**
    * Adds N new vertices to this polygon using an array of {@code Point2d}.
    * Note that this method recycles memory.
    * @param vertices {@code double[>=numberOfVertices][>=2]} the array of new vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices to add to this polygon. Note the: {@code vertices.length} can be greater or equal to numberOfVertices.
    */
   public void addVertices(double[][] vertices, int numberOfVertices)
   {
      isUpToDate = false;
      for (int i = 0; i < numberOfVertices; i++)
         addVertex(vertices[i][0], vertices[i][1]);
   }

   /**
    * Adds new vertices to this polygon from another convex polygon.
    * Note that this method recycles memory.
    * @param otherPolygon {@code ConvexPolygon2d} the other convex polygon that is used to add new vertices to this polygon.
    */
   public void addVertices(ConvexPolygon2d otherPolygon)
   {
      isUpToDate = false;
      for (int i = 0; i < otherPolygon.getNumberOfVertices(); i++)
         addVertex(otherPolygon.getVertex(i));
   }

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
      Collections.swap(clockwiseOrderedListOfPoints, indexOfVertexToRemove, numberOfVertices - 1);
      numberOfVertices--;
   }

   private void setOrCreate(Point2d point2d, int i)
   {
      setOrCreate(point2d.getX(), point2d.getY(), i);
   }

   private void setOrCreate(double x, double y, int i)
   {
      while (i >= clockwiseOrderedListOfPoints.size())
         clockwiseOrderedListOfPoints.add(new Point2d());
      clockwiseOrderedListOfPoints.get(i).set(x, y);
   }

   /**
    * Modifies the vertices so they are clockwise ordered.
    * Updates centroid, area and the bounding box.
    * Call this method once new vertices have been added.
    */
   public void update()
   {
      if (isUpToDate)
         return;

      // Need to reorder the vertices so they are clockwise ordered
      if (numberOfVertices >= 3 && !InPlaceConvexHullCalculator2d.isConvexAndClockwise(clockwiseOrderedListOfPoints, numberOfVertices))
      {
         numberOfVertices = InPlaceConvexHullCalculator2d.inPlaceGiftWrapConvexHull2d(clockwiseOrderedListOfPoints, numberOfVertices);
      }
      // Only two distinct vertices: trivial case
      else if (numberOfVertices == 2 && !clockwiseOrderedListOfPoints.get(0).equals(clockwiseOrderedListOfPoints.get(1)))
      {
         Point2d p0 = clockwiseOrderedListOfPoints.get(0);
         Point2d p1 = clockwiseOrderedListOfPoints.get(1);

         if (!(p0.getX() < p1.getX()) || ((p0.getX() == p1.getX()) && (p0.getY() > p1.getY())))
         {
            Collections.swap(clockwiseOrderedListOfPoints, 1, 0);
         }
      }
      // Else, nothing to do, it's either 0 vertex, 1 vertex, 2 identical vertices, or the vertices are already clockwise ordered
      isUpToDate = true;

      updateCentroidAndArea();
      updateBoundingBox();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertices(vertices, numberOfVertices)};
    * 3- {@code update()}.
    * @param vertices {@code List<Point2d>} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the polygon. Note the: {@code vertices.size()} can be greater or equal to numberOfVertices.
    */
   public void setAndUpdate(List<Point2d> vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertices(vertices, numberOfVertices)};
    * 3- {@code update()}.
    * @param vertices {@code Point2d[]} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the polygon. Note the: {@code vertices.length} can be greater or equal to numberOfVertices.
    */
   public void setAndUpdate(Point2d[] vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertices(vertices, numberOfVertices)};
    * 3- {@code update()}.
    * @param vertices {@code double[>=numberOfVertices][>=2]}
    * @param numberOfVertices {@code int}
    */
   public void setAndUpdate(double[][] vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does:
    * 1- clear();
    * 2- addVertices(otherPolygon);
    * 3- update().
    * <p/> TODO There is no need to call update() there, instead update everything from the other polygon to make it faster.<p/>
    * @param otherPolygon {@code ConvexPolygon2d}
    */
   public void setAndUpdate(ConvexPolygon2d otherPolygon)
   {
      clear();
      addVertices(otherPolygon);
      update();
   }

   /**
    * This method does:
    * 1- {@code clear()};
    * 2- {@code addVertices(firstPolygon)};
    * 2- {@code addVertices(secondPolygon)};
    * 3- {@code update()}.
    * <p/> TODO: Make this more efficient by finding the rotating calipers, as in the intersection method.<p/>
    * @param firstPolygon {@code ConvexPolygon2d}
    * @param secondPolygon {@code ConvexPolygon2d}
    */
   public void setAndUpdate(ConvexPolygon2d firstPolygon, ConvexPolygon2d secondPolygon)
   {
      clear();
      addVertices(firstPolygon);
      addVertices(secondPolygon);
      update();
   }

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
         Point2d firstVertex = getVertex(0);
         double minX = firstVertex.getX();
         double minY = firstVertex.getY();
         double maxX = firstVertex.getX();
         double maxY = firstVertex.getY();

         Point2d p;
         for (int i = 1; i < numberOfVertices; i++)
         {
            p = getVertex(i);

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

   // Compute centroid and area of this polygon. Formula taken from http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/
   private void updateCentroidAndArea()
   {
      area = 0.0;
      centroid.set(0, 0);

      if (hasAtLeastThreeVertices())
      {
         double Cx = 0.0;
         double Cy = 0.0;

         // Order counterclockwise to make the area positive
         for (int i = numberOfVertices - 1; i >= 0; i--)
         {
            Point2d ci = getVertex(i);
            Point2d ciMinus1 = getPreviousVertex(i);

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

   public double getArea()
   {
      checkIfUpToDate();
      return area;
   }

   public void getCentroid(Point2d centroid)
   {
      checkIfUpToDate();
      centroid.set(this.centroid);
   }

   public Point2d getCentroid()
   {
      checkIfUpToDate();
      return centroid;
   }

   public BoundingBox2d getBoundingBox()
   {
      return boundingBox;
   }

   public double getBoundingBoxRangeX()
   {
      return boundingBox.getMaxPoint().getX() - boundingBox.getMinPoint().getX();
   }

   public double getBoundingBoxRangeY()
   {
      return boundingBox.getMaxPoint().getY() - boundingBox.getMinPoint().getY();
   }

   public BoundingBox2d getBoundingBoxCopy()
   {
      checkIfUpToDate();
      BoundingBox2d ret = new BoundingBox2d(boundingBox);

      return ret;
   }

   public void getBoundingBox(BoundingBox2d boundingBoxToPack)
   {
      boundingBoxToPack.set(boundingBox);
   }

   /** Return the vertex from a clockwise ordered list */
   public Point2d getVertex(int vertexIndex)
   {
      checkIfUpToDate();
      return getVertexUnsafe(vertexIndex);
   }

   /** Same as getVertex(vertexIndex) but without checking if the polygon has been updated. Be careful when using it! */
   protected Point2d getVertexUnsafe(int vertexIndex)
   {
      checkNonEmpty();
      checkIndexInBoundaries(vertexIndex);
      return clockwiseOrderedListOfPoints.get(vertexIndex);
   }

   /** Return the next vertex from a clockwise ordered list */
   public Point2d getNextVertex(int index)
   {
      return getVertex(getNextVertexIndex(index));
   }

   protected Point2d getNextVertexUnsafe(int index)
   {
      return getVertexUnsafe(getNextVertexIndexUnsafe(index));
   }

   /** Return the previous vertex from a clockwise ordered list */
   public Point2d getPreviousVertex(int index)
   {
      return getVertex(getPreviousVertexIndex(index));
   }

   protected Point2d getPreviousVertexUnsafe(int index)
   {
      return getVertexUnsafe(getPreviousVertexIndexUnsafe(index));
   }

   /** Return the vertex from a counter clockwise ordered list */
   public Point2d getVertexCCW(int vertexIndex)
   {
      checkIfUpToDate();
      checkNonEmpty();
      checkIndexInBoundaries(vertexIndex);
      return clockwiseOrderedListOfPoints.get(numberOfVertices - 1 - vertexIndex);
   }

   /** Return the next vertex from a counter clockwise ordered list */
   public Point2d getNextVertexCCW(int index)
   {
      return getVertexCCW(getNextVertexIndex(index));
   }

   /** Return the previous vertex from a counter clockwise ordered list */
   public Point2d getPreviousVertexCCW(int index)
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
    * Scale this convex polygon about its centroid, i.e. once scaled the polygon centroid remains unchanged.
    * @param scaleFactor
    */
   public void scale(double scaleFactor)
   {
      scale(centroid, scaleFactor);
   }

   /**
    * Scale this convex polygon about pointToScaleAbout.
    * @param scaleFactor
    */
   public void scale(Point2d pointToScaleAbout, double scaleFactor)
   {
      checkIfUpToDate();
      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2d vertex = getVertexUnsafe(i);
         vertex.sub(pointToScaleAbout);
         vertex.scale(scaleFactor);
         vertex.add(pointToScaleAbout);
      }
      update();
   }

   // here ---------------------
   // nuke this
   public Point2d getClosestVertexWithRayCopy(Line2d ray, boolean throwAwayVerticesOutsideRay)
   {
      checkIfUpToDate();
      // O(n) for now, maybe there's a faster way?

      Point2d ret = null;
      double minDistanceSquared = Double.POSITIVE_INFINITY;

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2d vertex = getVertex(i);
         double distanceSquared;
         Point2d rayOrigin = ray.getPoint();
         Vector2d rayDirection = ray.getNormalizedVector();
         double dotProductOfRayDirectionAndRayOriginToVertex = rayDirection.getX() * (vertex.getX() - rayOrigin.getX())
               + rayDirection.getY() * (vertex.getY() - rayOrigin.getY());
         // The sign of this dot product indicates if the point is "outside" the ray, meaning the closest point of the ray to the vertex is the ray origin.
         if (dotProductOfRayDirectionAndRayOriginToVertex >= 0.0)
            distanceSquared = ray.distanceSquared(vertex);
         else if (!throwAwayVerticesOutsideRay)
            distanceSquared = vertex.distanceSquared(rayOrigin);
         else // Throw away that vertex
            continue;

         if (distanceSquared < minDistanceSquared)
         {
            ret = vertex;
            minDistanceSquared = distanceSquared;
         }
      }

      if (ret == null)
         return null;
      else
         return new Point2d(ret);
   }

   // nuke this
   public boolean getClosestVertexWithRay(Point2d closestVertexToPack, Line2d ray, boolean throwAwayVerticesOutsideRay)
   {
      checkIfUpToDate();
      // O(n) for now, maybe there's a faster way?

      double minDistanceSquared = Double.POSITIVE_INFINITY;
      boolean success = false;

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2d vertex = getVertex(i);
         double distanceSquared;
         Point2d rayOrigin = ray.getPoint();
         Vector2d rayDirection = ray.getNormalizedVector();
         double dotProductOfRayDirectionAndRayOriginToVertex = rayDirection.getX() * (vertex.getX() - rayOrigin.getX())
               + rayDirection.getY() * (vertex.getY() - rayOrigin.getY());
         // The sign of this dot product indicates if the point is "outside" the ray, meaning the closest point of the ray to the vertex is the ray origin.
         if (dotProductOfRayDirectionAndRayOriginToVertex >= 0.0)
            distanceSquared = ray.distanceSquared(vertex);
         else if (!throwAwayVerticesOutsideRay)
            distanceSquared = vertex.distanceSquared(rayOrigin);
         else // Throw away that vertex
            continue;

         if (distanceSquared < minDistanceSquared)
         {
            closestVertexToPack.set(vertex);
            minDistanceSquared = distanceSquared;
            success = true;
         }
      }

      return success;
   }

   // nuke this
   public double distanceToClosestVertex(Point2d point)
   {
      checkIfUpToDate();
      // O(n) for now, maybe there's a faster way?

      double minDistance = Double.POSITIVE_INFINITY;

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2d vertex = getVertex(i);
         double distance = vertex.distance(point);
         if (distance < minDistance)
            minDistance = distance;
      }

      return minDistance;
   }

   /**
    * Returns all of the vertices that are visible from the observerPoint2d, in left to right order.
    * If the observerPoint2d is inside the polygon, returns null.
    *
    * @param observerPoint2d Point2d
    * @return ArrayList<Point2d>
    */
   public ArrayList<Point2d> getAllVisibleVerticesFromOutsideLeftToRightCopy(Point2d observerPoint2d)
   {
      checkIfUpToDate();
      if (!hasAtLeastTwoVertices())
      {
         ArrayList<Point2d> points = new ArrayList<Point2d>();
         for (int i = 0; i < numberOfVertices; i++)
         {
            points.add(new Point2d(getVertex(i)));
         }

         return points;
      }

      int[] indices = ConvexPolygon2dCalculator.getLineOfSightVertexIndicesCopy(observerPoint2d, this);

      if (indices == null)
         return null;

      ArrayList<Point2d> ret = new ArrayList<Point2d>();

      int leavingLeftEdge = indices[0];
      int enteringRightEdge = indices[1];

      int edgeToAdd = leavingLeftEdge;

      while (edgeToAdd != enteringRightEdge)
      {
         Point2d point = new Point2d(getVertex(edgeToAdd));
         ret.add(point);

         edgeToAdd = getPreviousVertexIndex(edgeToAdd);
      }

      Point2d point = new Point2d(getVertex(edgeToAdd));
      ret.add(point);

      return ret;
   }

   /**
    * Returns the two LineSegment2ds that are the first segments around the corner that cannot be seen from the observerPoint2d.
    * If the observerPoint2d is null returns null. The line segments are returned in order of left, then right.
    * The line segments go from the line of sight points to the points that are not in view, but are around the corner.
    *
    * @param observerPoint2d Point2d marking the point of observation of this ConvexPolygon2d.
    * @return LineSegment2d[] Two line segments going from the line of sight points to the first points around the corners that are out of sight.
    * null if the observerPoint2d is inside this ConvexPolygon2d.
    */
   public LineSegment2d[] getAroundTheCornerEdgesCopy(Point2d observerPoint2d)
   {
      checkIfUpToDate();
      int[] indices = ConvexPolygon2dCalculator.getLineOfSightVertexIndicesCopy(observerPoint2d, this);

      // TODO indices == null || indices.length == 1 is deprecated
      if (indices == null || indices.length == 1 || indices[0] == indices[1])
         return null;

      int leavingLeftEdge = indices[0];
      int enteringRightEdge = indices[1];

      int aroundCornerOfLeftEdge = leavingLeftEdge + 1;
      if (aroundCornerOfLeftEdge == numberOfVertices)
         aroundCornerOfLeftEdge = 0;

      int aroundCornerRightEdge = enteringRightEdge - 1;
      if (aroundCornerRightEdge < 0)
         aroundCornerRightEdge = numberOfVertices - 1;

      Point2d leftPoint = new Point2d(getVertex(leavingLeftEdge));
      Point2d leftPointAroundCorner = new Point2d(getVertex(aroundCornerOfLeftEdge));

      Point2d rightPoint = new Point2d(getVertex(enteringRightEdge));
      Point2d rightPointAroundCorner = new Point2d(getVertex(aroundCornerRightEdge));

      LineSegment2d leftLineSegment = new LineSegment2d(leftPoint, leftPointAroundCorner);
      LineSegment2d rightLineSegment = new LineSegment2d(rightPoint, rightPointAroundCorner);

      return new LineSegment2d[] {leftLineSegment, rightLineSegment};
   }

   public LineSegment2d[] getNearestEdges(Point2d testPoint)
   {
      checkIfUpToDate();
      if (!hasAtLeastTwoVertices())
      {
         return null;
      }

      int numberOfEdges = getNearestEdgeIndices(testPoint, tempTwoIndices);
      if (numberOfEdges == 0)
         return null;

      LineSegment2d[] ret = new LineSegment2d[numberOfEdges];
      for (int i = 0; i < numberOfEdges; i++)
      {
         int edgeIndex = tempTwoIndices[i];
         LineSegment2d edge = new LineSegment2d(getVertex(edgeIndex), getNextVertex(edgeIndex));

         ret[i] = edge;
      }

      return ret;
   }

   private int getNearestEdgeIndices(Point2d pointToProject, int[] indicesToPack)
   {
      if (indicesToPack.length != 2)
         throw new RuntimeException("Expected array of length two");

      // First find the line of sight vertices. If inside the Polygon, then return null.
      if (!ConvexPolygon2dCalculator.getLineOfSightVertexIndices(pointToProject, tempTwoIndices, this))
      {
         indicesToPack[0] = -1;
         indicesToPack[1] = -1;
         return 0;
      }

      int leftEdge = (tempTwoIndices[0] - 1 + numberOfVertices) % numberOfVertices;
      int rightEdge = tempTwoIndices[1];

      // Binary search maintaining nearest left and nearest right vertices until they are adjacent: //TODO remove the q from maintaining
      while ((rightEdge != leftEdge) && (getNextVertexIndex(rightEdge) != leftEdge))
      {
         int testEdge = ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(leftEdge, rightEdge, this);

         if (isEdgeFullyToTheLeftOfObserver(testEdge, pointToProject))
         {
            leftEdge = testEdge;
         }
         else if (isEdgeFullyToTheRight(testEdge, pointToProject))
         {
            rightEdge = testEdge;
         }
         else
         {
            indicesToPack[0] = testEdge;
            indicesToPack[1] = -1;
            return 1;
         }
      }

      // If edges are the same, then just return one,
      if (leftEdge == rightEdge)
      {
         indicesToPack[0] = leftEdge;
         indicesToPack[1] = -1;
         return 1;
      }

      // Otherwise check if left is fully to the left or right is fully to the right:
      boolean leftEdgeIsFullyLeft = isEdgeFullyToTheLeftOfObserver(leftEdge, pointToProject);
      boolean rightEdgeIsFullyRight = isEdgeFullyToTheRight(rightEdge, pointToProject);

      // They should never both be not fully left or right...
      if (!leftEdgeIsFullyLeft && !rightEdgeIsFullyRight)
      {
         throw new RuntimeException("Should never get here!");
      }

      if (!leftEdgeIsFullyLeft)
      {
         indicesToPack[0] = leftEdge;
         indicesToPack[1] = -1;
         return 1;
      }
      if (!rightEdgeIsFullyRight)
      {
         indicesToPack[0] = rightEdge;
         indicesToPack[1] = -1;
         return 1;
      }

      indicesToPack[0] = leftEdge;
      indicesToPack[1] = rightEdge;
      return 2;
   }

   /**
    * Check if when projected on the edge line, the observer point is located at the left (when looking from inside the polygon) outside of the edge.
    * @param edgeIndex refers to the index of the first vertex of the edge.
    * @param observerPoint2d point that is compared with the edge.
    * @return true if the observer point is located at the left outside of the edge.
    */
   private boolean isEdgeFullyToTheLeftOfObserver(int edgeIndex, Point2d observerPoint2d)
   {
      Point2d vertex = getVertex(edgeIndex);
      Point2d nextVertex = getNextVertex(edgeIndex);

      // Vector perpendicular to the edge and pointing outside the polygon
      double edgeNormalX = -(nextVertex.getY() - vertex.getY());
      double edgeNormalY = (nextVertex.getX() - vertex.getX());

      double vertexToObserverX = observerPoint2d.getX() - vertex.getX();
      double vertexToObserverY = observerPoint2d.getY() - vertex.getY();

      // Equivalent to looking at the sign of the angle formed by (egdeNormal -> vertexToPoint)
      double crossProduct = edgeNormalX * vertexToObserverY - edgeNormalY * vertexToObserverX;

      // If positive, that means vertexToPoint is pointing outside the edge
      // Thus, the observer point is totally on the left of the edge
      return crossProduct > 0.0;
   }

   /**
    * Check if when projected on the edge line, the observer point is located at the right (when looking from inside the polygon) outside of the edge.
    * @param edgeIndex refers to the index of the first vertex of the edge.
    * @param observerPoint2d point that is compared with the edge.
    * @return true if the observer point is located at the right outside of the edge.
    */
   private boolean isEdgeFullyToTheRight(int edgeIndex, Point2d observerPoint2d)
   {
      Point2d vertex = getVertex(edgeIndex);
      Point2d nextVertex = getNextVertex(edgeIndex);

      // Vector perpendicular to the edge and pointing outside the polygon
      double edgeNormalX = -(nextVertex.getY() - vertex.getY());
      double edgeNormalY = (nextVertex.getX() - vertex.getX());

      double nextVertexToObserverX = observerPoint2d.getX() - nextVertex.getX();
      double nextVertexToObserverY = observerPoint2d.getY() - nextVertex.getY();

      // Equivalent to looking at the sign of the angle formed by (egdeNormal -> vertexToPoint)
      double crossProduct = edgeNormalX * nextVertexToObserverY - edgeNormalY * nextVertexToObserverX;

      // If positive, that means vertexToPoint is pointing outside the edge
      // Thus, the observer point is totally on the left of the edge
      return crossProduct < 0.0;
   }

   @Override
   public Point2d[] intersectionWith(LineSegment2d lineSegment2d)
   {
      checkIfUpToDate();
      if (hasExactlyOneVertex())
      {
         if (lineSegment2d.isPointOnLineSegment(getVertex(0)))
            return new Point2d[] {new Point2d(getVertex(0))};
         else
            return null;
      }

      if (hasExactlyTwoVertices())
      {
         LineSegment2d polygonAsSegment = new LineSegment2d(getVertex(0), getVertex(1));
         Point2d intersection = polygonAsSegment.intersectionWith(lineSegment2d);
         if (intersection != null)
            return new Point2d[] {intersection};
         else
            return null;
      }

      Point2d[] endPoints = lineSegment2d.endpoints;

      Point2d outsidePoint, otherPoint;
      boolean onePointIsInside;

      // First make sure that the ray goes from an outside point...
      if (ConvexPolygon2dCalculator.isPointInside(endPoints[0], this))
      {
         if (ConvexPolygon2dCalculator.isPointInside(endPoints[1], this))
            return null; // Both Points are inside!

         outsidePoint = endPoints[1];
         otherPoint = endPoints[0];
         onePointIsInside = true;
      }
      else if (ConvexPolygon2dCalculator.isPointInside(endPoints[1], this))
      {
         outsidePoint = endPoints[0];
         otherPoint = endPoints[1];
         onePointIsInside = true;
      }
      else
      {
         outsidePoint = endPoints[0];
         otherPoint = endPoints[1];
         onePointIsInside = false;
      }

      // +++ jjc 090625 swap the line points to set the first point as the closest point to the polygon so that line segment intersections are properly found.
      Line2d line2d;
      if (distance(outsidePoint) > distance(otherPoint))
      {
         line2d = new Line2d(otherPoint, outsidePoint);
      }
      else
      {
         line2d = new Line2d(outsidePoint, otherPoint);
      }

      // Line2d line2d = new Line2d(outsidePoint, otherPoint);

      Point2d[] intersections = intersectionWithRayCopy(line2d);
      if (intersections == null)
         return null;

      if (!onePointIsInside)
         return intersections;

      return new Point2d[] {intersections[0]}; // Only return the entering point, if the segment has one point inside the polygon.
   }

   @Override
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

   public LineSegment2d getClosestEdgeCopy(Point2d point)
   {
      checkIfUpToDate();
      if (!hasAtLeastTwoVertices())
      {
         return null;
      }

      int[] closestEdgeVertices = getClosestEdgeVertexIndicesInClockwiseOrderedList(point);

      Point2d prevP = getVertex(closestEdgeVertices[0]);
      Point2d p = getVertex(closestEdgeVertices[1]);
      LineSegment2d closestEdge = new LineSegment2d(prevP, p);

      return closestEdge;
   }

   public boolean getClosestEdge(LineSegment2d closestEdgeToPack, Point2d point)
   {
      checkIfUpToDate();
      if (!hasAtLeastTwoVertices())
      {
         return false;
      }

      int[] closestEdgeVertices = getClosestEdgeVertexIndicesInClockwiseOrderedList(point);

      Point2d prevP = getVertex(closestEdgeVertices[0]);
      Point2d p = getVertex(closestEdgeVertices[1]);
      closestEdgeToPack.set(prevP, p);

      return true;
   }

   private final LineSegment2d tempEdge = new LineSegment2d();

   protected int[] getClosestEdgeVertexIndicesInClockwiseOrderedList(Point2d point)
   {
      checkIfUpToDate();
      if (!hasAtLeastTwoVertices())
      {
         return null;
      }

      // TODO: Create a more efficient algorithm to find this distance
      double smallestDistance = Double.POSITIVE_INFINITY;

      int prevIndex = getNumberOfVertices() - 1;

      int[] closestEdgeVertexIndicesInClockwiseOrderedList = new int[2];
      closestEdgeVertexIndicesInClockwiseOrderedList[0] = prevIndex;
      closestEdgeVertexIndicesInClockwiseOrderedList[1] = 0;

      for (int index = 0; index < getNumberOfVertices(); index++)
      {
         Point2d currentVertex = getVertex(index);
         Point2d prevVertex = getVertex(prevIndex);
         tempEdge.set(prevVertex, currentVertex);
         double dist = tempEdge.distance(point);
         if (dist < smallestDistance)
         {
            closestEdgeVertexIndicesInClockwiseOrderedList[0] = prevIndex;
            closestEdgeVertexIndicesInClockwiseOrderedList[1] = index;
            smallestDistance = dist;
         }

         prevIndex = index;
      }

      return closestEdgeVertexIndicesInClockwiseOrderedList;
   }

   @Override
   public double distance(Line2d line)
   {
      checkIfUpToDate();
      throw new RuntimeException("Not yet implemented");
   }

   @Override
   public double distance(LineSegment2d lineSegment)
   {
      checkIfUpToDate();
      throw new RuntimeException("Not yet implemented");
   }

   @Override
   public double distance(ConvexPolygon2d convexPolygon)
   {
      checkIfUpToDate();
      throw new RuntimeException("Not yet implemented");
   }

   @Override
   public String toString()
   {
      String ret = "";
      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2d vertex = clockwiseOrderedListOfPoints.get(i);
         ret = ret + "{" + vertex.getX() + ", " + vertex.getY() + "}," + "\n";
      }

      return ret;
   }

   @Override
   public void applyTransform(RigidBodyTransform transform)
   {
      checkIsTransformationInPlane(transform);
      applyTransformAndProjectToXYPlane(transform);
   }

   @Override
   public void applyTransformAndProjectToXYPlane(RigidBodyTransform transform)
   {
      isUpToDate = false;

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2d vertex = getVertexUnsafe(i);
         tempVertex3d.set(vertex.getX(), vertex.getY(), 0.0);
         transform.transform(tempVertex3d);
         vertex.setX(tempVertex3d.getX());
         vertex.setY(tempVertex3d.getY());
      }
      update();
   }

   private final Point3d tempVertex3d = new Point3d();

   @Override
   public ConvexPolygon2d applyTransformCopy(RigidBodyTransform transform)
   {
      ConvexPolygon2d copy = new ConvexPolygon2d(this);
      copy.applyTransform(transform);
      return copy;
   }

   @Override
   public ConvexPolygon2d applyTransformAndProjectToXYPlaneCopy(RigidBodyTransform transform)
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

   public Point2d getMinXMaxYPointCopy()
   {
      checkIfUpToDate();
      return new Point2d(getVertex(minXmaxY_index));
   }

   public Point2d getMinXMinYPointCopy()
   {
      checkIfUpToDate();
      return new Point2d(getVertex(minXminY_index));
   }

   public Point2d getMaxXMaxYPointCopy()
   {
      checkIfUpToDate();
      return new Point2d(getVertex(maxXmaxY_index));
   }

   public Point2d getMaxXMinYPointCopy()
   {
      checkIfUpToDate();
      return new Point2d(getVertex(maxXminY_index));
   }

   public double perimeter()
   {
      checkIfUpToDate();

      // Sum the lengths of each of the edges:
      double perimeter = 0.0;
      for (int i = 0; i < numberOfVertices; i++)
      {
         perimeter += getVertex(i).distance(getNextVertex(i));
      }

      return perimeter;
   }

   public Point2d pointOnPerimeterGivenParameter(double parameter)
   {
      checkIfUpToDate();
      if (hasExactlyOneVertex())
      {
         return new Point2d(getVertex(0));
      }

      double perimeter = perimeter();
      double adjustedParameter = (parameter % 1 >= 0.0) ? parameter % 1 : parameter % 1 + 1.0;
      double desiredDistance = adjustedParameter * perimeter;

      int i = 0;
      double distance = 0.0;
      double edgeLength = getVertex(0).distance(getNextVertex(0));
      while (desiredDistance > distance + edgeLength)
      {
         distance += edgeLength;
         i++;
         edgeLength = getVertex(i).distance(getNextVertex(i));
      }

      LineSegment2d edge = new LineSegment2d(getVertex(i), getNextVertex(i));
      double desiredDistanceAlongEdge = desiredDistance - distance;
      double parameterAlongEdge = desiredDistanceAlongEdge / edge.length();

      return edge.pointBetweenEndPointsGivenParameter(parameterAlongEdge);
   }

   public void pullPointTowardsCentroid(Point2d point, double percent)
   {
      checkIfUpToDate();
      double x = centroid.getX() + (point.getX() - centroid.getX()) * (1.0 - percent);
      double y = centroid.getY() + (point.getY() - centroid.getY()) * (1.0 - percent);

      point.set(x, y);
   }

   public Point2d pullTowardsCentroidCopy(Point2d point, double percent)
   {
      checkIfUpToDate();
      Point2d copy = new Point2d(point);
      pullPointTowardsCentroid(copy, percent);

      return copy;
   }

   private void checkIsTransformationInPlane(RigidBodyTransform transform)
   {
      if (!isTransformationInPlane(transform))
      {
         throw new RuntimeException("Cannot transform FrameConvexPolygon2d to a plane with a different surface normal");
      }
   }

   private final Matrix3d tempRotation = new Matrix3d();

   private boolean isTransformationInPlane(RigidBodyTransform transform)
   {
      // arguably not a sufficient condition. ReferenceFrame2d needed!
      transform.getRotation(tempRotation);

      return ReferenceFrame.isRotationInPlane(tempRotation);
   }

   protected void getPointsInClockwiseOrder(int startIndexInclusive, int endIndexInclusive, ArrayList<Point2d> pointList)
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

   public boolean pointIsOnPerimeter(Point2d point)
   {
      checkIfUpToDate();
      if (hasExactlyOneVertex())
      {
         return getVertex(0).equals(point);
      }

      Point2d vertex0;
      Point2d vertex1 = getVertex(0);
      LineSegment2d edge;

      for (int i = 0; i < numberOfVertices; i++)
      {
         vertex0 = vertex1;
         vertex1 = getNextVertex(i);
         edge = new LineSegment2d(vertex0, vertex1);

         if (edge.isPointOnLineSegment(point))
            return true;
      }

      return false;
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

   public List<Vector2d> getOutSideFacingOrthoNormalVectorsCopy()
   {
      checkIfUpToDate();

      if (numberOfVertices == 1)
         return null;

      Vector2d[] outsideFacingOrthogonalVectors = new Vector2d[numberOfVertices];
      for (int i = 0; i < numberOfVertices; i++)
      {
         outsideFacingOrthogonalVectors[i] = new Vector2d();
      }

      getOutSideFacingOrthoNormalVectors(outsideFacingOrthogonalVectors);

      return Arrays.asList(outsideFacingOrthogonalVectors);
   }

   public void getOutSideFacingOrthoNormalVectors(Vector2d[] orthoNormalVectorsToPack)
   {
      checkIfUpToDate();

      if (orthoNormalVectorsToPack == null || orthoNormalVectorsToPack.length != numberOfVertices)
      {
         throw new RuntimeException("orthogonalVectorsToPack is null or wrong length");
      }

      if (numberOfVertices == 1)
      {
         throw new UndefinedOperationException("orthoNormal vectors undefined for a point");
      }

      for (int i = 0; i < numberOfVertices; i++)
      {
         orthoNormalVectorsToPack[i].set(getNextVertex(i));
         orthoNormalVectorsToPack[i].sub(getVertex(i));

         GeometryTools.getPerpendicularVector(orthoNormalVectorsToPack[i], orthoNormalVectorsToPack[i]);

         orthoNormalVectorsToPack[i].normalize();
      }
   }

   @Override
   public double distance(Point2d point)
   {
      checkIfUpToDate();
      tempPoint.set(point);
      orthogonalProjection(tempPoint);

      return point.distance(tempPoint);
   }

   @Override
   public Point2d orthogonalProjectionCopy(Point2d point)
   {
      checkIfUpToDate();
      Point2d copy = new Point2d(point);
      orthogonalProjection(copy);

      return copy;
   }

   /**
    * Compute the orthogonal projection of the given point and modify it to store the result.
    */
   @Override
   public void orthogonalProjection(Point2d point2d)
   {
      checkIfUpToDate();
      if (hasExactlyOneVertex())
      {
         point2d.set(getVertex(0));
         return;
      }

      tempPoint.set(point2d);
      int numberOfEdges = getNearestEdgeIndices(tempPoint, tempTwoIndices);
      if (numberOfEdges == 0)
         return; // point2d must be inside polygon, so leave it as it is.

      int leftEdge, rightEdge;
      if (numberOfEdges == 1)
      {
         leftEdge = rightEdge = tempTwoIndices[0];
      }
      else
      {
         leftEdge = tempTwoIndices[0];
         rightEdge = tempTwoIndices[1];
      }

      // Two adjacent edges. Return the left vertex:
      if (leftEdge != rightEdge)
         point2d.set(getVertex(leftEdge));

      // Just one edge. Find the point on the edge:
      Point2d firstEdgeVertex = getVertex(leftEdge);
      Point2d secondEdgeVertex = getNextVertex(leftEdge);

      tempVector1.set(point2d); // first vertex to point
      tempVector1.sub(firstEdgeVertex);

      tempVector2.set(secondEdgeVertex); // edge vector
      tempVector2.sub(firstEdgeVertex);

      if (tempVector2.lengthSquared() < 1e-10)
      {
         point2d.set(firstEdgeVertex);
         return;
      }

      double dotProduct = tempVector2.dot(tempVector1);
      double lengthSquared = tempVector2.lengthSquared();
      double alpha = dotProduct / lengthSquared;

      // Need to keep alpha between 0.0 and 1.0 since if only one edge is seen, the projection can be outside the edge.
      if (alpha < 0.0)
         alpha = 0.0;
      if (alpha > 1.0)
         alpha = 1.0;
      tempVector2.scale(alpha);

      point2d.set(firstEdgeVertex);
      point2d.add(tempVector2);

      // Make sure the returned point is inside the polygon by nudging it a little toward the centroid.
      // This will all but guarantee that projections are then inside.
      tempVector1.set(centroid);
      tempVector1.sub(point2d);
      tempVector1.scale(1.0e-12);
      point2d.add(tempVector1);
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
      if (clockwiseOrderedListOfPoints.isEmpty())
         clockwiseOrderedListOfPoints.add(new Point2d());

      Point2d point = clockwiseOrderedListOfPoints.get(0);
      point.set(0.0, 0.0);

      isUpToDate = false;
      update();
   }

   @Override
   public void setToNaN()
   {
      numberOfVertices = 1;
      if (clockwiseOrderedListOfPoints.isEmpty())
         clockwiseOrderedListOfPoints.add(new Point2d());

      Point2d point = clockwiseOrderedListOfPoints.get(0);
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
         Point2d point = clockwiseOrderedListOfPoints.get(i);

         if (Double.isNaN(point.getX()))
            return true;
         if (Double.isNaN(point.getY()))
            return true;
      }

      return false;
   }

   // --- remove these eventually ---
   public boolean isPointInside(double x, double y)
   {
      return ConvexPolygon2dCalculator.isPointInside(x, y, this);
   }

   public boolean isPointInside(double x, double y, double epsilon)
   {
      return ConvexPolygon2dCalculator.isPointInside(x, y, epsilon, this);
   }

   public boolean isPointInside(Point2d point)
   {
      return ConvexPolygon2dCalculator.isPointInside(point, this);
   }

   public boolean isPointInside(Point2d point, double epsilon)
   {
      return ConvexPolygon2dCalculator.isPointInside(point, epsilon, this);
   }

   public ConvexPolygon2d translateCopy(Tuple2d translation)
   {
      return ConvexPolygon2dCalculator.translatePolygonCopy(translation, this);
   }

   public LineSegment2d[] getIntersectingEdgesCopy(Line2d line)
   {
      // wherever this is used the method should be switched to intersection with ray once that is created.
      return ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line, this);
   }

   public int intersectionWithRay(Line2d ray, Point2d intersectionToPack1, Point2d intersectionToPack2)
   {
      return ConvexPolygon2dCalculator.intersectionWithRay(ray, intersectionToPack1, intersectionToPack2, this);
   }

   // --- implementations that make garbage ---
   @Override
   public Point2d[] intersectionWith(Line2d line)
   {
      return ConvexPolygon2dCalculator.intersectionWithLineCopy(line, this);
   }

   public Point2d[] intersectionWithRayCopy(Line2d ray)
   {
      return ConvexPolygon2dCalculator.intersectionWithRayCopy(ray, this);
   }
}
