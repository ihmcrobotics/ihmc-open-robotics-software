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
public class ConvexPolygon2d implements Geometry2d
{
   private static final boolean DEBUG = false;

   private final ArrayList<Point2d> clockwiseOrderedListOfPoints = new ArrayList<Point2d>();
   private final BoundingBox2d boundingBox = new BoundingBox2d();
   private final Point2d centroid = new Point2d();
   private int numberOfVertices = 0;
   private boolean isUpToDate = false;
   private double area;

   private int minX_index = 0, maxX_index = 0, minY_index = 0, maxY_index = 0;
   private int minXminY_index = 0, maxXminY_index = 0, maxXmaxY_index = 0;
   private final int minXmaxY_index = 0;

   private final Point2d tempPoint2dForDistance = new Point2d();

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

      for (int i=0; i<numberOfPossiblePoints; i++)
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
      setOrCreate(point2d.x, point2d.y, i);
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
         double minX = firstVertex.x;
         double minY = firstVertex.y;
         double maxX = firstVertex.x;
         double maxY = firstVertex.y;

         Point2d p;
         for (int i = 1; i < numberOfVertices; i++)
         {
            p = getVertex(i);

            if (p.x < minX)
            {
               minX = p.x;
               minX_index = i;
               minXminY_index = i;
            }
            else if (p.x > maxX)
            {
               maxX = p.x;
               maxX_index = i;
               maxXmaxY_index = i;
               maxXminY_index = i;
            }
            else if (p.x == getVertex(minXminY_index).x && p.y < getVertex(minXminY_index).y)
            {
               minXminY_index = i;
            }
            else if (p.x == getVertex(maxXminY_index).x) // any case: getVertex(maxXmaxY_index).x == getVertex(maxXminY_index).x
            {
               if (p.y < getVertex(maxXminY_index).y)
               {
                  maxXminY_index = i;
               }
               else if (p.y > getVertex(maxXmaxY_index).y)
               {
                  maxXmaxY_index = i;
               }
            }

            if (p.y <= minY)
            {
               minY = p.y;
               minY_index = i;
            }
            else if (p.y >= maxY)
            {
               maxY = p.y;
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

         Cx *= 1.0 / (6.0 * area);
         Cy *= 1.0 / (6.0 * area);

         centroid.set(Cx, Cy);
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

   public double[] distanceToEachVertex(Point2d point)
   {
      checkIfUpToDate();
      double[] ret = new double[numberOfVertices];
      for (int i = 0; i < numberOfVertices; i++)
      {
         ret[i] = point.distance(getVertex(i));
      }

      return ret;
   }
   
   /**
    * Returns distance from the point to the boundary of this polygon.
    * Positive number if inside. Negative number if outside.
    * If inside, the distance is the exact distance to an edge.
    * If outside, the negative distance is an underestimate.
    * It is actually the furthest distance from a projected edge that
    * it is outside of.
    * @param point
    * @return distance from point to this polygon.
    */
   public double getDistanceInside(Point2d point)
   {
      checkIfUpToDate();
            
      if (numberOfVertices == 1)
      {
         return -point.distance(getVertex(0));
      }
      
      if (numberOfVertices == 2)
      {
         Point2d pointOne = getVertex(0);
         Point2d pointTwo = getVertex(1);
         
         return -Math.abs(computeDistanceToSideOfSegment(point, pointOne, pointTwo));
      }
      
      double closestDistance = Double.POSITIVE_INFINITY;
      
      for (int index = 0; index < numberOfVertices; index++)
      {
         Point2d pointOne = getVertex(index);
         int nextIndex = index+1;
         if (nextIndex == numberOfVertices)
            nextIndex = 0;
         
         Point2d pointTwo = getVertex(nextIndex);
         
         double distance = computeDistanceToSideOfSegment(point, pointOne, pointTwo);
         if (distance < closestDistance)
         {
            closestDistance = distance;
         }
      }
      
      return closestDistance;
   }

   private double computeDistanceToSideOfSegment(Point2d point, Point2d pointOne, Point2d pointTwo)
   {
      double x0 = point.getX();
      double y0 = point.getY();
      
      double x1 = pointOne.getX();
      double y1 = pointOne.getY();
      
      double x2 = pointTwo.getX();
      double y2 = pointTwo.getY();
      
      double numerator = (y2 - y1) * x0 - (x2 - x1) * y0 + x2*y1 - y2*x1;
      double denominator = Math.sqrt((y2-y1) * (y2-y1) + (x2-x1) * (x2-x1));
      
      return numerator/denominator;
   }

   public Point2d getClosestVertexCopy(Point2d point)
   {
      checkIfUpToDate();
      // throw new RuntimeException("Not yet implemented");
      // O(n) for now, maybe there's a faster way?

      double minDistance = Double.POSITIVE_INFINITY;
      Point2d ret = null;
      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2d vertex = getVertex(i);
         double distance = vertex.distance(point);
         if (distance < minDistance)
         {
            ret = vertex;
            minDistance = distance;
         }
      }

      return ret;
   }

   public Point2d getClosestVertexCopy(Line2d line)
   {
      checkIfUpToDate();
      // O(n) for now, maybe there's a faster way?

      Point2d ret = null;
      double minDistanceSquared = Double.POSITIVE_INFINITY;

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2d vertex = getVertex(i);
         double distanceSquared = line.distanceSquared(vertex);
         if (distanceSquared < minDistanceSquared)
         {
            ret = vertex;
            minDistanceSquared = distanceSquared;
         }
      }

      if (ret == null)
      {
         System.err.println(this.getClass().getSimpleName() + ": Distance squared to vertex 0: " + line.distanceSquared(getVertex(0)));
         return null;
      }
      else
         return new Point2d(ret);
   }

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
         double dotProductOfRayDirectionAndRayOriginToVertex = rayDirection.x * (vertex.x - rayOrigin.x) + rayDirection.y * (vertex.y - rayOrigin.y);
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
         double dotProductOfRayDirectionAndRayOriginToVertex = rayDirection.x * (vertex.x - rayOrigin.x) + rayDirection.y * (vertex.y - rayOrigin.y);
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

   public boolean isPointInside(Point2d point)
   {
      checkIfUpToDate();
      return isPointInside(point, 0.0);
   }

   /**
    * isPointInside
    * Determines whether a point is inside the convex polygon (point in polygon test).
    * Uses the orientation method (Nordbeck, Rystedt, 1967)
    * Test is only valid for convex polygons, and only if the vertices are ordered clockwise.
    *
    * @param point Point2d the point to be tested
    * @return boolean true if the point is inside the polygon
    */
   public boolean isPointInside(Point2d point, double epsilon)
   {
      checkIfUpToDate();
      return isPointInside(point.x, point.y, epsilon);
   }

   public ConvexPolygon2d translateCopy(Tuple2d translation)
   {
      checkIfUpToDate();
      ArrayList<Point2d> points = new ArrayList<Point2d>(numberOfVertices);

      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2d newPoint = new Point2d(getVertex(i));
         newPoint.add(translation);
         points.add(newPoint);
      }

      return new ConvexPolygon2d(points);
   }

   public boolean isPointInside(double x, double y)
   {
      checkIfUpToDate();
      return isPointInside(x, y, 0.0);
   }

   /**
    * isPointInside
    * Determines whether a point is inside the convex polygon (point in polygon test).
    * Uses the orientation method (Nordbeck, Rystedt, 1967)
    * Test is only valid for convex polygons, and only if the vertices are ordered clockwise.
    *
    * @param x
    * @param y
    * @return boolean true if the point is inside the polygon
    */
   public boolean isPointInside(double x, double y, double epsilon)
   {
      checkIfUpToDate();
      if (hasAtLeastThreeVertices())
      {
         if (x < boundingBox.getMinPoint().x)
            return false;
         if (y < boundingBox.getMinPoint().y)
            return false;

         if (x > boundingBox.getMaxPoint().x)
            return false;
         if (y > boundingBox.getMaxPoint().y)
            return false;

         // Determine whether the point is on the right side of each edge:
         for (int i = 0; i < numberOfVertices; i++)
         {
            double x0 = getVertex(i).x;
            double y0 = getVertex(i).y;

            double x1 = getNextVertex(i).x;
            double y1 = getNextVertex(i).y;

            if ((y - y0) * (x1 - x0) - (x - x0) * (y1 - y0) > epsilon)
            {
               return false;
            }
         }

         return true;
      }
      else if (hasAtLeastTwoVertices())
      {
         Point2d point0 = getVertex(0);
         Point2d point1 = getVertex(1);
         LineSegment2d lineSegment = new LineSegment2d(point0.getX(), point0.getY(), point1.getX(), point1.getY());

         return lineSegment.isPointOnLineSegment(new Point2d(x, y));
      }
      else if (hasAtLeastOneVertex())
      {
         Point2d point = getVertex(0);

         return (point.getX() == x) && (point.getY() == y);
      }

      return false;
   }

   /**
    * areAllPointsInside
    * Determines whether all the points in points are inside the convex
    * polygon.
    *
    * @param points Point2d[]
    * @return boolean
    */
   public boolean areAllPointsInside(Point2d[] points)
   {
      checkIfUpToDate();
      for (Point2d point : points)
      {
         if (!isPointInside(point))
         {
            return false;
         }
      }

      return true;
   }

   /**
    * checks to see if the passed polygon is inside of this one.
    *
    * @param polygon2
    * @return true if polygon2 is inside of this polygon
    */
   public boolean isPolygonInside(ConvexPolygon2d polygon2)
   {
      checkIfUpToDate();
      for (int i = 0; i < polygon2.getNumberOfVertices(); i++)
      {
         if (!isPointInside(polygon2.getVertex(i)))
         {
            return false;
         }
      }

      return true;
   }

   public Line2d[] getLinesOfSight(Point2d observerPoint)
   {
      checkIfUpToDate();
      Point2d[] lineOfSightVertices = getLineOfSightVerticesCopy(observerPoint);

      if ((lineOfSightVertices == null) || (lineOfSightVertices.length < 1))
         return null;

      if (lineOfSightVertices.length == 1)
      {
         return new Line2d[] { new Line2d(observerPoint, lineOfSightVertices[0]) };
      }

      return new Line2d[] { new Line2d(observerPoint, lineOfSightVertices[0]), new Line2d(observerPoint, lineOfSightVertices[1]) };
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

      int[] indices = getLineOfSightVerticesIndices(observerPoint2d);

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

   public Point2d[] getLineOfSightVerticesCopy(Point2d observerPoint2d)
   {
      checkIfUpToDate();
      int[] indices = getLineOfSightVerticesIndices(observerPoint2d);

      if (indices == null)
         return null;

      // TODO indices.length == 1 is deprecated
      if (indices.length == 1 || indices[0] == indices[1])
      {
         return new Point2d[]{new Point2d(getVertex(indices[0]))};
      }

      int leavingLeftEdge = indices[0];
      int enteringRightEdge = indices[1];

      Point2d leftPoint = new Point2d(getVertex(leavingLeftEdge));
      Point2d rightPoint = new Point2d(getVertex(enteringRightEdge));

      return new Point2d[] { leftPoint, rightPoint };
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
      int[] indices = getLineOfSightVerticesIndices(observerPoint2d);

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

      return new LineSegment2d[] { leftLineSegment, rightLineSegment };
   }

   protected boolean getLineOfSightVerticesIndices(Point2d observerPoint2d, int[] verticesIndices)
   {
      checkIfUpToDate();
      if (hasExactlyOneVertex() && !getVertex(0).equals(observerPoint2d))
      {
         verticesIndices[0] = 0;
         verticesIndices[0] = 0;
         return true;
      }

      // At any time we'll hold onto 4 edge indices. -1 signifies not found yet:
      int leavingRightEdge = -1, leavingLeftEdge = -1, enteringLeftEdge = -1, enteringRightEdge = -1;

      // First choose an edge at random. An edge index will be signified by its first vertex in clockwise order.
      int firstEdgeIndex = numberOfVertices / 2;

      if (DEBUG) System.out.println("firstEdgeIndex = " + firstEdgeIndex);

      if (isEdgeEntering(firstEdgeIndex, observerPoint2d))
      {
         if (DEBUG) System.out.println("firstEdgeIndex is Entering");

         enteringLeftEdge = firstEdgeIndex;
         enteringRightEdge = firstEdgeIndex;
      }
      else
      {
         if (DEBUG) System.out.println("firstEdgeIndex is Leaving");

         leavingLeftEdge = firstEdgeIndex;
         leavingRightEdge = firstEdgeIndex;
      }

      // Now we need to search for the other two edges:
      boolean foundLeavingEdges = (leavingRightEdge >= 0) && (leavingLeftEdge >= 0);
      boolean foundEnteringEdges = (enteringRightEdge >= 0) && (enteringLeftEdge >= 0);

      if (DEBUG)
      {
         System.out.println("foundLeavingEdges is " + foundLeavingEdges);
         System.out.println("foundEnteringEdges is " + foundEnteringEdges);
      }

      while (!foundLeavingEdges)
      {
         int edgeToTest = getMidEdgeOppositeClockwiseOrdering(enteringRightEdge, enteringLeftEdge);

         if (DEBUG)
            System.out.println("edgeToTest is " + edgeToTest);

         if ((edgeToTest == enteringLeftEdge) || (edgeToTest == enteringRightEdge))
         {
            throw new RuntimeException("Couldn't find a leaving edge! This should never happen!!");
         }

         if (isEdgeEntering(edgeToTest, observerPoint2d))
         {
            if (DEBUG)
               System.out.println("edgeToTest is entering ");

            // Figure out if the edgeToTest should replace the leftEdge or the rightEdge:
            enteringLeftEdge = whichVertexIsToTheLeft(edgeToTest, enteringLeftEdge, observerPoint2d);
            enteringRightEdge = whichVertexIsToTheRight(edgeToTest, enteringRightEdge, observerPoint2d);
         }

         else
         {
            if (DEBUG)
               System.out.println("edgeToTest is leaving ");

            foundLeavingEdges = true;

            leavingRightEdge = edgeToTest;
            leavingLeftEdge = edgeToTest;
         }
      }

      while (!foundEnteringEdges)
      {
         int edgeToTest = getMidEdgeOppositeClockwiseOrdering(leavingLeftEdge, leavingRightEdge);

         if (DEBUG) System.out.println("edgeToTest is " + edgeToTest);

         if ((edgeToTest == leavingLeftEdge) || (edgeToTest == leavingRightEdge))
         {
            if (DEBUG)
               throw new RuntimeException("Couldn't find an entering edge! Must be inside!!");
            else
               return false;
         }

         if (isEdgeEntering(edgeToTest, observerPoint2d))
         {
            if (DEBUG) System.out.println("edgeToTest is entering ");

            foundEnteringEdges = true;

            enteringRightEdge = edgeToTest;
            enteringLeftEdge = edgeToTest;
         }
         else
         {
            if (DEBUG) System.out.println("edgeToTest is leaving ");

            // Figure out if the edgeToTest should replace the leftEdge or the rightEdge:
            int newLeavingLeftEdge = whichVertexIsToTheLeft(edgeToTest, leavingLeftEdge, observerPoint2d);
            int newLeavingRightEdge = whichVertexIsToTheRight(edgeToTest, leavingRightEdge, observerPoint2d);

            if ((newLeavingLeftEdge == leavingLeftEdge) && (newLeavingRightEdge == leavingRightEdge))
            {
               // Will loop forever if you don't do something about it!
               throw new RuntimeException("Looping forever!");
            }

            leavingLeftEdge = newLeavingLeftEdge;
            leavingRightEdge = newLeavingRightEdge;

            if (leavingLeftEdge == leavingRightEdge)
            {
               if (DEBUG)
                  throw new RuntimeException("Start Point must have been inside the polygon!!");
               else
                  return false;

            }

            if (DEBUG)
            {
               System.out.println("New leavingLeftEdge = " + leavingLeftEdge);
               System.out.println("New leavingRightEdge = " + leavingLeftEdge);
            }
         }
      }

      // Now binary search till their are no gaps:
      if (DEBUG) System.out.println("leavingLeftEdge = " + leavingLeftEdge + ", enteringLeftEdge = " + enteringLeftEdge + ". Binary search to reduce gaps.");

      while (getNextVertexIndex(enteringLeftEdge) != leavingLeftEdge)
      {
         int edgeToTest = getMidEdgeOppositeClockwiseOrdering(leavingLeftEdge, enteringLeftEdge);
         if (isEdgeEntering(edgeToTest, observerPoint2d))
         {
            enteringLeftEdge = edgeToTest;
         }
         else
         {
            leavingLeftEdge = edgeToTest;
         }

         if (DEBUG)
            System.out.println("leavingLeftEdge = " + leavingLeftEdge + ", enteringLeftEdge = " + enteringLeftEdge);

      }

      if (DEBUG)
         System.out.println("leavingRightEdge = " + leavingRightEdge + ", enteringRightEdge = " + enteringRightEdge + ". Binary search to reduce gaps");

      while (getNextVertexIndex(leavingRightEdge) != enteringRightEdge)
      {
         int edgeToTest = getMidEdgeOppositeClockwiseOrdering(enteringRightEdge, leavingRightEdge);
         if (isEdgeEntering(edgeToTest, observerPoint2d))
         {
            enteringRightEdge = edgeToTest;
         }
         else
         {
            leavingRightEdge = edgeToTest;
         }

         if (DEBUG)
            System.out.println("leavingRightEdge = " + leavingRightEdge + ", enteringRightEdge = " + enteringRightEdge);

      }

      // Now the edges are adjacent. Want the common nodes:
      if (DEBUG)
         System.out.println("leftPoint = " + leavingLeftEdge + ", rightPoint = " + enteringRightEdge);

      verticesIndices[0] = leavingLeftEdge;
      verticesIndices[1] = enteringRightEdge;
      return true;
   }

   protected int[] getLineOfSightVerticesIndices(Point2d observerPoint2d)
   {
      int[] verticesIndices = new int[2];
      boolean succeeded = getLineOfSightVerticesIndices(observerPoint2d, verticesIndices);

      if (!succeeded)
         return null;

      return verticesIndices;
   }

   public LineSegment2d[] getNearestEdges(Point2d testPoint)
   {
      checkIfUpToDate();
      if (!hasAtLeastTwoVertices())
      {
         return null;
      }

      int[] nearestEdgeIndices = getNearestEdgeIndices(testPoint);
      if (nearestEdgeIndices == null)
         return null;

      int numberOfEdges = nearestEdgeIndices.length;

      LineSegment2d[] ret = new LineSegment2d[numberOfEdges];

      for (int i = 0; i < numberOfEdges; i++)
      {
         int edgeIndex = nearestEdgeIndices[i];
         LineSegment2d edge = new LineSegment2d(getVertex(edgeIndex), getNextVertex(edgeIndex));

         ret[i] = edge;
      }

      return ret;
   }

   private int[] getNearestEdgeIndices(Point2d pointToProject)
   {
      // First find the line of sight vertices. If inside the Polygon, then return null.
      int[] indices = getLineOfSightVerticesIndices(pointToProject);
      if ((indices == null) || (indices.length < 2))
         return null;

      int leftEdge = (indices[0] - 1 + numberOfVertices) % numberOfVertices;
      int rightEdge = indices[1];

      // Binary search maintaining nearest left and nearest right vertices until they are adjacent: //TODO remove the q from maintaining
      while ((rightEdge != leftEdge) && (getNextVertexIndex(rightEdge) != leftEdge))
      {
         int testEdge = getMidEdgeOppositeClockwiseOrdering(leftEdge, rightEdge);

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
            return new int[] { testEdge };
         }
      }

      // If edges are the same, then just return one,
      if (leftEdge == rightEdge)
         return new int[] { leftEdge };

      // Otherwise check if left is fully to the left or right is fully to the right:
      boolean leftEdgeIsFullyLeft = isEdgeFullyToTheLeftOfObserver(leftEdge, pointToProject);
      boolean rightEdgeIsFullyRight = isEdgeFullyToTheRight(rightEdge, pointToProject);

      // They should never both be not fully left or right...
      if (!leftEdgeIsFullyLeft && !rightEdgeIsFullyRight)
      {
         throw new RuntimeException("Should never get here!");
      }

      if (!leftEdgeIsFullyLeft)
         return new int[] { leftEdge };
      if (!rightEdgeIsFullyRight)
         return new int[] { rightEdge };

      return new int[] { leftEdge, rightEdge };

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
      double edgeNormalX = -(nextVertex.y - vertex.y);
      double edgeNormalY = (nextVertex.x - vertex.x);

      double vertexToObserverX = observerPoint2d.x - vertex.x;
      double vertexToObserverY = observerPoint2d.y - vertex.y;

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
      double edgeNormalX = -(nextVertex.y - vertex.y);
      double edgeNormalY = (nextVertex.x - vertex.x);

      double nextVertexToObserverX = observerPoint2d.x - nextVertex.x;
      double nextVertexToObserverY = observerPoint2d.y - nextVertex.y;

      // Equivalent to looking at the sign of the angle formed by (egdeNormal -> vertexToPoint)
      double crossProduct = edgeNormalX * nextVertexToObserverY - edgeNormalY * nextVertexToObserverX;

      // If positive, that means vertexToPoint is pointing outside the edge
      // Thus, the observer point is totally on the left of the edge
      return crossProduct < 0.0;
   }

   private int[] getFirstCrossedEdgeFromInsideLineIndices(Point2d lineStart, Vector2d lineDirection)
   {
      if (!hasAtLeastOneVertex())
      {
         return null;
      }

      // At any time we'll hold onto 4 vertex indices. -1 signifies not found yet:
      int forwardLeftVertex = -1, behindLeftVertex = -1, forwardRightVertex = -1, behindRightVertex = -1;

      // First choose a vertex at random.

      int firstVertex = numberOfVertices / 2;

      if (isVertexToTheLeftOrStraightOn(firstVertex, lineStart, lineDirection))
      {
         forwardLeftVertex = firstVertex;
         behindLeftVertex = firstVertex;
      }

      else
      {
         forwardRightVertex = firstVertex;
         behindRightVertex = firstVertex;
      }

      // Now we need to search for the other two vertices:
      boolean foundLeftVertices = forwardLeftVertex >= 0;
      boolean foundRightVertices = forwardRightVertex >= 0;

      while (!foundLeftVertices)
      {
         int vertexToTest = getMidEdgeOppositeClockwiseOrdering(forwardRightVertex, behindRightVertex);

         if (isVertexToTheLeftOrStraightOn(vertexToTest, lineStart, lineDirection))
         {
            foundLeftVertices = true;

            forwardLeftVertex = vertexToTest;
            behindLeftVertex = vertexToTest;
         }

         else
         {
            forwardRightVertex = whichVertexIsToTheLeft(vertexToTest, forwardRightVertex, lineStart);
            behindRightVertex = whichVertexIsToTheRight(vertexToTest, behindRightVertex, lineStart);
         }
      }

      while (!foundRightVertices)
      {
         int vertexToTest = getMidEdgeOppositeClockwiseOrdering(behindLeftVertex, forwardLeftVertex);

         if (isVertexToTheRightOrStraightOn(vertexToTest, lineStart, lineDirection))
         {
            foundRightVertices = true;

            forwardRightVertex = vertexToTest;
            behindRightVertex = vertexToTest;
         }

         else
         {
            forwardLeftVertex = whichVertexIsToTheRight(vertexToTest, forwardLeftVertex, lineStart);
            behindLeftVertex = whichVertexIsToTheLeft(vertexToTest, behindLeftVertex, lineStart);
         }
      }

      // Now binary search till their are no gaps:

      while (getNextVertexIndex(forwardLeftVertex) != forwardRightVertex)
      {
         int vertexToTest = getMidEdgeOppositeClockwiseOrdering(forwardRightVertex, forwardLeftVertex);
         if (isVertexToTheLeftOrStraightOn(vertexToTest, lineStart, lineDirection))
         {
            forwardLeftVertex = vertexToTest;
         }
         else
         {
            forwardRightVertex = vertexToTest;
         }
      }

      return new int[] { forwardLeftVertex, forwardRightVertex };
   }

   private boolean isVertexToTheLeftOrStraightOn(int index, Point2d lineStart, Vector2d lineDirection)
   {
      Point2d point = getVertex(index);

      double vectorToVertexX = point.x - lineStart.x; // x_index - x_start
      double vectorToVertexY = point.y - lineStart.y; // y_index - y_start
      // 0 * (y_index - y_start) - 1 * (x_index - x_start) = x_start - x_index
      double crossProduct = lineDirection.x * vectorToVertexY - lineDirection.y * vectorToVertexX;

      // x_start >= x_index
      if (crossProduct >= 0.0)
      {
         return true;
      }
      else
      {
         return false;
      }
   }

   private boolean isVertexToTheRightOrStraightOn(int index, Point2d lineStart, Vector2d lineDirection)
   {
      Point2d point = getVertex(index);

      double vectorToVertexX = point.x - lineStart.x;
      double vectorToVertexY = point.y - lineStart.y;

      double crossProduct = lineDirection.x * vectorToVertexY - lineDirection.y * vectorToVertexX;

      if (crossProduct <= 0.0)
      {
         return true;
      }
      else
      {
         return false;
      }
   }

   private int whichVertexIsToTheLeft(int index1, int index2, Point2d observerFramePoint2d)
   {
      int ret = whichVertexIsToTheLeftRight(index1, index2, observerFramePoint2d, true);

      //    System.out.println("Checking Which edge is to the left. index1 = " + index1 + ", index2 = " + index2 + ", observerFramePoint2d = " + observerFramePoint2d + ", answer = " + ret);

      return ret;
   }

   private int whichVertexIsToTheRight(int index1, int index2, Point2d observerFramePoint2d)
   {
      int ret = whichVertexIsToTheLeftRight(index1, index2, observerFramePoint2d, false);

      //    System.out.println("Checking Which edge is to the right. index1 = " + index1 + ", index2 = " + index2 + ", observerFramePoint2d = " + observerFramePoint2d + ", answer = " + ret);

      return ret;
   }

   private int whichVertexIsToTheLeftRight(int index1, int index2, Point2d observerFramePoint2d, boolean returnTheLeftOne)
   {
      Point2d point1 = getVertex(index1);
      Point2d point2 = getVertex(index2);

      double vectorToVertex1X = point1.x - observerFramePoint2d.x;
      double vectorToVertex1Y = point1.y - observerFramePoint2d.y;

      double vectorToVertex2X = point2.x - observerFramePoint2d.x;
      double vectorToVertex2Y = point2.y - observerFramePoint2d.y;

      double crossProduct = vectorToVertex1X * vectorToVertex2Y - vectorToVertex1Y * vectorToVertex2X;

      if (crossProduct < 0.0)
      {
         if (returnTheLeftOne)
            return index1;
         else
            return index2;
      }
      else
      {
         if (returnTheLeftOne)
            return index2;
         else
            return index1;
      }
   }

   /**
    * Check if from the observer point you're looking at the "outside-of-polygon side of the edge"
    * @param edgeIndex edge you look at.
    * @param observerPoint2d point from where you look at the edge.
    * @return true = "outside-of-polygon side of the edge".
    */
   private boolean isEdgeEntering(int edgeIndex, Point2d observerPoint2d)
   {
      Point2d vertex = getVertex(edgeIndex);
      Point2d nextVertex = getNextVertex(edgeIndex);

      // Vector perpendicular to the edge and pointing outside the polygon
      double edgeNormalX = -(nextVertex.y - vertex.y);
      double edgeVectorY = nextVertex.x - vertex.x;

      double observerToVertexX = vertex.x - observerPoint2d.x;
      double observerToVertexY = vertex.y - observerPoint2d.y;

      // Equivalent to looking at the projection of the observerToVertex vector on the edge normal vector, since only need to look at the sign.
      double dotProduct = edgeNormalX * observerToVertexX + edgeVectorY * observerToVertexY;

      return dotProduct < 0.0;
   }

   @Override
   public Point2d[] intersectionWith(LineSegment2d lineSegment2d)
   {
      checkIfUpToDate();
      if (hasExactlyOneVertex())
      {
         if (lineSegment2d.isPointOnLineSegment(getVertex(0)))
            return new Point2d[] { new Point2d(getVertex(0)) };
         else
            return null;
      }

      if (hasExactlyTwoVertices())
      {
         LineSegment2d polygonAsSegment = new LineSegment2d(getVertex(0), getVertex(1));
         Point2d intersection = polygonAsSegment.intersectionWith(lineSegment2d);
         if (intersection != null)
            return new Point2d[] { intersection };
         else
            return null;
      }

      Point2d[] endPoints = lineSegment2d.endpoints;

      Point2d outsidePoint, otherPoint;
      boolean onePointIsInside;

      // First make sure that the ray goes from an outside point...
      if (isPointInside(endPoints[0]))
      {
         if (isPointInside(endPoints[1]))
            return null; // Both Points are inside!

         outsidePoint = endPoints[1];
         otherPoint = endPoints[0];
         onePointIsInside = true;
      }
      else if (isPointInside(endPoints[1]))
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

      Point2d[] intersections = intersectionWithRay(line2d);
      if (intersections == null)
         return null;

      if (!onePointIsInside)
         return intersections;

      return new Point2d[] { intersections[0] }; // Only return the entering point, if the segment has one point inside the polygon.
   }

   //TODO: Clean this up and add a lot of test cases...
   // ALesman: suggestion, move the origin out to edge of the capacity of double
   // then project a single ray in from there
   // seems to produce same point twice when intersecting a line

   @Override
   public Point2d[] intersectionWith(Line2d line)
   {
      checkIfUpToDate();
      Point2d[] intersectionWithRay1 = intersectionWithRay(line);

      if ((intersectionWithRay1 == null) || (intersectionWithRay1.length == 0))
      {
         Line2d oppositeDirection = new Line2d(line);
         oppositeDirection.negateDirection();

         return intersectionWithRay(oppositeDirection);
      }

      if (intersectionWithRay1.length == 2)
      {
         return intersectionWithRay1;
      }

      else if (intersectionWithRay1.length == 1) // Must have been inside or on the polygon. See what we get with the ray the other direction from the outside
      {
         Point2d newPoint = new Point2d();
         line.getPoint(newPoint);
         Vector2d vectorHeadingTheOtherWay = new Vector2d();
         line.getNormalizedVector(vectorHeadingTheOtherWay);

         newPoint.add(vectorHeadingTheOtherWay);
         vectorHeadingTheOtherWay.negate();

         Line2d backwardLineWithADifferentStartingPoint = new Line2d(newPoint, vectorHeadingTheOtherWay);

         Point2d[] newIntersections = intersectionWithRay(backwardLineWithADifferentStartingPoint);
         if ((newIntersections == null) || (!(newIntersections.length >= 1)))
         {
            return intersectionWithRay1; // Kindof a bug. Should fix this later, and then throw the exception as follows...

            //          throw new RuntimeException("Bug. Should find at least one intersection if the ray the other way found one!");
         }

         if (newIntersections.length == 2)
            return newIntersections;
         else
         {
            return new Point2d[] { intersectionWithRay1[0], newIntersections[0] }; // definitely causes line intersections to be incorrect
         }
      }
      else
      {
         throw new RuntimeException("Should never get here");
      }

   }

   public Point2d[] intersectionWithRay(Line2d ray)
   {
      checkIfUpToDate();
      if (ray == null || ray.containsNaN())
         return null;

      LineSegment2d[] intersectingEdges = getIntersectingEdges(ray);
      if (intersectingEdges == null)
         return null;

      Point2d[] ret = new Point2d[intersectingEdges.length];

      for (int i = 0; i < intersectingEdges.length; i++)
      {
         ret[i] = intersectingEdges[i].intersectionWith(ray);
         if (ret[i] == null)
            return new Point2d[] { this.getClosestVertexCopy(ray) };

         //       if (ret[i] == null) throw new RuntimeException("Shouldn't be null! line2d = " + line2d + ", intersectingEdges[" + i + "] = " + intersectingEdges[i]);
      }

      if (hasExactlyTwoVertices() && ret.length == 2 && ret[0].equals(ret[1]))
         ret = new Point2d[] { ret[0] };

      return ret;
   }

   public LineSegment2d[] getIntersectingEdges(Line2d line2d)
   {
      checkIfUpToDate();
      if (!hasAtLeastTwoVertices())
      {
         return null;
      }

      else if (!hasAtLeastThreeVertices())
      {
         LineSegment2d polygonAsSegment = new LineSegment2d(getVertex(0), getVertex(1));
         LineSegment2d polygonAsSegmentReversed = new LineSegment2d(getVertex(1), getVertex(0));
         if (polygonAsSegment.intersectionWith(line2d) != null)
         {
            return new LineSegment2d[] { polygonAsSegment, polygonAsSegmentReversed };
         }
         else
         {
            return null;
         }
      }

      // First find the indices of the two line of sight vertices:
      Point2d lineStart = new Point2d();
      line2d.getPoint(lineStart);
      Vector2d lineDirection = new Vector2d();
      line2d.getNormalizedVector(lineDirection);

      int[] lineOfSightVertices = getLineOfSightVerticesIndices(lineStart);
      int leftLineOfSightVertex, rightLineOfSightVertex;

      if (lineOfSightVertices == null) // Means the line starts inside the Polygon! Only worry about the leaving vertices and not the entering vertices!
      {
         int[] leavingEdgeVertices = getFirstCrossedEdgeFromInsideLineIndices(lineStart, lineDirection);
         int firstLeavingVertex = leavingEdgeVertices[0];
         int secondLeavingVertex = leavingEdgeVertices[1];

         if (getNextVertexIndex(firstLeavingVertex) != secondLeavingVertex)
         {
            throw new RuntimeException("!areAdjacentInClockwiseOrder");
         }

         LineSegment2d leavingEdge = new LineSegment2d(getVertex(firstLeavingVertex), getVertex(secondLeavingVertex));

         return new LineSegment2d[] { leavingEdge };

      }

      leftLineOfSightVertex = lineOfSightVertices[0];
      rightLineOfSightVertex = lineOfSightVertices[1];

      // Check if line is between the vertices or not:
      if (!isLineStrictlyBetweenVertices(lineStart, lineDirection, leftLineOfSightVertex, rightLineOfSightVertex))
         return null;

      // Now binary search between them to find the entering edge.
      int leftEnteringVertex = leftLineOfSightVertex;
      int rightEnteringVertex = rightLineOfSightVertex;

      while (getNextVertexIndex(rightEnteringVertex) != leftEnteringVertex)
      {
         int testVertex = getMidEdgeOppositeClockwiseOrdering(leftEnteringVertex, rightEnteringVertex);

         if (isLineStrictlyBetweenVertices(lineStart, lineDirection, testVertex, rightEnteringVertex))
         {
            leftEnteringVertex = testVertex;
         }
         else if (isLineStrictlyBetweenVertices(lineStart, lineDirection, leftEnteringVertex, testVertex))
         {
            rightEnteringVertex = testVertex;
         }
         else
         {
            // Hit vertex right on!
            rightEnteringVertex = testVertex;
            leftEnteringVertex = (testVertex + 1) % numberOfVertices;

            // throw new RuntimeException("Should never get here!!");

         }
      }

      // Now binary search between them to find the leaving edge.
      int leftLeavingVertex = leftLineOfSightVertex;
      int rightLeavingVertex = rightLineOfSightVertex;

      while (getNextVertexIndex(leftLeavingVertex) != rightLeavingVertex)
      {
         int testVertex = getMidEdgeOppositeClockwiseOrdering(rightLeavingVertex, leftLeavingVertex);

         if (isLineStrictlyBetweenVertices(lineStart, lineDirection, testVertex, rightLeavingVertex))
         {
            leftLeavingVertex = testVertex;
         }
         else if (isLineStrictlyBetweenVertices(lineStart, lineDirection, leftLeavingVertex, testVertex))
         {
            rightLeavingVertex = testVertex;
         }
         else
         {
            // Hit vertex right on!
            leftLeavingVertex = testVertex;
            rightLeavingVertex = (testVertex + 1) % numberOfVertices;

            //          throw new RuntimeException("Should never get here!!");
         }
      }

      // Now we have adjacent vertices. Return the first ones to signify the edges:

      LineSegment2d enteringEdge = new LineSegment2d(getVertex(rightEnteringVertex), getVertex(leftEnteringVertex));
      LineSegment2d leavingEdge = new LineSegment2d(getVertex(leftLeavingVertex), getVertex(rightLeavingVertex));

      return new LineSegment2d[] { enteringEdge, leavingEdge };

   }

   private boolean isLineStrictlyBetweenVertices(Point2d lineStart, Vector2d lineDirection, int leftIndex, int rightIndex)
   {
      Point2d leftVertex = getVertex(leftIndex);
      Point2d rightVertex = getVertex(rightIndex);

      double startToLeftVertexX = leftVertex.x - lineStart.x;
      double startToLeftVertexY = leftVertex.y - lineStart.y;

      double startToRightVertexX = rightVertex.x - lineStart.x;
      double startToRightVertexY = rightVertex.y - lineStart.y;

      double leftCrossProduct = lineDirection.x * startToLeftVertexY - lineDirection.y * startToLeftVertexX;
      double rightCrossProduct = lineDirection.x * startToRightVertexY - lineDirection.y * startToRightVertexX;

      if ((leftCrossProduct > 0.0) && (rightCrossProduct < 0.0))
         return true;

      return false;
   }

   @Override
   public ConvexPolygon2d intersectionWith(ConvexPolygon2d convexPolygon)
   {
      checkIfUpToDate();
      ConvexPolygon2d ret = new ConvexPolygon2d();
      boolean success = ConvexPolygonTools.computeIntersectionOfPolygons(this, convexPolygon, ret);
      if (!success) ret = null;
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
         ret = ret + "{" + vertex.x + ", " + vertex.y + "}," + "\n";
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
         tempVertex3d.set(vertex.x, vertex.y, 0.0);
         transform.transform(tempVertex3d);
         vertex.x = tempVertex3d.x;
         vertex.y = tempVertex3d.y;
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
      double x = centroid.x + (point.x - centroid.x) * (1.0 - percent);
      double y = centroid.y + (point.y - centroid.y) * (1.0 - percent);

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
      transform.get(tempRotation);

      return ReferenceFrame.isRotationInPlane(tempRotation);
   }

   protected void getPointsInClockwiseOrder(int startIndexInclusive, int endIndexInclusive, ArrayList<Point2d> pointList)
   {
      checkIfUpToDate();
      int index = startIndexInclusive;

      while(true)
      {
         pointList.add(getVertex(index));

         if (index == endIndexInclusive) break;
         index = getNextVertexIndex(index);
      }
   }

   protected void addVerticesInClockwiseOrderInPolygon(int startIndexInclusive, int endIndexInclusive, ConvexPolygon2d polygonToPack)
   {
      checkIfUpToDate();
      int index = startIndexInclusive;

      while(true)
      {
         polygonToPack.addVertex(getVertex(index));

         if (index == endIndexInclusive) break;
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

   public boolean isCompletelyInside(ConvexPolygon2d polygonQ)
   {
      checkIfUpToDate();
      for (int i = 0; i < numberOfVertices; i++)
      {
         if (!polygonQ.isPointInside(getVertex(i)))
            return false;
      }

      return true;
   }

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
      Vector2d[] outsideFacingOrthogonalVectors = new Vector2d[numberOfVertices];
      
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
      tempPoint2dForDistance.set(point);
      orthogonalProjection(tempPoint2dForDistance);
      
      return point.distance(tempPoint2dForDistance);
   }

   @Override
   public Point2d orthogonalProjectionCopy(Point2d point)
   {
      checkIfUpToDate();
      Point2d copy = new Point2d(point);
      orthogonalProjection(copy);

      return copy;
   }

   
   private final Point2d tempPoint2d = new Point2d();

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

      tempPoint2d.set(point2d);
      int[] nearestEdges = getNearestEdgeIndices(tempPoint2d);
      if (nearestEdges == null)
         return; // point2d must be inside polygon, so leave it as it is.

      int leftEdge, rightEdge;

      if (nearestEdges.length == 1)
      {
         leftEdge = rightEdge = nearestEdges[0];
      }

      else
      {
         leftEdge = nearestEdges[0];
         rightEdge = nearestEdges[1];
      }

      // Two adjacent edges. Return the left vertex:
      if (leftEdge != rightEdge)
         point2d.set(getVertex(leftEdge));

      // Just one edge. Find the point on the edge:

      Point2d firstEdgeVertex = getVertex(leftEdge);
      Point2d secondEdgeVertex = getNextVertex(leftEdge);

      Vector2d firstVertexToPoint = new Vector2d(point2d);
      firstVertexToPoint.sub(firstEdgeVertex);

      Vector2d edgeVector = new Vector2d(secondEdgeVertex);
      edgeVector.sub(firstEdgeVertex);

      if (edgeVector.lengthSquared() < 1e-10)
      {
         point2d.set(firstEdgeVertex);

         return;
      }

      double dotProduct = edgeVector.dot(firstVertexToPoint);
      double lengthSquared = edgeVector.lengthSquared();
      double alpha = dotProduct / lengthSquared;

      // Need to keep alpha between 0.0 and 1.0 since if only one edge is seen, the projection can be outside the edge.
      if (alpha < 0.0)
         alpha = 0.0;
      if (alpha > 1.0)
         alpha = 1.0;

      edgeVector.scale(alpha);

      Point2d ret = new Point2d(firstEdgeVertex);
      ret.add(edgeVector);

      // Make sure the returned point is inside the polygon by nudging it a little toward the centroid.
      // This will all but guarantee that projections are then inside.

      Vector2d retToCentroid = new Vector2d(centroid);
      retToCentroid.sub(ret);
      retToCentroid.scale(1.0e-12);

      ret.add(retToCentroid);

      point2d.set(ret);
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

   protected int getMidEdgeOppositeClockwiseOrdering(int leftEdgeIndex, int rightEdgeIndex)
   {
      checkIfUpToDate();
      if (rightEdgeIndex >= leftEdgeIndex)
         return (rightEdgeIndex + (leftEdgeIndex + numberOfVertices - rightEdgeIndex + 1) / 2) % numberOfVertices;
      else
         return (rightEdgeIndex + leftEdgeIndex + 1) / 2;
   }
}
