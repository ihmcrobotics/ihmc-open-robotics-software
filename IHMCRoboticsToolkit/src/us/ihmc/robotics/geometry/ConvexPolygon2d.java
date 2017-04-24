package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonTools.EmptyPolygonException;
import us.ihmc.robotics.geometry.ConvexPolygonTools.OutdatedPolygonException;
import us.ihmc.robotics.random.RandomGeometry;

/**
 * <p>
 * Title: ConvexPolygon2d
 * </p>
 *
 * <p>
 * Description: Describes a planar convex polygon. The vertices of this polygon are clockwise and
 * are all different.
 * </p>
 *
 * @author IHMC Biped Team
 * @version 1.0
 */
public class ConvexPolygon2d implements GeometryObject<ConvexPolygon2d>
{
   private final boolean clockwiseOrdered = true;
   final ArrayList<Point2D> clockwiseOrderedListOfPoints = new ArrayList<Point2D>();
   private final BoundingBox2D boundingBox = new BoundingBox2D();
   private final Point2D centroid = new Point2D();
   private int numberOfVertices = 0;
   private boolean isUpToDate = false;
   private double area;

   private int minX_index = 0, maxX_index = 0, minY_index = 0, maxY_index = 0;
   private int minXminY_index = 0, maxXminY_index = 0, maxXmaxY_index = 0;
   private final int minXmaxY_index = 0;

   /**
    * Creates an empty convex polygon.
    */
   public ConvexPolygon2d()
   {
      numberOfVertices = 0;
      update();
   }

   /**
    * Creates an empty convex polygon, adds N new vertices using a list of {@code Point2d}, updates
    * the vertices so they are clockwise ordered, and initializes some essential numbers such as the
    * centroid.
    * 
    * @param vertices {@code List<Point2d>} the list of points that is used to creates the vertices.
    * @param numberOfVertices int that is used to determine the number of vertices of the polygon.
    *           Note the: {@code pointList.size()} can be greater or equal to numberOfVertices.
    */
   public ConvexPolygon2d(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      setAndUpdate(vertices, numberOfVertices);
   }

   /**
    * Creates an empty convex polygon, adds N new vertices using a list of {@code Point2d}, updates
    * the vertices so they are clockwise ordered, and initializes some essential numbers such as the
    * centroid.
    * 
    * @param vertices {@code List<Point2d>} the list of points that is used to creates the vertices.
    *           The number of vertices of this polygon will be equal to the size of the point list.
    */
   public ConvexPolygon2d(List<? extends Point2DReadOnly> vertices)
   {
      this(vertices, vertices.size());
   }

   /**
    * Creates an empty convex polygon, adds N new vertices using an array of {@code Point2d},
    * updates the vertices so they are clockwise ordered, and initializes some essential numbers
    * such as the centroid.
    * 
    * @param vertices {@code Point2d[]} the array of points that is used to creates the vertices.
    * @param numberOfVertices int that is used to determine the number of vertices of the polygon.
    *           Note the: {@code vertices.length} can be greater or equal to numberOfVertices.
    */
   public ConvexPolygon2d(Point2DReadOnly[] vertices, int numberOfVertices)
   {
      setAndUpdate(vertices, numberOfVertices);
   }

   /**
    * Creates an empty convex polygon, adds N new vertices using an array of {@code Point2d},
    * updates the vertices so they are clockwise ordered, and initializes some essential numbers
    * such as the centroid.
    * 
    * @param vertices {@code Point2d[]} the array of points that is used to creates the vertices.
    *           The number of vertices of this polygon will be equal to the length of the point
    *           array.
    */
   public ConvexPolygon2d(Point2DReadOnly[] vertices)
   {
      this(vertices, vertices.length);
   }

   /**
    * Creates an empty convex polygon, adds N new vertices using an array of {@code Point2d},
    * updates the vertices so they are clockwise ordered, and initializes some essential numbers
    * such as the centroid.
    * 
    * @param vertices {@code double[>=numberOfVertices][>=2]} the array of points that is used to
    *           creates the vertices. The each row contains one point whereas the (at least) two
    *           columns contains the coordinates x and y.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the
    *           polygon. Note the: {@code pointListArray.length} can be greater or equal to
    *           numberOfVertices.
    */
   public ConvexPolygon2d(double[][] vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * Creates an empty convex polygon, adds N new vertices using an array of {@code Point2d},
    * updates the vertices so they are clockwise ordered, and initializes some essential numbers
    * such as the centroid.
    * 
    * @param vertices {@code double[N][>=2]} the array of points that is used to creates the
    *           vertices. Each row contains one point whereas the (at least) two columns contains
    *           the coordinates x and y. The number of vertices of this polygon will be equal to the
    *           length of the point array.
    */
   public ConvexPolygon2d(double[][] vertices)
   {
      this(vertices, vertices.length);
   }

   /**
    * Creates a polygon with the same properties as the other polygon.
    * 
    * @param otherPolygon {@code ConvexPolygon2d} the other convex polygon.
    */
   public ConvexPolygon2d(ConvexPolygon2d otherPolygon)
   {
      setAndUpdate(otherPolygon);
   }

   /**
    * Creates a new convex polygon by combining two other convex polygons. The result is the
    * smallest convex hull that contains both polygons. Then it updates the vertices so they are
    * clockwise ordered, and initializes some essential numbers such as the centroid.
    * <p/>
    * TODO: Make this more efficient by finding the rotating calipers, as in the intersection
    * method.
    * <p/>
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
      ArrayList<Point2D> vertices = new ArrayList<Point2D>();

      for (int i = 0; i < numberOfPossiblePoints; i++)
      {
         vertices.add(RandomGeometry.nextPoint2D(random, maxAbsoluteXY, maxAbsoluteXY));
      }

      ConvexPolygon2d polygonToReturn = new ConvexPolygon2d(vertices);

      return polygonToReturn;
   }

   /**
    * After calling this method, the polygon has no vertex, area, or centroid. Note that calling
    * that method doesn't generate garbage.
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
    * Use only when an empty is desired. Note that calling that method doesn't generate garbage.
    */
   public void clearAndUpdate()
   {
      clear();
      isUpToDate = true;
   }

   /**
    * Add a vertex to this polygon. Note that this method recycles memory.
    * 
    * @param vertex {@code Point2d} the new vertex.
    */
   public void addVertex(Point2DReadOnly vertex)
   {
      isUpToDate = false;
      setOrCreate(vertex, numberOfVertices);
      numberOfVertices++;
   }

   /**
    * Add a vertex to this polygon. Note that this method recycles memory.
    * 
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
    * Adds N new vertices to this polygon using a list of {@code Point2d}. Note that this method
    * recycles memory.
    * 
    * @param vertices {@code List<Point2d>} the list of new vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices to add to
    *           this polygon. Note the: {@code vertices.size()} can be greater or equal to
    *           numberOfVertices.
    */
   public void addVertices(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      isUpToDate = false;
      for (int i = 0; i < numberOfVertices; i++)
         addVertex(vertices.get(i));
   }

   /**
    * Adds N new vertices to this polygon using an array of {@code Point2d}. Note that this method
    * recycles memory.
    * 
    * @param vertices {@code Point2d[]} the list of new vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices to add to
    *           this polygon. Note the: {@code vertices.length} can be greater or equal to
    *           numberOfVertices.
    */
   public void addVertices(Point2DReadOnly[] vertices, int numberOfVertices)
   {
      isUpToDate = false;
      for (int i = 0; i < numberOfVertices; i++)
         addVertex(vertices[i]);
   }

   /**
    * Adds N new vertices to this polygon using an array of {@code Point2d}. Note that this method
    * recycles memory.
    * 
    * @param vertices {@code double[>=numberOfVertices][>=2]} the array of new vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices to add to
    *           this polygon. Note the: {@code vertices.length} can be greater or equal to
    *           numberOfVertices.
    */
   public void addVertices(double[][] vertices, int numberOfVertices)
   {
      isUpToDate = false;
      for (int i = 0; i < numberOfVertices; i++)
         addVertex(vertices[i][0], vertices[i][1]);
   }

   /**
    * Adds new vertices to this polygon from another convex polygon. Note that this method recycles
    * memory.
    * 
    * @param otherPolygon {@code ConvexPolygon2d} the other convex polygon that is used to add new
    *           vertices to this polygon.
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

   private void setOrCreate(Point2DReadOnly point2d, int i)
   {
      setOrCreate(point2d.getX(), point2d.getY(), i);
   }

   private void setOrCreate(double x, double y, int i)
   {
      while (i >= clockwiseOrderedListOfPoints.size())
         clockwiseOrderedListOfPoints.add(new Point2D());
      clockwiseOrderedListOfPoints.get(i).set(x, y);
   }

   /**
    * Modifies the vertices so they are clockwise ordered. Updates centroid, area and the bounding
    * box. Call this method once new vertices have been added.
    */
   public void update()
   {
      if (isUpToDate)
         return;

      numberOfVertices = EuclidGeometryPolygonTools.inPlaceGiftWrapConvexHull2D(clockwiseOrderedListOfPoints, numberOfVertices);
      isUpToDate = true;

      updateCentroidAndArea();
      updateBoundingBox();
   }

   /**
    * This method does: 1- {@code clear()}; 2- {@code addVertices(vertices, numberOfVertices)}; 3-
    * {@code update()}.
    * 
    * @param vertices {@code List<Point2d>} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the
    *           polygon. Note the: {@code vertices.size()} can be greater or equal to
    *           numberOfVertices.
    */
   public void setAndUpdate(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does: 1- {@code clear()}; 2- {@code addVertices(vertices, numberOfVertices)}; 3-
    * {@code update()}.
    * 
    * @param vertices {@code Point2d[]} the list of points that is used to creates the vertices.
    * @param numberOfVertices {@code int} that is used to determine the number of vertices of the
    *           polygon. Note the: {@code vertices.length} can be greater or equal to
    *           numberOfVertices.
    */
   public void setAndUpdate(Point2DReadOnly[] vertices, int numberOfVertices)
   {
      clear();
      addVertices(vertices, numberOfVertices);
      update();
   }

   /**
    * This method does: 1- {@code clear()}; 2- {@code addVertices(vertices, numberOfVertices)}; 3-
    * {@code update()}.
    * 
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
    * This method does: 1- clear(); 2- addVertices(otherPolygon); 3- update().
    * <p/>
    * TODO There is no need to call update() there, instead update everything from the other polygon
    * to make it faster.
    * <p/>
    * 
    * @param otherPolygon {@code ConvexPolygon2d}
    */
   public void setAndUpdate(ConvexPolygon2d otherPolygon)
   {
      clear();
      addVertices(otherPolygon);
      update();
   }

   /**
    * This method does: 1- {@code clear()}; 2- {@code addVertices(firstPolygon)}; 2-
    * {@code addVertices(secondPolygon)}; 3- {@code update()}.
    * <p/>
    * TODO: Make this more efficient by finding the rotating calipers, as in the intersection
    * method.
    * <p/>
    * 
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

   void updateBoundingBox()
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

   // Compute centroid and area of this polygon. Formula taken from http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/
   void updateCentroidAndArea()
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

   public double getArea()
   {
      checkIfUpToDate();
      return area;
   }

   public void getCentroid(Point2DBasics centroidToPack)
   {
      checkIfUpToDate();
      centroidToPack.set(this.centroid);
   }

   public Point2DReadOnly getCentroid()
   {
      checkIfUpToDate();
      return centroid;
   }

   public BoundingBox2D getBoundingBox()
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

   public BoundingBox2D getBoundingBoxCopy()
   {
      checkIfUpToDate();
      BoundingBox2D ret = new BoundingBox2D(boundingBox);

      return ret;
   }

   public void getBoundingBox(BoundingBox2D boundingBoxToPack)
   {
      boundingBoxToPack.set(boundingBox);
   }

   /** Return the vertex from a clockwise ordered list */
   public Point2DReadOnly getVertex(int vertexIndex)
   {
      checkIfUpToDate();
      return getVertexUnsafe(vertexIndex);
   }

   /**
    * Same as getVertex(vertexIndex) but without checking if the polygon has been updated. Be
    * careful when using it!
    */
   protected Point2D getVertexUnsafe(int vertexIndex)
   {
      checkNonEmpty();
      checkIndexInBoundaries(vertexIndex);
      return clockwiseOrderedListOfPoints.get(vertexIndex);
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
      return clockwiseOrderedListOfPoints.get(numberOfVertices - 1 - vertexIndex);
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

   @Override
   public String toString()
   {
      String ret = "";
      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2D vertex = clockwiseOrderedListOfPoints.get(i);
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
      if (clockwiseOrderedListOfPoints.isEmpty())
         clockwiseOrderedListOfPoints.add(new Point2D());

      Point2D point = clockwiseOrderedListOfPoints.get(0);
      point.set(0.0, 0.0);

      isUpToDate = false;
      update();
   }

   @Override
   public void setToNaN()
   {
      numberOfVertices = 1;
      if (clockwiseOrderedListOfPoints.isEmpty())
         clockwiseOrderedListOfPoints.add(new Point2D());

      Point2D point = clockwiseOrderedListOfPoints.get(0);
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
         Point2D point = clockwiseOrderedListOfPoints.get(i);

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
      return EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(x, y, clockwiseOrderedListOfPoints, numberOfVertices, clockwiseOrdered, epsilon);
   }

   public boolean isPointInside(Point2DReadOnly point)
   {
      return isPointInside(point, 0.0);
   }

   public boolean isPointInside(Point2DReadOnly point, double epsilon)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.isPoint2DInsideConvexPolygon2D(point, clockwiseOrderedListOfPoints, numberOfVertices, clockwiseOrdered, epsilon);
   }

   public ConvexPolygon2d translateCopy(Tuple2DReadOnly translation)
   {
      return ConvexPolygon2dCalculator.translatePolygonCopy(translation, this);
   }

   public LineSegment2d[] getIntersectingEdgesCopy(Line2d line)
   {
      return ConvexPolygon2dCalculator.getIntersectingEdgesCopy(line, this);
   }

   public boolean getClosestPointWithRay(Point2DBasics pointToPack, Line2d ray)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestPointToNonInterectingRay2D(ray.point, ray.normalizedVector, clockwiseOrderedListOfPoints, numberOfVertices,
                                                                          clockwiseOrdered, pointToPack);
   }

   public Point2D getClosestPointWithRay(Line2d ray)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestPointToNonInterectingRay2D(ray.point, ray.normalizedVector, clockwiseOrderedListOfPoints, numberOfVertices,
                                                                          clockwiseOrdered);
   }

   public double distance(Point2DReadOnly point)
   {
      return Math.max(0.0, signedDistance(point));
   }

   public double signedDistance(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.signedDistanceFromPoint2DToConvexPolygon2D(point, clockwiseOrderedListOfPoints, numberOfVertices, clockwiseOrdered);
   }

   public void orthogonalProjection(Point2DBasics point2d)
   {
      orthogonalProjection(point2d, point2d);
   }

   public void orthogonalProjection(Point2DReadOnly point2d, Point2DBasics projectionToPack)
   {
      checkIfUpToDate();
      EuclidGeometryPolygonTools.orthogonalProjectionOnConvexPolygon2D(point2d, clockwiseOrderedListOfPoints, numberOfVertices, clockwiseOrdered,
                                                                       projectionToPack);
   }

   public Point2D orthogonalProjectionCopy(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.orthogonalProjectionOnConvexPolygon2D(point, clockwiseOrderedListOfPoints, numberOfVertices, clockwiseOrdered);
   }

   public int lineOfSightStartIndex(Point2DReadOnly observer)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.lineOfSightStartIndex(observer, clockwiseOrderedListOfPoints, numberOfVertices, clockwiseOrdered);
   }

   public int lineOfSightEndIndex(Point2DReadOnly observer)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.lineOfSightEndIndex(observer, clockwiseOrderedListOfPoints, numberOfVertices, clockwiseOrdered);
   }

   public int[] lineOfSightIndices(Point2DReadOnly observer)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.lineOfSightIndices(observer, clockwiseOrderedListOfPoints, numberOfVertices, clockwiseOrdered);
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
      return EuclidGeometryPolygonTools.canObserverSeeEdge(edgeIndex, observer, clockwiseOrderedListOfPoints, numberOfVertices, clockwiseOrdered);
   }

   public boolean pointIsOnPerimeter(Point2DReadOnly point)
   {
      return distance(point) < 1.0E-10;
   }

   public int intersectionWith(Line2d line, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLine2DAndConvexPolygon2D(line.point, line.normalizedVector, clockwiseOrderedListOfPoints,
                                                                                    numberOfVertices, clockwiseOrdered, firstIntersectionToPack,
                                                                                    secondIntersectionToPack);
   }

   public Point2D[] intersectionWith(Line2d line)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLine2DAndConvexPolygon2D(line.point, line.normalizedVector, clockwiseOrderedListOfPoints,
                                                                                    numberOfVertices, clockwiseOrdered);
   }

   public int intersectionWithRay(Line2d ray, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenRay2DAndConvexPolygon2D(ray.point, ray.normalizedVector, clockwiseOrderedListOfPoints,
                                                                                   numberOfVertices, clockwiseOrdered, firstIntersectionToPack,
                                                                                   secondIntersectionToPack);
   }

   public Point2D[] intersectionWithRay(Line2d ray)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenRay2DAndConvexPolygon2D(ray.point, ray.normalizedVector, clockwiseOrderedListOfPoints,
                                                                                   numberOfVertices, clockwiseOrdered);
   }

   public int intersectionWith(LineSegment2d lineSegment2d, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLineSegment2DAndConvexPolygon2D(lineSegment2d.getFirstEndpoint(), lineSegment2d.getSecondEndpoint(),
                                                                                           clockwiseOrderedListOfPoints, numberOfVertices, clockwiseOrdered,
                                                                                           firstIntersectionToPack, secondIntersectionToPack);
   }

   public Point2D[] intersectionWith(LineSegment2d lineSegment2d)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.intersectionBetweenLineSegment2DAndConvexPolygon2D(lineSegment2d.getFirstEndpoint(), lineSegment2d.getSecondEndpoint(),
                                                                                           clockwiseOrderedListOfPoints, numberOfVertices, clockwiseOrdered);
   }

   public int getClosestEdgeIndex(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestEdgeIndexToPoint2D(point, clockwiseOrderedListOfPoints, numberOfVertices, clockwiseOrdered);
   }

   public boolean getClosestEdge(LineSegment2d closestEdgeToPack, Point2DReadOnly point)
   {
      int edgeIndex = getClosestEdgeIndex(point);
      if (edgeIndex == -1)
         return false;
      getEdge(edgeIndex, closestEdgeToPack);
      return true;
   }

   public LineSegment2d getClosestEdgeCopy(Point2DReadOnly point)
   {
      LineSegment2d closestEdge = new LineSegment2d();
      if (getClosestEdge(closestEdge, point))
         return closestEdge;
      else
         return null;
   }

   public int getClosestVertexIndex(Point2DReadOnly point)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestVertexIndexToPoint2D(point, clockwiseOrderedListOfPoints, numberOfVertices);
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

   public int getClosestVertexIndex(Line2d line)
   {
      checkIfUpToDate();
      return EuclidGeometryPolygonTools.closestVertexIndexToLine2D(line.point, line.normalizedVector, clockwiseOrderedListOfPoints, numberOfVertices);
   }
   
   public boolean getClosestVertex(Line2d line, Point2DBasics vertexToPack)
   {
      int vertexIndex = getClosestVertexIndex(line);
      if (vertexIndex == -1)
         return false;
      vertexToPack.set(getVertex(vertexIndex));
      return true;
   }
   
   public Point2D getClosestVertexCopy(Line2d line)
   {
      int vertexIndex = getClosestVertexIndex(line);
      if (vertexIndex == -1)
         return null;
      return new Point2D(getVertex(vertexIndex));
   }

   public void getEdge(int edgeIndex, LineSegment2d edgeToPack)
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

   public double distance(Line2d line)
   {
      checkIfUpToDate();
      throw new RuntimeException("Not yet implemented");
   }

   public double distance(LineSegment2d lineSegment)
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
