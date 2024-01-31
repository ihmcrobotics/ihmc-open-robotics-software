package us.ihmc.robotics.geometry.concavePolygon2D;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DBasics;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

public class ConcavePolygon2D implements ConcavePolygon2DBasics
{
   private final boolean clockwiseOrdered = true;

   private int numberOfVertices = 0;

   private double area;

   private boolean isUpToDate = false;

   private final List<Point2D> vertexBuffer = new ArrayList<>();
   private final List<Point2D> vertexBufferView = Collections.unmodifiableList(vertexBuffer);

   private final BoundingBox2D boundingBox = new BoundingBox2D();
   private final Point2D centroid = new Point2D();

   public ConcavePolygon2D()
   {
   }

   public ConcavePolygon2D(Vertex2DSupplier vertex2DSupplier)
   {
      set(vertex2DSupplier);
   }

   @Override
   public void clear()
   {
      numberOfVertices = 0;
      area = Double.NaN;
      centroid.setToNaN();
      boundingBox.setToNaN();
      isUpToDate = false;
   }

   @Override
   public void clearAndUpdate()
   {
      clear();
      isUpToDate = true;
   }

   @Override
   public boolean isClockwiseOrdered()
   {
      return clockwiseOrdered;
   }

   @Override
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
      for (int index = indexOfVertexToRemove; index < numberOfVertices - 1; index++)
      {
         Point2DReadOnly nextVertex = getVertex(index + 1);
         setOrCreate(nextVertex.getX(), nextVertex.getY(), index);
      }
      numberOfVertices--;
   }

   @Override
   public void insertVertex(int indexToSetVertex, double vertexXToSet, double vertexYToSet)
   {
      checkNonEmpty();
      checkIndexInBoundaries(indexToSetVertex);

      numberOfVertices++;

      for (int vertex = numberOfVertices - 2; vertex >= indexToSetVertex; vertex--)
      {
         Point2DReadOnly vertexToShift = getVertex(vertex);
         setOrCreate(vertexToShift.getX(), vertexToShift.getY(), vertex + 1);
      }
      setOrCreate(vertexXToSet, vertexYToSet, indexToSetVertex);

      isUpToDate = false;
   }

   @Override
   public void update()
   {
      if (isUpToDate)
         return;

      ensureClockwiseOrdering();
      if (!GeometryPolygonTools.isSimplePolygon(vertexBuffer, numberOfVertices))
         throw new ComplexPolygonException("Polygon is not simple, as in it has self intersections.");

      removePointsThatAreNotVertices();

      // TODO should maybe do alpha shape?
      isUpToDate = true;

      updateCentroidAndArea();
      updateBoundingBox();
   }


   private void ensureClockwiseOrdering()
   {
      if (!GeometryPolygonTools.isClockwiseOrdered(vertexBuffer, numberOfVertices))
         Collections.reverse(vertexBuffer);
   }

   @Override
   public void addVertex(double x, double y)
   {
      isUpToDate = false;
      setOrCreate(x, y, numberOfVertices);
      numberOfVertices++;
   }

   @Override
   public int getNumberOfVertices()
   {
      return numberOfVertices;
   }

   @Override
   public List<? extends Point2DReadOnly> getVertexBufferView()
   {
      return vertexBufferView;
   }

   @Override
   public BoundingBox2DBasics getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public boolean isUpToDate()
   {
      return isUpToDate;
   }

   @Override
   public double getArea()
   {
      checkIfUpToDate();
      return area;
   }

   @Override
   public Point2DReadOnly getCentroid()
   {
      checkIfUpToDate();
      return centroid;
   }

   @Override
   public void notifyVerticesChanged()
   {
      isUpToDate = false;
   }

   @Override
   public Point2DBasics getVertexUnsafe(int index)
   {
      checkNonEmpty();
      checkIndexInBoundaries(index);
      return vertexBuffer.get(index);
   }

   @Override
   public void updateCentroidAndArea()
   {
      area = EuclidGeometryPolygonTools.computeConvexPolygon2DArea(vertexBuffer, numberOfVertices, clockwiseOrdered, centroid);
   }

   private void setOrCreate(double x, double y, int i)
   {
      while (i >= vertexBuffer.size())
         vertexBuffer.add(new Point2D());
      vertexBuffer.get(i).set(x, y);
   }

   private void removePointsThatAreNotVertices()
   {
      int i = 0;
      while (i < getNumberOfVertices())
      {
         Point2DReadOnly previousVertex = getVertex(EuclidGeometryPolygonTools.previous(i, getNumberOfVertices()));
         Point2DReadOnly vertex = getVertex(i);
         Point2DReadOnly nextVertex = getVertex(EuclidGeometryPolygonTools.next(i, getNumberOfVertices()));

         if (EuclidGeometryTools.areLine2DsCollinear(previousVertex, vertex, vertex, nextVertex, 1e-3, 1e-4))
            removeVertex(i);
         else
            i++;
      }
   }
}
