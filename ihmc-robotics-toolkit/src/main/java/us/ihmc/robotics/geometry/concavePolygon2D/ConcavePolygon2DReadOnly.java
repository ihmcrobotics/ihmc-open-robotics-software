package us.ihmc.robotics.geometry.concavePolygon2D;

import us.ihmc.euclid.geometry.exceptions.EmptyPolygonException;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.List;

public interface ConcavePolygon2DReadOnly extends Vertex2DSupplier
{
   int getNumberOfVertices();

   List<? extends Point2DReadOnly> getVertexBufferView();

   default List<? extends Point2DReadOnly> getPolygonVerticesView()
   {
      return getVertexBufferView().subList(0, getNumberOfVertices());
   }

   BoundingBox2DReadOnly getBoundingBox();

   boolean isClockwiseOrdered();

   double getArea();

   Point2DReadOnly getCentroid();

   default boolean isPointInside(Point2DReadOnly point)
   {
      return isPointInside(point.getX(), point.getY());
   }

   default boolean isPointInside(double x, double y)
   {
      checkIfUpToDate();
      if (!getBoundingBox().isInsideInclusive(x, y))
         return false;

      return GeometryPolygonTools.isPoint2DInsideSimplePolygon2D(x, y, getVertexBufferView(), getNumberOfVertices());
   }

   default boolean isPointInsideEpsilon(Point2DReadOnly point, double epsilon)
   {
      return isPointInsideEpsilon(point.getX(), point.getY(), epsilon);
   }

   default boolean isPointInsideEpsilon(double x, double y, double epsilon)
   {
      checkIfUpToDate();
      if (!getBoundingBox().isInsideEpsilon(x, y, epsilon))
         return false;

      return GeometryPolygonTools.isPoint2DInsideSimplePolygon2D(x, y, getVertexBufferView(), getNumberOfVertices(), epsilon);
   }

   default boolean isEmpty()
   {
      return getNumberOfVertices() == 0;
   }

   default void checkNonEmpty()
   {
      if (isEmpty())
         throw new EmptyPolygonException("This polygon has no vertex.");
   }

   default void checkIndexInBoundaries(int index)
   {
      if (index < 0)
         throw new IndexOutOfBoundsException("vertexIndex < 0");
      if (index >= getNumberOfVertices())
         throw new IndexOutOfBoundsException("vertexIndex >= numberOfVertices. numberOfVertices = " + getNumberOfVertices());
   }

   default Point2DReadOnly getVertex(int index)
   {
      checkNonEmpty();
      checkIndexInBoundaries(index);
      return getVertexBufferView().get(index);
   }

   boolean isUpToDate();

   default void checkIfUpToDate()
   {
      if (!isUpToDate())
         throw new OutdatedPolygonException("Call the update method before doing any other calculation!");
   }

   default int getNextVertexIndex(int currentVertexIndex)
   {
      checkIfUpToDate();
      checkIndexInBoundaries(currentVertexIndex);
      checkNonEmpty();

      if (currentVertexIndex < getNumberOfVertices() - 1)
         return currentVertexIndex + 1;
      else
         return 0;
   }

   default int getPreviousVertexIndex(int currentVertexIndex)
   {
      checkIfUpToDate();
      checkIndexInBoundaries(currentVertexIndex);
      checkNonEmpty();

      if (currentVertexIndex < 1)
         return getNumberOfVertices() - 1;
      else
         return currentVertexIndex - 1;
   }

   default Point2DReadOnly getNextVertex(int index)
   {
      return getVertex(getNextVertexIndex(index));
   }

   default Point2DReadOnly getPreviousVertex(int index)
   {
      return getVertex(getPreviousVertexIndex(index));
   }


   default boolean epsilonEquals(ConcavePolygon2DReadOnly other, double epsilon)
   {
      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;

      for (int i = 0; i < other.getNumberOfVertices(); i++)
      {
         Point2DReadOnly thisVertex = getVertexBufferView().get(i);
         if (!other.epsilonContains(thisVertex, epsilon))
            return false;
      }

      return true;
   }

   default boolean epsilonContains(Point2DReadOnly thisVertex, double epsilon)
   {
      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         if (getVertexBufferView().get(i).epsilonEquals(thisVertex, epsilon))
            return true;
      }

      return false;
   }
}
