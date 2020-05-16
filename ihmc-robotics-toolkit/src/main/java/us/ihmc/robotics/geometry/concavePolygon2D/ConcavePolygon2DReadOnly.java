package us.ihmc.robotics.geometry.concavePolygon2D;

import us.ihmc.euclid.geometry.exceptions.EmptyPolygonException;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.List;

public interface ConcavePolygon2DReadOnly extends Vertex2DSupplier
{
   int getNumberOfVertices();

   List<? extends Point2DReadOnly> getVertexBufferView();

   BoundingBox2DReadOnly getBoundingBox();

   boolean isClockwiseOrdered();

   double getArea();

   Point2DReadOnly getCentroid();

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
}
