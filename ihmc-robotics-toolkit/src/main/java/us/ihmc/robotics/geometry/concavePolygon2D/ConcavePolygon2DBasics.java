package us.ihmc.robotics.geometry.concavePolygon2D;

import us.ihmc.euclid.geometry.exceptions.EmptyPolygonException;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DBasics;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

import java.util.List;

public interface ConcavePolygon2DBasics extends ConcavePolygon2DReadOnly, Transformable
{
   void update();

   void updateCentroidAndArea();

   void clear();

   void clearAndUpdate();

   default void setToZero()
   {
      this.clear();
      this.addVertex(0.0D, 0.0D);
      this.update();
   }

   default void setToNaN()
   {
      this.clear();
      this.addVertex(0.0D / 0.0, 0.0D / 0.0);
      this.update();
   }

   default void updateBoundingBox()
   {
      BoundingBox2DBasics boundingBox = this.getBoundingBox();
      boundingBox.setToNaN();
      boundingBox.updateToIncludePoints(this);
   }

   void addVertex(double var1, double var3);

   void removeVertex(int indexOfVertexToRemove);

   default void insertVertex(int indexToSetVertex, Point2DReadOnly vertexToSet)
   {
      insertVertex(indexToSetVertex, vertexToSet.getX(), vertexToSet.getY());
   }

   void insertVertex(int indexToSetVertex, double vertexXToSet, double vertexYToSet);


   BoundingBox2DBasics getBoundingBox();

   void notifyVerticesChanged();

   Point2DBasics getVertexUnsafe(int var1);

   default void set(Vertex2DSupplier vertex2DSupplier)
   {
      clear();
      addVertices(vertex2DSupplier);
      update();
   }

   default void addVertices(Vertex2DSupplier vertex2DSupplier)
   {
      for (int index = 0; index < vertex2DSupplier.getNumberOfVertices(); index++)
      {
         addVertex(vertex2DSupplier.getVertex(index));
      }
   }

   default void addVertex(Point2DReadOnly vertex)
   {
      this.addVertex(vertex.getX(), vertex.getY());
   }

   default void addVertex(Point3DReadOnly vertex)
   {
      this.addVertex(vertex.getX(), vertex.getY());
   }

   default void applyTransform(Transform transform)
   {
      this.applyTransform(transform, true);
   }

   default void applyTransform(Transform transform, boolean checkIfTransformInXYPlane)
   {
      checkIfUpToDate();
      notifyVerticesChanged();

      for (int i = 0; i < this.getNumberOfVertices(); ++i)
      {
         this.getVertexUnsafe(i).applyTransform(transform, checkIfTransformInXYPlane);
      }

      update();
   }

   default void applyInverseTransform(Transform transform)
   {
      this.applyInverseTransform(transform, true);
   }

   default void applyInverseTransform(Transform transform, boolean checkIfTransformInXYPlane)
   {
      checkIfUpToDate();
      notifyVerticesChanged();

      for (int i = 0; i < this.getNumberOfVertices(); ++i)
      {
         this.getVertexUnsafe(i).applyInverseTransform(transform, checkIfTransformInXYPlane);
      }

      update();
   }
}
