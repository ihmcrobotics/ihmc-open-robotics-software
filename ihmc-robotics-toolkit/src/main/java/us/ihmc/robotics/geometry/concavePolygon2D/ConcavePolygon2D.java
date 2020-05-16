package us.ihmc.robotics.geometry.concavePolygon2D;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

import java.util.ArrayList;
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
   public boolean isClockwiseOrdered()
   {
      return clockwiseOrdered;
   }

   @Override
   public void update()
   {
      if (isUpToDate)
         return;

      if (!GeometryPolygonTools.isClockwiseOrdered(vertexBuffer))
         throw new RuntimeException("Vertices are not clockwise ordered.");
      if (!GeometryPolygonTools.isSimplePolygon(vertexBuffer))
         throw new RuntimeException("Polygon is not simple, as in it has self intersections.");

      // TODO should maybe do alpha shape?
      numberOfVertices = vertexBuffer.size();
      isUpToDate = true;

      updateCentroidAndArea();
      updateBoundingBox();
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
}
