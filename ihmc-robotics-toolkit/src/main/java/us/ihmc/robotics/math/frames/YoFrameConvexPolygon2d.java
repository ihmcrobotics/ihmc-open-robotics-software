package us.ihmc.robotics.math.frames;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryPolygonTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoFrameConvexPolygon2d implements FixedFrameConvexPolygon2DBasics
{
   private final List<YoFramePoint2d> vertexBuffer = new ArrayList<YoFramePoint2d>();
   private final List<FixedFramePoint2DBasics> vertexBufferView = Collections.unmodifiableList(vertexBuffer);
   /**
    * Field for future expansion of {@code ConvexPolygon2d} to enable having the vertices in
    * clockwise or counter-clockwise ordered.
    */
   private final boolean clockwiseOrdered = true;
   /** Rigid-body transform used to perform garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();
   private final YoInteger numberOfVertices;
   /** The reference frame in which this polygon is currently expressed. */
   private final ReferenceFrame referenceFrame;

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
   private final FixedFramePoint2DBasics centroid = new FixedFramePoint2DBasics()
   {
      private double x, y;

      @Override
      public void setX(double x)
      {
         this.x = x;
      };

      @Override
      public void setY(double y)
      {
         this.y = y;
      }

      @Override
      public double getX()
      {
         return x;
      }

      @Override
      public double getY()
      {
         return y;
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public String toString()
      {
         return EuclidCoreIOTools.getTuple2DString(this) + "-" + referenceFrame;
      }
   };
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
   /** Vertex to store intermediate results to allow garbage free operations. */
   private final Point3D vertex3D = new Point3D();

   private AtomicBoolean hasChanged = new AtomicBoolean(true);

   public YoFrameConvexPolygon2d(String namePrefix, String nameSuffix, ReferenceFrame referenceFrame, int maxNumberOfVertices, YoVariableRegistry registry)
   {
      this.numberOfVertices = new YoInteger(namePrefix + "NumVertices" + nameSuffix, registry);

      this.referenceFrame = referenceFrame;

      for (int i = 0; i < maxNumberOfVertices; i++)
      {
         YoFramePoint2d point = new YoFramePoint2d(namePrefix + "_" + i + "_", nameSuffix, referenceFrame, registry);
         vertexBuffer.add(point);
      }
   }

   public YoFrameConvexPolygon2d(String namePrefix, ReferenceFrame referenceFrame, int maxNumberOfVertices, YoVariableRegistry registry)
   {
      this(namePrefix, "", referenceFrame, maxNumberOfVertices, registry);
   }

   public YoFrameConvexPolygon2d(List<YoFramePoint2d> yoFramePoints, YoInteger yoNumVertices, ReferenceFrame referenceFrame)
   {
      this.numberOfVertices = yoNumVertices;

      this.referenceFrame = referenceFrame;

      for (YoFramePoint2d point : yoFramePoints)
      {
         this.vertexBuffer.add(point);
      }
   }

   /** {@inheritDoc} */
   @Override
   public FixedFramePoint2DBasics getVertexUnsafe(int index)
   {
      checkNonEmpty();
      checkIndexInBoundaries(index);
      return vertexBuffer.get(index);
   }

   /** {@inheritDoc} */
   @Override
   public void notifyVerticesChanged()
   {
      isUpToDate = false;
   }

   /** {@inheritDoc} */
   @Override
   public void clear()
   {
      numberOfVertices.set(0);
      area = Double.NaN;
      centroid.setToNaN();
      boundingBox.setToNaN();
      isUpToDate = false;
   }

   /** {@inheritDoc} */
   @Override
   public void clearAndUpdate()
   {
      clear();
      isUpToDate = true;
   }

   /** {@inheritDoc} */
   @Override
   public void addVertexMatchingFrame(ReferenceFrame referenceFrame, Point2DReadOnly vertex, boolean checkIfTransformInXYPlane)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (getReferenceFrame() == referenceFrame)
      {
         addVertex(vertex);
      }
      else
      {
         referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, getReferenceFrame());
         addVertex(vertex);
         getVertexUnsafe(getNumberOfVertices() - 1).applyTransform(transformToDesiredFrame, checkIfTransformInXYPlane);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void addVertexMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly vertex)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (getReferenceFrame() == referenceFrame)
      {
         addVertex(vertex);
      }
      else
      {
         referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, getReferenceFrame());
         transformToDesiredFrame.transform(vertex, vertex3D);
         addVertex(vertex3D);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void update()
   {
      if (isUpToDate)
         return;

      numberOfVertices.set(EuclidGeometryPolygonTools.inPlaceGiftWrapConvexHull2D(vertexBuffer, numberOfVertices.getValue()));
      isUpToDate = true;

      updateCentroidAndArea();
      updateBoundingBox();
   }

   /** {@inheritDoc} */
   @Override
   public void updateCentroidAndArea()
   {
      area = EuclidGeometryPolygonTools.computeConvexPolyong2DArea(vertexBuffer, numberOfVertices.getValue(), clockwiseOrdered, centroid);
   }

   /** {@inheritDoc} */
   @Override
   public void addVertex(double x, double y)
   {
      isUpToDate = false;
      YoFramePoint2d newVertex = vertexBuffer.get(numberOfVertices.getValue());
      if (newVertex == null)
         throw new RuntimeException("This polygon has reached its maximum number of vertices.");
      newVertex.set(x, y);
      numberOfVertices.increment();
   }

   /** {@inheritDoc} */
   @Override
   public void removeVertex(int indexOfVertexToRemove)
   {
      checkNonEmpty();
      checkIndexInBoundaries(indexOfVertexToRemove);

      if (indexOfVertexToRemove == numberOfVertices.getValue() - 1)
      {
         numberOfVertices.decrement();
         ;
         return;
      }
      isUpToDate = false;
      swap(vertexBuffer, indexOfVertexToRemove, numberOfVertices.getValue());
      numberOfVertices.decrement();
   }

   private static void swap(List<YoFramePoint2d> vertexBuffer, int i, int j)
   {
      if (i == j)
         return;

      YoFramePoint2d iVertex = vertexBuffer.get(i);
      double x_i = iVertex.getX();
      double y_i = iVertex.getY();
      YoFramePoint2d jVertex = vertexBuffer.get(j);
      double x_j = jVertex.getX();
      double y_j = jVertex.getY();

      iVertex.set(x_j, y_j);
      jVertex.set(x_i, y_i);
   }

   /** {@inheritDoc} */
   @Override
   public List<? extends FramePoint2DReadOnly> getVertexBufferView()
   {
      return vertexBufferView;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint2DReadOnly getCentroid()
   {
      return centroid;
   }

   /** {@inheritDoc} */
   @Override
   public boolean isClockwiseOrdered()
   {
      return clockwiseOrdered;
   }

   /** {@inheritDoc} */
   @Override
   public boolean isUpToDate()
   {
      return isUpToDate;
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfVertices()
   {
      return numberOfVertices.getValue();
   }

   public int getMaxNumberOfVertices()
   {
      return vertexBuffer.size();
   }

   public YoInteger getYoNumberOfVertices()
   {
      return numberOfVertices;
   }

   public List<YoFramePoint2d> getVertexBuffer()
   {
      return vertexBuffer;
   }

   /** {@inheritDoc} */
   @Override
   public double getArea()
   {
      return area;
   }

   /** {@inheritDoc} */
   @Override
   public BoundingBox2DBasics getBoundingBox()
   {
      return boundingBox;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getConvexPolygon2DString(this) + "-" + referenceFrame;
   }

   public boolean getHasChangedAndReset()
   {
      return hasChanged.getAndSet(false);
   }
}
