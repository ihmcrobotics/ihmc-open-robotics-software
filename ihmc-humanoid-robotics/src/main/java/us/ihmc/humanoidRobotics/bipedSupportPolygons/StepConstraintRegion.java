package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.*;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.RegionInWorldInterface;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;

import java.util.ArrayList;
import java.util.List;

public class StepConstraintRegion implements RegionInWorldInterface<StepConstraintRegion>
{
   private final ConcavePolygon2D concaveHull;
   private final ConvexPolygon2D convexHull = new ConvexPolygon2D();

   private final RigidBodyTransform fromLocalToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform fromWorldToLocalTransform = new RigidBodyTransform();

   private final RecyclingArrayList<ConcavePolygon2D> holesInRegion = new RecyclingArrayList<>(ConcavePolygon2D::new);

   private final BoundingBox3D boundingBox3dInWorld = new BoundingBox3D(new Point3D(Double.NaN, Double.NaN, Double.NaN),
                                                                        new Point3D(Double.NaN, Double.NaN, Double.NaN));
   private final Point3D tempPointForConvexPolygonProjection = new Point3D();

   private int regionId = -1;

   private final Point3DReadOnly origin = new Point3DReadOnly()
   {
      @Override
      public double getX()
      {
         return fromLocalToWorldTransform.getM03();
      }

      @Override
      public double getY()
      {
         return fromLocalToWorldTransform.getM13();
      }

      @Override
      public double getZ()
      {
         return fromLocalToWorldTransform.getM23();
      }
   };
   private final UnitVector3DReadOnly normal = new UnitVector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return getRawX();
      }

      @Override
      public double getY()
      {
         return getRawY();
      }

      @Override
      public double getZ()
      {
         return getRawZ();
      }

      @Override
      public double getRawX()
      {
         return fromLocalToWorldTransform.getM02();
      }

      @Override
      public double getRawY()
      {
         return fromLocalToWorldTransform.getM12();
      }

      @Override
      public double getRawZ()
      {
         return fromLocalToWorldTransform.getM22();
      }

      @Override
      public boolean isDirty()
      {
         return false;
      }
   };

   public StepConstraintRegion()
   {
      concaveHull = new ConcavePolygon2D();
      updateConvexHull();
      updateBoundingBox();
   }

   public void setRegionId(int regionId)
   {
      this.regionId = regionId;
   }

   public int getRegionId()
   {
      return regionId;
   }

   public StepConstraintRegion(RigidBodyTransformReadOnly transformToWorld, List<? extends Point2DReadOnly> concaveHullVertices)
   {
      this(transformToWorld, Vertex2DSupplier.asVertex2DSupplier(concaveHullVertices));
   }

   public StepConstraintRegion(RigidBodyTransformReadOnly transformToWorld, Vertex2DSupplier vertexSupplier)
   {
      this(transformToWorld, vertexSupplier, new ArrayList<>());
   }

   public StepConstraintRegion(RigidBodyTransformReadOnly transformToWorld,
                               Vertex2DSupplier vertexSupplier,
                               List<? extends ConcavePolygon2DReadOnly> holesInRegion)
   {
      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);

      concaveHull = new ConcavePolygon2D(vertexSupplier);
      checkConcaveHullRepeatVertices();
      updateConvexHull();
      updateBoundingBox();

      for (ConcavePolygon2DReadOnly hole : holesInRegion)
         this.holesInRegion.add().set(hole);
   }

   public void clear()
   {
      fromLocalToWorldTransform.setToNaN();
      fromWorldToLocalTransform.setToNaN();
      for (int i = 0; i < holesInRegion.size(); i++)
         holesInRegion.get(i).clear();
      holesInRegion.clear();

      concaveHull.clear();
      convexHull.clear();
      boundingBox3dInWorld.setToNaN();
   }

   public void set(StepConstraintRegion other)
   {
      clear();

      fromLocalToWorldTransform.set(other.fromLocalToWorldTransform);
      fromWorldToLocalTransform.set(other.fromWorldToLocalTransform);
      for (int i = 0; i < other.getNumberOfHolesInRegion(); i++)
         holesInRegion.add().set(other.holesInRegion.get(i));

      concaveHull.set(other.concaveHull);
      convexHull.set(other.convexHull);
      updateBoundingBox();
   }

   public void addOffset(Vector3DReadOnly offset)
   {
      fromLocalToWorldTransform.prependTranslation(offset);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);

      updateBoundingBox();
   }

   private void checkConcaveHullRepeatVertices()
   {
      if (concaveHull.getNumberOfVertices() < 2)
         return;

      for (int i = 0; i < concaveHull.getNumberOfVertices(); i++)
      {
         int nextIndex = (i + 1) % concaveHull.getNumberOfVertices();

         Point2DReadOnly vertex = concaveHull.getVertex(i);
         Point2DReadOnly nextVertex = concaveHull.getVertex(nextIndex);

         if (vertex.distance(nextVertex) < 1e-7)
         {
            LogTools.error("Setting concave hull with repeat vertices" + vertex);
         }
      }
   }

   private void updateConvexHull()
   {
      convexHull.set(concaveHull);
   }

   public void set(RigidBodyTransformReadOnly transformToWorld, List<? extends Point2DReadOnly> concaveHullsVertices)
   {
      set(transformToWorld, concaveHullsVertices, null);
   }

   public void set(RigidBodyTransformReadOnly transformToWorld, List<? extends Point2DReadOnly> concaveHullsVertices, List<ConcavePolygon2D> holesInRegion)
   {
      clear();

      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);

      this.concaveHull.clear();
      for (int i = 0; i < concaveHullsVertices.size(); i++)
         concaveHull.addVertex(concaveHullsVertices.get(i));
      concaveHull.update();

      updateConvexHull();
      updateBoundingBox();

      if (holesInRegion != null)
      {
         for (int i = 0; i < holesInRegion.size(); i++)
            this.holesInRegion.add().set(holesInRegion.get(i));
      }
   }

   /**
    * Computes the z-coordinate in world of the plane for a given xy-coordinates in world.
    *
    * @param xWorld x-coordinate of the query
    * @param yWorld y-coordinate of the query
    * @return the z-coordinate
    */
   public double getPlaneZGivenXY(double xWorld, double yWorld)
   {
      // The three components of the plane origin
      double x0 = fromLocalToWorldTransform.getM03();
      double y0 = fromLocalToWorldTransform.getM13();
      double z0 = fromLocalToWorldTransform.getM23();
      // The three components of the plane normal
      double a = fromLocalToWorldTransform.getM02();
      double b = fromLocalToWorldTransform.getM12();
      double c = fromLocalToWorldTransform.getM22();

      // Given the plane equation: a*x + b*y + c*z + d = 0, with d = -(a*x0 + b*y0 + c*z0), we find z:
      double z = a / c * (x0 - xWorld) + b / c * (y0 - yWorld) + z0;
      return z;
   }

   public List<? extends Point2DReadOnly> getConcaveHullVertices()
   {
      return concaveHull.getVertexBufferView();
   }

   public int getConcaveHullSize()
   {
      return concaveHull.getNumberOfVertices();
   }

   public int getNumberOfHolesInRegion()
   {
      return holesInRegion.size();
   }

   public Tuple3DReadOnly getRegionOriginInWorld()
   {
      return fromLocalToWorldTransform.getTranslation();
   }

   public void getRegionOriginInWorld(Point3DBasics originToPack)
   {
      originToPack.set(fromLocalToWorldTransform.getTranslation());
   }

   public ConvexPolygon2DReadOnly getConvexHullInConstraintRegion()
   {
      return convexHull;
   }

   public List<? extends ConcavePolygon2DReadOnly> getHolesInConstraintRegion()
   {
      return holesInRegion;
   }

   public ConcavePolygon2DReadOnly getHoleInConstraintRegion(int idx)
   {
      return holesInRegion.get(idx);
   }

   public RigidBodyTransformReadOnly getTransformToWorld()
   {
      return fromLocalToWorldTransform;
   }

   public RigidBodyTransformReadOnly getTransformToLocal()
   {
      return fromWorldToLocalTransform;
   }

   /** {@inheritDoc} */
   @Override
   public Point3DReadOnly getPoint()
   {
      return origin;
   }

   /** {@inheritDoc} */
   @Override
   public UnitVector3DReadOnly getNormal()
   {
      return normal;
   }

   public ConcavePolygon2DReadOnly getConcaveHull()
   {
      return concaveHull;
   }

   public Point2DReadOnly getConcaveHullVertexInRegionFrame(int i)
   {
      return concaveHull.getVertex(i);
   }

   public BoundingBox3DReadOnly getBoundingBox3dInWorld()
   {
      return this.boundingBox3dInWorld;
   }

   public boolean isPointInside(double xInLocal, double yInLocal)
   {
      if (!concaveHull.isPointInside(xInLocal, yInLocal))
         return false;

      for (int i = 0; i < getNumberOfHolesInRegion(); i++)
      {
         if (getHoleInConstraintRegion(i).isPointInside(xInLocal, yInLocal))
            return false;
      }

      return true;
   }

   public boolean epsilonEquals(StepConstraintRegion other, double epsilon)
   {
      if (!fromLocalToWorldTransform.epsilonEquals(other.fromLocalToWorldTransform, epsilon))
         return false;
      // Not necessary, but just in case
      if (!fromWorldToLocalTransform.epsilonEquals(other.fromWorldToLocalTransform, epsilon))
         return false;

      if (!concaveHull.epsilonEquals(other.concaveHull, epsilon))
         return false;

      if (getNumberOfHolesInRegion() != other.getNumberOfHolesInRegion())
         return false;

      for (int i = 0; i < getNumberOfHolesInRegion(); i++)
      {
         if (!holesInRegion.get(i).epsilonEquals(other.holesInRegion.get(i), epsilon))
            return false;
      }
      return true;
   }

   public boolean isPolygonInWorldIntersecting(ConvexPolygon2DReadOnly polygonInWorld)
   {
      BoundingBox2DReadOnly polygonBoundingBox = polygonInWorld.getBoundingBox();
      if (!boundingBox3dInWorld.intersectsInclusiveInXYPlane(polygonBoundingBox))
         return false;

      // Instead of projecting all the polygons of this region onto the world XY-plane,
      // the given convex polygon is projected along the z-world axis to be snapped onto plane.
      ConvexPolygon2D localPolygon = new ConvexPolygon2D(polygonInWorld);
      localPolygon.applyTransform(fromWorldToLocalTransform, false);

      return GeometryPolygonTools.doPolygonsIntersect(concaveHull, localPolygon);
   }

   private void updateBoundingBox()
   {
      boundingBox3dInWorld.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);

      for (int i = 0; i < concaveHull.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = concaveHull.getVertex(i);
         tempPointForConvexPolygonProjection.set(vertex.getX(), vertex.getY(), 0.0);
         fromLocalToWorldTransform.transform(tempPointForConvexPolygonProjection);
         this.boundingBox3dInWorld.updateToIncludePoint(tempPointForConvexPolygonProjection);
      }

      Point3DReadOnly minPoint = boundingBox3dInWorld.getMinPoint();
      Point3DReadOnly maxPoint = boundingBox3dInWorld.getMaxPoint();

      this.boundingBox3dInWorld.setMin(minPoint.getX(), minPoint.getY(), minPoint.getZ());
      this.boundingBox3dInWorld.setMax(maxPoint.getX(), maxPoint.getY(), maxPoint.getZ());
   }
}
