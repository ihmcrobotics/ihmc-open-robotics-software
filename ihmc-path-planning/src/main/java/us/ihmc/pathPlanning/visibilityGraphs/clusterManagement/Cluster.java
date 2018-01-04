package us.ihmc.pathPlanning.visibilityGraphs.clusterManagement;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;

public class Cluster
{
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private final List<Point3D> rawPointsLocal = new ArrayList<>();
   private final List<Point2D> navigableExtrusionsInLocal = new ArrayList<>();
   private final List<Point2D> nonNavigableExtrusionsInLocal = new ArrayList<>();

   private final BoundingBox2D nonNavigableExtrusionsBoundingBox = new BoundingBox2D();

   public enum ExtrusionSide
   {
      INSIDE, OUTSIDE
   };

   private ExtrusionSide extrusionSide = ExtrusionSide.OUTSIDE;

   public enum Type
   {
      LINE, MULTI_LINE, POLYGON
   };

   private Type type = Type.POLYGON;

   public Cluster()
   {
   }

   public void updateBoundingBox()
   {
      nonNavigableExtrusionsInLocal.forEach(nonNavigableExtrusionsBoundingBox::updateToIncludePoint);
   }

   public boolean isInsideNonNavigableZone(Point2DReadOnly query)
   {
      if (extrusionSide == ExtrusionSide.INSIDE)
      {
         if (!nonNavigableExtrusionsBoundingBox.isInsideInclusive(query))
            return true;
         return !PlanarRegionTools.isPointInsidePolygon(nonNavigableExtrusionsInLocal, query);
      }
      else
      {
         if (!nonNavigableExtrusionsBoundingBox.isInsideInclusive(query))
            return false;
         return PlanarRegionTools.isPointInsidePolygon(nonNavigableExtrusionsInLocal, query);
      }
   }

   public void setExtrusionSide(ExtrusionSide extrusionSide)
   {
      this.extrusionSide = extrusionSide;
   }

   public ExtrusionSide getExtrusionSide()
   {
      return extrusionSide;
   }

   public void setType(Type type)
   {
      this.type = type;
   }

   public Type getType()
   {
      return type;
   }

   public void setTransformToWorld(RigidBodyTransform transform)
   {
      transformToWorld.set(transform);
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return transformToWorld;
   }

   public void addRawPointInLocal3D(Point3DReadOnly pointInLocal)
   {
      rawPointsLocal.add(new Point3D(pointInLocal));
   }

   public void addRawPointInWorld3D(Point3DReadOnly pointInWorld)
   {
      addRawPointInLocal3D(toLocal3D(pointInWorld));
   }

   public void addRawPointsInLocal2D(List<? extends Point2DReadOnly> pointsInLocal)
   {
      List<Point3D> point3DsInLocal = pointsInLocal.stream().map(Point3D::new).collect(Collectors.toList());
      addRawPointsInLocal3D(point3DsInLocal);
   }

   public void addRawPointsInLocal3D(List<? extends Point3DReadOnly> pointsInLocal)
   {
      pointsInLocal.forEach(point -> rawPointsLocal.add(new Point3D(point)));
   }

   public void addRawPointsInWorld3D(List<? extends Point3DReadOnly> pointsInWorld)
   {
      List<Point3D> pointsInLocal = pointsInWorld.stream().map(this::toLocal3D).collect(Collectors.toList());
      addRawPointsInLocal3D(pointsInLocal);
   }

   public void addRawPointsInLocal2D(Point2DReadOnly[] pointsInLocal)
   {
      addRawPointsInLocal2D(Arrays.asList(pointsInLocal));
   }

   public void addRawPointsInLocal3D(Point3DReadOnly[] pointsInLocal)
   {
      addRawPointsInLocal3D(Arrays.asList(pointsInLocal));
   }

   public void addRawPointsInWorld3D(Point3DReadOnly[] pointsInWorld)
   {
      addRawPointsInWorld3D(Arrays.asList(pointsInWorld));
   }

   public int getNumberOfRawPoints()
   {
      return rawPointsLocal.size();
   }

   public Point3D getRawPointInLocal3D(int i)
   {
      return rawPointsLocal.get(i);
   }

   public Point3D getLastRawPointInLocal3D()
   {
      return rawPointsLocal.get(getNumberOfRawPoints() - 1);
   }

   public List<Point3D> getRawPointsInLocal3D()
   {
      return rawPointsLocal;
   }

   public Point3D getRawPointInWorld3D(int i)
   {
      return toWorld3D(rawPointsLocal.get(i));
   }

   public Point3D getLastRawPointInWorld3D()
   {
      return toWorld3D(getLastRawPointInLocal3D());
   }

   public List<Point3D> getRawPointsInWorld3D()
   {
      return rawPointsLocal.stream().map(this::toWorld3D).collect(Collectors.toList());
   }

   public Point2D getRawPointInLocal2D(int i)
   {
      return new Point2D(rawPointsLocal.get(i));
   }

   public Point2D getLastRawPointInLocal2D()
   {
      return new Point2D(rawPointsLocal.get(getNumberOfRawPoints() - 1));
   }

   public List<Point2D> getRawPointsInLocal2D()
   {
      return rawPointsLocal.stream().map(Point2D::new).collect(Collectors.toList());
   }

   public void addNavigableExtrusionInLocal2D(Point2DReadOnly navigableExtrusionInLocal)
   {
      navigableExtrusionsInLocal.add(new Point2D(navigableExtrusionInLocal));
   }

   public void addNavigableExtrusionInWorld3D(Point3DReadOnly navigableExtrusionInWorld)
   {
      navigableExtrusionsInLocal.add(toLocal2D(navigableExtrusionInWorld));
   }

   public void addNavigableExtrusionsInLocal2D(List<? extends Point2DReadOnly> navigableExtrusionInLocal)
   {
      navigableExtrusionInLocal.forEach(this::addNavigableExtrusionInLocal2D);
   }

   public void addNavigableExtrusionsInWorld3D(List<? extends Point3DReadOnly> navigableExtrusionInWorld)
   {
      navigableExtrusionInWorld.forEach(this::addNavigableExtrusionInWorld3D);
   }

   public void addFirstNavigableExtrusionInLocal2D(Point2DReadOnly navigableExtrusionInLocal)
   {
      navigableExtrusionsInLocal.add(0, new Point2D(navigableExtrusionInLocal));
   }

   public void addFirstNavigableExtrusionInWorld3D(Point3DReadOnly navigableExtrusionInWorld)
   {
      navigableExtrusionsInLocal.add(0, toLocal2D(navigableExtrusionInWorld));
   }

   public void addNonNavigableExtrusionInLocal2D(Point2DReadOnly nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionsInLocal.add(new Point2D(nonNavigableExtrusionInLocal));
   }

   public void addNonNavigableExtrusionInWorld3D(Point3DReadOnly nonNavigableExtrusionInWorld)
   {
      nonNavigableExtrusionsInLocal.add(toLocal2D(nonNavigableExtrusionInWorld));
   }

   public void addNonNavigableExtrusionsInLocal2D(List<? extends Point2DReadOnly> nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionInLocal.forEach(this::addNonNavigableExtrusionInLocal2D);
   }

   public void addNonNavigableExtrusionsInWorld3D(List<? extends Point3DReadOnly> nonNavigableExtrusionInWorld)
   {
      nonNavigableExtrusionInWorld.forEach(this::addNonNavigableExtrusionInWorld3D);
   }

   public void addFirstNonNavigableExtrusionInLocal2D(Point2DReadOnly nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionsInLocal.add(0, new Point2D(nonNavigableExtrusionInLocal));
   }

   public void addFirstNonNavigableExtrusionInWorld3D(Point3DReadOnly nonNavigableExtrusionInWorld)
   {
      nonNavigableExtrusionsInLocal.add(0, toLocal2D(nonNavigableExtrusionInWorld));
   }

   public int getNumberOfNavigableExtrusions()
   {
      return navigableExtrusionsInLocal.size();
   }

   public Point2D getNavigableExtrusionInLocal2D(int i)
   {
      return navigableExtrusionsInLocal.get(i);
   }

   public List<Point2D> getNavigableExtrusionsInLocal2D()
   {
      return navigableExtrusionsInLocal;
   }

   public Point3D getNavigableExtrusionInWorld3D(int i)
   {
      return toWorld3D(getNavigableExtrusionInLocal2D(i));
   }

   public List<Point3D> getNavigableExtrusionsInWorld3D()
   {
      return navigableExtrusionsInLocal.stream().map(this::toWorld3D).collect(Collectors.toList());
   }

   public BoundingBox2D getNonNavigableExtrusionsBoundingBox()
   {
      return nonNavigableExtrusionsBoundingBox;
   }

   public int getNumberOfNonNavigableExtrusions()
   {
      return nonNavigableExtrusionsInLocal.size();
   }

   public Point2D getNonNavigableExtrusionInLocal2D(int i)
   {
      return nonNavigableExtrusionsInLocal.get(i);
   }

   public List<Point2D> getNonNavigableExtrusionsInLocal2D()
   {
      return nonNavigableExtrusionsInLocal;
   }

   public Point3D getNonNavigableExtrusionInWorld3D(int i)
   {
      return toWorld3D(getNonNavigableExtrusionInLocal2D(i));
   }

   public List<Point3D> getNonNavigableExtrusionsInWorld3D()
   {
      return nonNavigableExtrusionsInLocal.stream().map(this::toWorld3D).collect(Collectors.toList());
   }

   private Point3D toWorld3D(Point2DReadOnly pointInLocal)
   {
      return toWorld3D(new Point3D(pointInLocal));
   }

   private Point3D toWorld3D(Point3DReadOnly pointInLocal)
   {
      Point3D pointInWorld = new Point3D(pointInLocal);
      transformToWorld.transform(pointInWorld);
      return pointInWorld;
   }

   private Point3D toLocal3D(Point3DReadOnly pointInWorld)
   {
      Point3D pointInLocal = new Point3D();
      transformToWorld.inverseTransform(pointInWorld, pointInLocal);
      return pointInLocal;
   }

   private Point2D toLocal2D(Point3DReadOnly pointInWorld)
   {
      return new Point2D(toLocal3D(pointInWorld));
   }
}
