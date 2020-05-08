package us.ihmc.pathPlanning.visibilityGraphs.clusterManagement;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionTools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

/**
 * A Cluster typically represents an obstacle over a planar region.
 * It consists of the 3D points of the obstacle, the 2D points of the navigable spots on the planar region
 * and the 2D points of the nonNavigable spots on the planar region resulting from the obstacle.
 * It contains the RigidBodyTransform for moving the points from local to world coordinates.
 *
 * There is one cluster per planar region per obstacle. Each PlanarRegion has a closed concave hull.
 */
public class Cluster
{
   //TODO: +++JerryPratt: Refactor packages. No need for clusterManagement package.
   //TODO: +++JerryPratt: Remove so many methods and clean up the API so easier to maintain.

   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   private final List<Point3DReadOnly> rawPointsInLocal3D = new ArrayList<>();
   private final List<ExtrusionHull> preferredNavigableExtrusionsInLocal = new ArrayList<>();
   private final List<ExtrusionHull> preferredNonNavigableExtrusionsInLocal = new ArrayList<>();
   private final ExtrusionHull navigableExtrusionsInLocal = new ExtrusionHull();
   private final ExtrusionHull nonNavigableExtrusionInLocal = new ExtrusionHull();

   private final BoundingBox2D nonNavigableExtrusionsBoundingBox = new BoundingBox2D(Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   private final BoundingBox2D preferredNonNavigableExtrusionsBoundingBox = new BoundingBox2D(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

   public enum ExtrusionSide
   {
      INSIDE, OUTSIDE;

      public static ExtrusionSide[] values = values();

      public byte toByte()
      {
         return (byte) ordinal();
      }

      public static ExtrusionSide fromByte(byte enumAsByte)
      {
         if (enumAsByte == -1)
            return null;
         return values[enumAsByte];
      }
   };

   private final ExtrusionSide extrusionSide;

   public enum ClusterType
   {
      /**
       * MULTI_LINE means open at the end, not closed. This type is often used when projecting a PlanarRegion onto another parallel PlanarRegion.
       */
      MULTI_LINE,
      /**
       * POLYGON is a closed, perhaps concave, polygon.
       */
      POLYGON;

      public static ClusterType[] values = values();

      public byte toByte()
      {
         return (byte) ordinal();
      }

      public static ClusterType fromByte(byte enumAsByte)
      {
         if (enumAsByte == -1)
            return null;
         return values[enumAsByte];
      }
   };

   //TODO: +++JerryPratt: Try to make this final.
   private ClusterType type;

   public Cluster(ExtrusionSide extrusionSide, ClusterType type)
   {
      this.extrusionSide = extrusionSide;
      this.type = type;
   }

   public void clearNonNavigableExtrusions()
   {
      nonNavigableExtrusionsBoundingBox.setToNaN();
      nonNavigableExtrusionInLocal.clear();
   }

   public void clearPreferredNonNavigableExtrusions()
   {
      preferredNonNavigableExtrusionsBoundingBox.setToNaN();
      preferredNonNavigableExtrusionsInLocal.clear();
   }

   /** Returns true if it's on an edge.    */
   public boolean isInsideNonNavigableZone(Point2DReadOnly query)
   {
      return isInsideNonNavigableZone(query, 1e-7);
   }

   private boolean isInsideNonNavigableZone(Point2DReadOnly query, double epsilon)
   {
      if (nonNavigableExtrusionInLocal.isEmpty())
         return false;

      BoundingBox2DReadOnly boundingBox = getNonNavigableExtrusionsBoundingBox();

      if (extrusionSide == ExtrusionSide.INSIDE)
      {
         if (!boundingBox.isInsideEpsilon(query, epsilon))
            return true;
         return !PlanarRegionTools.isPointInsidePolygon(nonNavigableExtrusionInLocal.getPoints(), query, epsilon);
      }
      else
      {
         if (!boundingBox.isInsideEpsilon(query, epsilon))
            return false;
         return PlanarRegionTools.isPointInsidePolygon(nonNavigableExtrusionInLocal.getPoints(), query, epsilon);
      }
   }

   public boolean isInsidePreferredNonNavigableZone(Point2DReadOnly query)
   {
      if (preferredNonNavigableExtrusionsInLocal.isEmpty())
         return false;

      BoundingBox2DReadOnly boundingBox = getPreferredNonNavigableExtrusionsBoundingBox();

      if (extrusionSide == ExtrusionSide.INSIDE)
      {
         if (!boundingBox.isInsideEpsilon(query, 1e-7))
            return true;
         return preferredNonNavigableExtrusionsInLocal.stream().noneMatch(extrusion -> PlanarRegionTools.isPointInsidePolygon(extrusion.getPoints(), query));
      }
      else
      {
         if (!boundingBox.isInsideEpsilon(query, 1e-7))
            return false;
         return preferredNonNavigableExtrusionsInLocal.stream().anyMatch(extrusion -> PlanarRegionTools.isPointInsidePolygon(extrusion.getPoints(), query));
      }
   }

   public ExtrusionSide getExtrusionSide()
   {
      return extrusionSide;
   }

   public void setType(ClusterType type)
   {
      this.type = type;
   }

   public boolean isClosed()
   {
      return (type == ClusterType.POLYGON);
   }

   public ClusterType getType()
   {
      return type;
   }

   public void setTransformToWorld(RigidBodyTransformReadOnly transform)
   {
      //TODO: +++JerryPratt: Should never have to set the transform if we get it from the planar region. Right?
      transformToWorld.set(transform);
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return transformToWorld;
   }

   public int getNumberOfRawPoints()
   {
      return rawPointsInLocal3D.size();
   }

   public void addRawPointInLocal(Point2DReadOnly pointInLocal)
   {
      rawPointsInLocal3D.add(new Point3D(pointInLocal));
   }

   public void addRawPointInLocal(Point3DReadOnly pointInLocal)
   {
      rawPointsInLocal3D.add(new Point3D(pointInLocal));
   }

   public void addRawPointInWorld(Point3DReadOnly pointInWorld)
   {
      rawPointsInLocal3D.add(toLocal3D(pointInWorld));
   }

   public void addRawPointsInLocal2D(List<? extends Point2DReadOnly> pointsInLocal)
   {
      pointsInLocal.forEach(this::addRawPointInLocal);
   }

   public void addRawPointsInLocal3D(List<? extends Point3DReadOnly> pointsInLocal)
   {
      pointsInLocal.forEach(this::addRawPointInLocal);
   }

   public void addRawPointsInWorld(List<? extends Point3DReadOnly> pointsInWorld)
   {
      pointsInWorld.forEach(this::addRawPointInWorld);
   }

   public Point3DReadOnly getRawPointInLocal(int i)
   {
      return rawPointsInLocal3D.get(i);
   }

   public Point3DReadOnly getRawPointInWorld(int i)
   {
      return toWorld3D(rawPointsInLocal3D.get(i));
   }

   public List<Point3DReadOnly> getRawPointsInLocal3D()
   {
      return rawPointsInLocal3D;
   }

   public List<Point3DReadOnly> getRawPointsInWorld()
   {
      return rawPointsInLocal3D.stream().map(this::toWorld3D).collect(Collectors.toList());
   }

   public List<Point2DReadOnly> getRawPointsInWorld2D()
   {
      return rawPointsInLocal3D.stream().map(this::toWorld2D).collect(Collectors.toList());
   }

   public List<Point2DReadOnly> getRawPointsInLocal2D()
   {
      return rawPointsInLocal3D.stream().map(Point2D::new).collect(Collectors.toList());
   }

   public void addNavigableExtrusionInLocal(Point2DReadOnly navigableExtrusionInLocal)
   {
      navigableExtrusionsInLocal.addPoint(navigableExtrusionInLocal);
   }

   public void addNavigableExtrusionsInLocal(ExtrusionHull navigableExtrusionInLocal)
   {
      navigableExtrusionsInLocal.addAllPoints(navigableExtrusionInLocal);
   }

   public int getNumberOfNavigableExtrusions()
   {
      return navigableExtrusionsInLocal.size();
   }

   public Point2DReadOnly getNavigableExtrusionInLocal(int i)
   {
      return navigableExtrusionsInLocal.get(i);
   }

   public ExtrusionHull getNavigableExtrusionsInLocal()
   {
      return navigableExtrusionsInLocal;
   }

   public void setNavigableExtrusionsInLocal(ExtrusionHull points)
   {
      navigableExtrusionsInLocal.clear();
      addNavigableExtrusionsInLocal(points);
   }

   public List<Point3DReadOnly> getNavigableExtrusionsInWorld()
   {
      return navigableExtrusionsInLocal.stream().map(this::toWorld3D).collect(Collectors.toList());
   }

   public ExtrusionHull getNavigableExtrusionsInWorld2D()
   {
      return navigableExtrusionsInLocal.copy(extrusionHull -> extrusionHull.stream().map(this::toWorld2D).collect(Collectors.toList()));
   }


   public List<List<Point3DReadOnly>> getPreferredNavigableExtrusionsInWorld()
   {
      List<List<Point3DReadOnly>> extrusionsInWorld = new ArrayList<>();
      for (ExtrusionHull preferredExtrusionsInLocal : preferredNavigableExtrusionsInLocal)
         extrusionsInWorld.add(preferredExtrusionsInLocal.stream().map(this::toWorld3D).collect(Collectors.toList()));
      return extrusionsInWorld;
   }

   public void addPreferredNavigableExtrusionInLocal(ExtrusionHull navigableExtrusionInLocal)
   {
      preferredNavigableExtrusionsInLocal.add(navigableExtrusionInLocal.copy());
   }

   public void addPreferredNavigableExtrusionsInLocal(List<ExtrusionHull> navigableExtrusionInLocal)
   {
      navigableExtrusionInLocal.forEach(this::addPreferredNavigableExtrusionInLocal);
   }

   public List<ExtrusionHull> getPreferredNavigableExtrusionsInLocal()
   {
      return preferredNavigableExtrusionsInLocal;
   }

   public void setPreferredNavigableExtrusionsInLocal(List<ExtrusionHull> listsOfPoints)
   {
      preferredNavigableExtrusionsInLocal.clear();
      addPreferredNavigableExtrusionsInLocal(listsOfPoints);
   }

   public void setNonNavigableExtrusionsInLocal(ExtrusionHull points)
   {
      clearNonNavigableExtrusions();
      addNonNavigableExtrusionsInLocal(points);
   }

   private void addNonNavigableExtrusionInLocal(Point2DReadOnly nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionsBoundingBox.updateToIncludePoint(nonNavigableExtrusionInLocal);
      this.nonNavigableExtrusionInLocal.addPoint(nonNavigableExtrusionInLocal);
   }

   public void addNonNavigableExtrusionInLocal(Point3DReadOnly nonNavigableExtrusionInLocal)
   {
      addNonNavigableExtrusionInLocal(new Point2D(nonNavigableExtrusionInLocal));
   }

   public void addNonNavigableExtrusionsInLocal(ExtrusionHull nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionInLocal.stream().forEach(this::addNonNavigableExtrusionInLocal);
   }

   public BoundingBox2DReadOnly getNonNavigableExtrusionsBoundingBox()
   {
      return nonNavigableExtrusionsBoundingBox;
   }

   public int getNumberOfNonNavigableExtrusions()
   {
      return nonNavigableExtrusionInLocal.size();
   }

   public Point2DReadOnly getNonNavigableExtrusionInLocal(int i)
   {
      return nonNavigableExtrusionInLocal.get(i);
   }

   public Point3DReadOnly getNonNavigableExtrusionInWorld(int i)
   {
      return toWorld3D(getNonNavigableExtrusionInLocal(i));
   }

   public ExtrusionHull getNonNavigableExtrusionsInLocal()
   {
      return nonNavigableExtrusionInLocal;
   }

   public List<Point3DReadOnly> getNonNavigableExtrusionsInWorld()
   {
      return nonNavigableExtrusionInLocal.stream().map(this::toWorld3D).collect(Collectors.toList());
   }

   public ExtrusionHull getNonNavigableExtrusionsInWorld2D()
   {
      return nonNavigableExtrusionInLocal.copy(extrusionHull -> extrusionHull.stream().map(this::toWorld2D).collect(Collectors.toList()));
   }

   public void setPreferredNonNavigableExtrusionsInLocal(List<ExtrusionHull> points)
   {
      clearPreferredNonNavigableExtrusions();
      addPreferredNonNavigableExtrusionsInLocal(points);
   }

   public void addPreferredNonNavigableExtrusionInLocal(ExtrusionHull nonNavigableExtrusionInLocal)
   {
      ExtrusionHull extrusionCopy = nonNavigableExtrusionInLocal.copy();
      extrusionCopy.stream().forEach(preferredNonNavigableExtrusionsBoundingBox::updateToIncludePoint);
      preferredNonNavigableExtrusionsInLocal.add(extrusionCopy);
   }

   public void addPreferredNonNavigableExtrusionsInLocal(List<ExtrusionHull> nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionInLocal.forEach(this::addPreferredNonNavigableExtrusionInLocal);
   }

   public BoundingBox2DReadOnly getPreferredNonNavigableExtrusionsBoundingBox()
   {
      return preferredNonNavigableExtrusionsBoundingBox;
   }

   public List<ExtrusionHull> getPreferredNonNavigableExtrusionsInLocal()
   {
      return preferredNonNavigableExtrusionsInLocal;
   }

   public List<List<Point3DReadOnly>> getPreferredNonNavigableExtrusionsInWorld()
   {
      List<List<Point3DReadOnly>> listToReturn = new ArrayList<>();
      preferredNonNavigableExtrusionsInLocal.forEach(pointList -> listToReturn.add(pointList.stream().map(this::toWorld3D).collect(Collectors.toList())));
      return listToReturn;
   }



   private Point3DReadOnly toWorld3D(Point2DReadOnly pointInLocal)
   {
      return toWorld3D(new Point3D(pointInLocal));
   }

   private Point2DReadOnly toWorld2D(Point3DReadOnly pointInLocal)
   {
      Point2D pointInWorld = new Point2D(pointInLocal);
      transformToWorld.transform(pointInWorld, false);
      return pointInWorld;
   }

   private Point2DReadOnly toWorld2D(Point2DReadOnly pointInLocal)
   {
      Point2D pointInWorld = new Point2D(pointInLocal);
      transformToWorld.transform(pointInWorld, false);
      return pointInWorld;
   }

   private Point3DReadOnly toWorld3D(Point3DReadOnly pointInLocal)
   {
      Point3D pointInWorld = new Point3D(pointInLocal);
      transformToWorld.transform(pointInWorld);
      return pointInWorld;
   }

   private Point3DReadOnly toLocal3D(Point3DReadOnly pointInWorld)
   {
      Point3D pointInLocal = new Point3D();
      transformToWorld.inverseTransform(pointInWorld, pointInLocal);
      return pointInLocal;
   }

}
