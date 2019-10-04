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
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;

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
   private final List<Point2DReadOnly> navigableExtrusionsInLocal = new ArrayList<>();
   private final List<Point2DReadOnly> nonNavigableExtrusionsInLocal = new ArrayList<>();

   private final BoundingBox2D nonNavigableExtrusionsBoundingBox = new BoundingBox2D(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

   private List<Point3DReadOnly> navigablePointsInsideHomeRegionInWorld = null;

   private static final double epsilonForMakingSurePointInsideHomeRegion = 1e-5; //Add a little to make sure we do not miss anything.

   public List<Point3DReadOnly> getNavigablePointsInsideHomeRegionInWorld(PlanarRegion homeRegion)
   {
      if (navigablePointsInsideHomeRegionInWorld == null)
      {
         navigablePointsInsideHomeRegionInWorld = new ArrayList<>();

         for (Point2DReadOnly point : navigableExtrusionsInLocal)
         {
            //TODO: PlanarRegionTools.isPointInLocalInsidePlanarRegion() was buggy. Make sure to unit test it well, especially if use concave hull...

            if (PlanarRegionTools.isPointInLocalInsidePlanarRegion(homeRegion, point, epsilonForMakingSurePointInsideHomeRegion))
            {
               Point3D point3D = toWorld3D(point);
               navigablePointsInsideHomeRegionInWorld.add(point3D);
            }
         }
      }

      return navigablePointsInsideHomeRegionInWorld;
   }

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
      nonNavigableExtrusionsInLocal.clear();
   }

   public boolean isInsideNonNavigableZone(Point2DReadOnly query)
   {
      if (nonNavigableExtrusionsInLocal.isEmpty())
         return false;

      BoundingBox2D boundingBox = getNonNavigableExtrusionsBoundingBox();

      if (extrusionSide == ExtrusionSide.INSIDE)
      {
         if (!boundingBox.isInsideInclusive(query))
            return true;
         return !PlanarRegionTools.isPointInsidePolygon(nonNavigableExtrusionsInLocal, query);
      }
      else
      {
         if (!boundingBox.isInsideInclusive(query))
            return false;
         return PlanarRegionTools.isPointInsidePolygon(nonNavigableExtrusionsInLocal, query);
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

   public void setTransformToWorld(RigidBodyTransform transform)
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

   public void addRawPointsInLocal2D(Point2DReadOnly[] pointsInLocal)
   {
      addRawPointsInLocal2D(Arrays.asList(pointsInLocal));
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
      navigableExtrusionsInLocal.add(new Point2D(navigableExtrusionInLocal));
   }

   public void addNavigableExtrusionInLocal(Point3DReadOnly navigableExtrusionInLocal)
   {
      addNavigableExtrusionInLocal(new Point2D(navigableExtrusionInLocal));
   }

   public void addNavigableExtrusionsInLocal(List<? extends Point2DReadOnly> navigableExtrusionInLocal)
   {
      navigableExtrusionInLocal.forEach(this::addNavigableExtrusionInLocal);
   }

   public int getNumberOfNavigableExtrusions()
   {
      return navigableExtrusionsInLocal.size();
   }

   public Point2DReadOnly getNavigableExtrusionInLocal(int i)
   {
      return navigableExtrusionsInLocal.get(i);
   }

   public Point3DReadOnly getNavigableExtrusionInWorld(int i)
   {
      return toWorld3D(getNavigableExtrusionInLocal(i));
   }

   public List<Point2DReadOnly> getNavigableExtrusionsInLocal()
   {
      return navigableExtrusionsInLocal;
   }

   public void setNavigableExtrusionsInLocal(List<Point2DReadOnly> points)
   {
      navigableExtrusionsInLocal.clear();
      navigableExtrusionsInLocal.addAll(points);

      //TODO: Need to set to null in more places when this happens. Or reduce the number of calls available. Maybe make immuatable?
      navigablePointsInsideHomeRegionInWorld = null;
   }

   public void setNonNavigableExtrusionsInLocal(List<Point2DReadOnly> points)
   {
      clearNonNavigableExtrusions();
      addNonNavigableExtrusionsInLocal(points);
   }

   public List<Point3DReadOnly> getNavigableExtrusionsInWorld()
   {
      return navigableExtrusionsInLocal.stream().map(this::toWorld3D).collect(Collectors.toList());
   }

   public List<Point2DReadOnly> getNavigableExtrusionsInWorld2D()
   {
      return navigableExtrusionsInLocal.stream().map(this::toWorld2D).collect(Collectors.toList());
   }

   public void addNonNavigableExtrusionInLocal(Point2DReadOnly nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionsBoundingBox.updateToIncludePoint(nonNavigableExtrusionInLocal);
      nonNavigableExtrusionsInLocal.add(new Point2D(nonNavigableExtrusionInLocal));
   }

   public void addNonNavigableExtrusionInLocal(Point3DReadOnly nonNavigableExtrusionInLocal)
   {
      addNonNavigableExtrusionInLocal(new Point2D(nonNavigableExtrusionInLocal));
   }

   public void addNonNavigableExtrusionsInLocal(List<? extends Point2DReadOnly> nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionInLocal.forEach(this::addNonNavigableExtrusionInLocal);
   }

   public BoundingBox2D getNonNavigableExtrusionsBoundingBox()
   {
      return nonNavigableExtrusionsBoundingBox;
   }

   public int getNumberOfNonNavigableExtrusions()
   {
      return nonNavigableExtrusionsInLocal.size();
   }

   public Point2DReadOnly getNonNavigableExtrusionInLocal(int i)
   {
      return nonNavigableExtrusionsInLocal.get(i);
   }

   public Point3DReadOnly getNonNavigableExtrusionInWorld(int i)
   {
      return toWorld3D(getNonNavigableExtrusionInLocal(i));
   }

   public List<Point2DReadOnly> getNonNavigableExtrusionsInLocal()
   {
      return nonNavigableExtrusionsInLocal;
   }

   public List<Point3DReadOnly> getNonNavigableExtrusionsInWorld()
   {
      return nonNavigableExtrusionsInLocal.stream().map(this::toWorld3D).collect(Collectors.toList());
   }

   public List<Point2DReadOnly> getNonNavigableExtrusionsInWorld2D()
   {
      return nonNavigableExtrusionsInLocal.stream().map(this::toWorld2D).collect(Collectors.toList());
   }

   private Point3D toWorld3D(Point2DReadOnly pointInLocal)
   {
      return toWorld3D(new Point3D(pointInLocal));
   }

   private Point2D toWorld2D(Point3DReadOnly pointInLocal)
   {
      Point2D pointInWorld = new Point2D(pointInLocal);
      transformToWorld.transform(pointInWorld, false);
      return pointInWorld;
   }

   private Point2D toWorld2D(Point2DReadOnly pointInLocal)
   {
      Point2D pointInWorld = new Point2D(pointInLocal);
      transformToWorld.transform(pointInWorld, false);
      return pointInWorld;
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

}
