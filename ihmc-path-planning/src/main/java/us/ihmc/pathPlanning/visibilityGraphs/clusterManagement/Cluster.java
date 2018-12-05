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
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * A Cluster typically represents an obstacle over a planar region.
 * It consists of the 3D points of the obstacle, the 2D points of the navigable spots on the planar region 
 * and the 2D points of the nonNavigable spots on the planar region resulting from the obstacle.
 * It contains the RigidBodyTransform for moving the points from local to world coordinates.
 * 
 * There is one cluster per planar region per obstacle.
 */
public class Cluster
{
   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   //TODO: +++ This seems to assume that a bunch of points are one closed region if POLYGON. But what if the planar region it is created from
   // has several independent areas? Perhaps we need a flag or guarantee or something that we have only 1 closed polygon per planar region used?

   private final List<Point3DReadOnly> rawPointsLocal = new ArrayList<>();
   private final List<Point2DReadOnly> navigableExtrusionsInLocal = new ArrayList<>();
   private final List<Point2DReadOnly> nonNavigableExtrusionsInLocal = new ArrayList<>();

   private boolean nonNavigableExtrusionsBoundingBoxIsDirty = true;
   private final BoundingBox2D nonNavigableExtrusionsBoundingBox = new BoundingBox2D();

   private List<Point3DReadOnly> navigablePointsInsideHomeRegionInWorld = null;

   private static final double epsilonForMakingSurePointInsideHomeRegion = 1e-5; //Add a little to make sure we do not miss anything.

   public List<Point3DReadOnly> getNavigablePointsInsideHomeRegionInWorld(PlanarRegion homeRegion)
   {
      //TODO: +++JEP: Finish.
      //      return navigableExtrusionsInLocal.stream().map(this::toWorld3D).collect(Collectors.toList());

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

   private ExtrusionSide extrusionSide = ExtrusionSide.OUTSIDE;

   public enum Type
   {
      // Multi-Line means open at the end, not closed. 
      // Polygon is a closed, perhaps concave, polygon.
      LINE, MULTI_LINE, POLYGON;

      public static Type[] values = values();

      public byte toByte()
      {
         return (byte) ordinal();
      }

      public static Type fromByte(byte enumAsByte)
      {
         if (enumAsByte == -1)
            return null;
         return values[enumAsByte];
      }
   };

   private Type type = Type.POLYGON;

   public Cluster()
   {
   }

   public void updateBoundingBox()
   {
      //TODO: First reset the bounding box to zero or null or something?
      nonNavigableExtrusionsInLocal.forEach(nonNavigableExtrusionsBoundingBox::updateToIncludePoint);
      nonNavigableExtrusionsBoundingBoxIsDirty = false;
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
      //TODO: +++JEP: Should never have to set the transform if we get it from the planar region. Right?
      transformToWorld.set(transform);
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return transformToWorld;
   }

   public int getNumberOfRawPoints()
   {
      return rawPointsLocal.size();
   }

   public void addRawPointInLocal(Point2DReadOnly pointInLocal)
   {
      rawPointsLocal.add(new Point3D(pointInLocal));
   }

   public void addRawPointInLocal(Point3DReadOnly pointInLocal)
   {
      rawPointsLocal.add(new Point3D(pointInLocal));
   }

   public void addRawPointInWorld(Point3DReadOnly pointInWorld)
   {
      rawPointsLocal.add(toLocal3D(pointInWorld));
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
      return rawPointsLocal.get(i);
   }

   public Point3DReadOnly getRawPointInWorld(int i)
   {
      return toWorld3D(rawPointsLocal.get(i));
   }

   public List<Point3DReadOnly> getRawPointsInLocal3D()
   {
      return rawPointsLocal;
   }

   public List<Point3DReadOnly> getRawPointsInWorld()
   {
      return rawPointsLocal.stream().map(this::toWorld3D).collect(Collectors.toList());
   }

   public List<Point2DReadOnly> getRawPointsInLocal2D()
   {
      return rawPointsLocal.stream().map(Point2D::new).collect(Collectors.toList());
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

   public List<Point3DReadOnly> getNavigableExtrusionsInWorld()
   {
      return navigableExtrusionsInLocal.stream().map(this::toWorld3D).collect(Collectors.toList());
   }

   public void addNonNavigableExtrusionInLocal(Point2DReadOnly nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionsInLocal.add(new Point2D(nonNavigableExtrusionInLocal));
      nonNavigableExtrusionsBoundingBoxIsDirty = true;
   }

   public void addNonNavigableExtrusionInLocal(Point3DReadOnly nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionsInLocal.add(new Point2D(nonNavigableExtrusionInLocal));
      nonNavigableExtrusionsBoundingBoxIsDirty = true;
   }

   public void addNonNavigableExtrusionsInLocal(List<? extends Point2DReadOnly> nonNavigableExtrusionInLocal)
   {
      nonNavigableExtrusionInLocal.forEach(this::addNonNavigableExtrusionInLocal);
      nonNavigableExtrusionsBoundingBoxIsDirty = true;
   }

   public BoundingBox2D getNonNavigableExtrusionsBoundingBox()
   {
      if (nonNavigableExtrusionsBoundingBoxIsDirty)
         updateBoundingBox();

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
