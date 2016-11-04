package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

public class PlanarRegionsList
{
   private final List<PlanarRegion> regions;

   public PlanarRegionsList(List<PlanarRegion> planarRegions)
   {
      regions = planarRegions;
   }

   /**
    * Find all the planar regions that intersect with the given convex polygon.
    * The algorithm is equivalent to projecting all the regions onto the XY-plane and then finding the regions intersecting with the given convex polygon.
    * @param convexPolygon the query.
    * @return the list of planar regions intersecting with the given polygon. Returns null when no region intersects.
    */
   public List<PlanarRegion> findPlanarRegionsIntersectingPolygon(ConvexPolygon2d convexPolygon)
   {
      List<PlanarRegion> containers = null;

      for (int i = 0; i < regions.size(); i++)
      {
         PlanarRegion candidateRegion = regions.get(i);
         if (candidateRegion.isPolygonIntersecting(convexPolygon))
         {
            if (containers == null)
               containers = new ArrayList<>();
            containers.add(candidateRegion);
         }
      }

      return containers;
   }

   /**
    * Find all the planar regions that contain the given point.
    * @param point the query coordinates.
    * @param epsilon tolerance expressed as maximum orthogonal distance from the region.
    * @return the list of planar regions containing the query. Returns null when no region contains the query.
    */
   public List<PlanarRegion> findPlanarRegionsContainingPoint(Point3d point, double epsilon)
   {
      List<PlanarRegion> containers = null;

      for (int i = 0; i < regions.size(); i++)
      {
         PlanarRegion candidateRegion = regions.get(i);
         if (candidateRegion.isPointInside(point, epsilon))
         {
            if (containers == null)
               containers = new ArrayList<>();
            containers.add(candidateRegion);
         }
      }

      return containers;
   }

   /**
    * Find all the planar regions that contain the given point.
    * The algorithm is equivalent to projecting all the regions onto the XY-plane and then finding the regions containing the point.
    * @param point the query coordinates.
    * @return the list of planar regions containing the query. Returns null when no region contains the query.
    */
   public List<PlanarRegion> findPlanarRegionsContainingPointByProjectionOntoXYPlane(Point2d point)
   {
      return findPlanarRegionsContainingPointByProjectionOntoXYPlane(point.getX(), point.getY());
   }

   /**
    * Find all the planar regions that contain the given point.
    * The algorithm is equivalent to projecting all the regions onto the XY-plane and then finding the regions containing the point.
    * @param x the query x-coordinate.
    * @param y the query y-coordinate.
    * @return the list of planar regions containing the query. Returns null when no region contains the query.
    */
   public List<PlanarRegion> findPlanarRegionsContainingPointByProjectionOntoXYPlane(double x, double y)
   {
      List<PlanarRegion> containers = null;

      for (int i = 0; i < regions.size(); i++)
      {
         PlanarRegion candidateRegion = regions.get(i);
         if (candidateRegion.isPointInsideByProjectionOntoXYPlane(x, y))
         {
            if (containers == null)
               containers = new ArrayList<>();
            containers.add(candidateRegion);
         }
      }

      return containers;
   }

   /** Return the number of planar regions contained in this list. */
   public int getNumberOfPlanarRegions()
   {
      return regions.size();
   }

   /** Retrieves the i<sup>th</sup> planar region of this list. */
   public PlanarRegion getPlanarRegion(int index)
   {
      return regions.get(index);
   }
}
