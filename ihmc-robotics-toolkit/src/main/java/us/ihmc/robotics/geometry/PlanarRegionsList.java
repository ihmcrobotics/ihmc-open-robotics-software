package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class PlanarRegionsList
{
   private final List<PlanarRegion> regions;

   public PlanarRegionsList(PlanarRegion... planarRegions)
   {
      regions = new ArrayList<>();
      for (PlanarRegion planarRegion : planarRegions)
      {
         regions.add(planarRegion);
      }
   }

   public PlanarRegionsList(List<PlanarRegion> planarRegions)
   {
      regions = planarRegions;
   }

   /**
    * Adds a planar region to this list of planar regions.
    * 
    * @param region to add.
    */
   public void addPlanarRegion(PlanarRegion region)
   {
      regions.add(region);
   }

   /**
    * Clears the planar regions list.
    */
   public void clear()
   {
      regions.clear();
   }

   /**
    * Find all the planar regions that intersect with the given convex polygon. The algorithm is
    * equivalent to projecting all the regions onto the XY-plane and then finding the regions
    * intersecting with the given convex polygon.
    * 
    * @param convexPolygon the query.
    * @return the list of planar regions intersecting with the given polygon. Returns null when no
    *         region intersects.
    */
   public List<PlanarRegion> findPlanarRegionsIntersectingPolygon(ConvexPolygon2DReadOnly convexPolygon)
   {
      List<PlanarRegion> containers = null;

      for (int i = 0; i < regions.size(); i++)
      {
         PlanarRegion candidateRegion = regions.get(i);
         if (candidateRegion.isVertical())
            continue;

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
    * Find all the planar regions that intersect with the given 2d line segment. The algorithm is
    * equivalent to projecting all the regions onto the XY-plane and then finding the regions
    * intersecting with the given line segment.
    * 
    * @param lineSegmentInWorld the query.
    * @param intersectingRegionsToPack ArrayList were the intersecting regions will be packed into.
    */
   public void findPlanarRegionsIntersectingLineSegment(LineSegment2D lineSegmentInWorld, ArrayList<PlanarRegion> intersectingRegionsToPack)
   {
      for (int i = 0; i < regions.size(); i++)
      {
         PlanarRegion candidateRegion = regions.get(i);
         if (isLineSegmentObviouslyOutsideBoundingBox(candidateRegion, lineSegmentInWorld))
            continue;

         if (candidateRegion.isVertical())
            continue;

         if (candidateRegion.isLineSegmentIntersecting(lineSegmentInWorld))
         {
            intersectingRegionsToPack.add(candidateRegion);
         }
      }
   }

   /**
    * Returns true if lineSegment is Obviously Outside BoundingBox. If returns true, then definitely
    * outside. If returns false, might still be outside. If intersects, will always return false.
    * 
    * @param candidateRegion
    * @param lineSegmentInWorld
    * @return
    */
   private boolean isLineSegmentObviouslyOutsideBoundingBox(PlanarRegion candidateRegion, LineSegment2D lineSegmentInWorld)
   {
      BoundingBox3D boundingBox = candidateRegion.getBoundingBox3dInWorld();

      double xMin = boundingBox.getMinX();
      double yMin = boundingBox.getMinY();
      double xMax = boundingBox.getMaxX();
      double yMax = boundingBox.getMaxY();

      Point2DReadOnly firstEndpoint = lineSegmentInWorld.getFirstEndpoint();
      Point2DReadOnly secondEndpoint = lineSegmentInWorld.getSecondEndpoint();

      if ((firstEndpoint.getX() < xMin) && (secondEndpoint.getX() < xMin))
         return true;
      if ((firstEndpoint.getX() > xMax) && (secondEndpoint.getX() > xMax))
         return true;
      if ((firstEndpoint.getY() < yMin) && (secondEndpoint.getY() < yMin))
         return true;
      if ((firstEndpoint.getY() > yMax) && (secondEndpoint.getY() > yMax))
         return true;

      return false;
   }

   /**
    * Find all the planar regions that contain the given point.
    * 
    * @param point the query coordinates.
    * @param maximumOrthogonalDistance tolerance expressed as maximum orthogonal distance from the
    *           region.
    * @return the list of planar regions containing the query. Returns null when no region contains
    *         the query.
    */
   public List<PlanarRegion> findPlanarRegionsContainingPoint(Point3DReadOnly point, double maximumOrthogonalDistance)
   {
      List<PlanarRegion> containers = null;

      for (int i = 0; i < regions.size(); i++)
      {
         PlanarRegion candidateRegion = regions.get(i);
         if (candidateRegion.isPointInside(point, maximumOrthogonalDistance))
         {
            if (containers == null)
               containers = new ArrayList<>();
            containers.add(candidateRegion);
         }
      }

      return containers;
   }

   /**
    * Find all the planar regions that contain the given point. The algorithm is equivalent to
    * projecting all the regions onto the XY-plane and then finding the regions containing the
    * point.
    * 
    * @param point the query coordinates.
    * @return the list of planar regions containing the query. Returns null when no region contains
    *         the query.
    */
   public List<PlanarRegion> findPlanarRegionsContainingPointByProjectionOntoXYPlane(Point2D point)
   {
      return findPlanarRegionsContainingPointByProjectionOntoXYPlane(point.getX(), point.getY());
   }

   /**
    * Find all the planar regions that contain the given point. The algorithm is equivalent to
    * projecting all the regions onto the XY-plane and then finding the regions containing the
    * point.
    * 
    * @param x the query x-coordinate.
    * @param y the query y-coordinate.
    * @return the list of planar regions containing the query. Returns null when no region contains
    *         the query.
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

   /** Returns true if this list of planar regions is empty (contains no planar regions). */
   public boolean isEmpty()
   {
      return regions.isEmpty();
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

   /** Retrieves the planar regions as a {@code List}. */
   public List<PlanarRegion> getPlanarRegionsAsList()
   {
      return regions;
   }

   /**
    * Retrieves the last planar region of this list. Special case: returns null when the list is
    * empty.
    */
   public PlanarRegion getLastPlanarRegion()
   {
      if (isEmpty())
         return null;
      else
         return getPlanarRegion(getNumberOfPlanarRegions() - 1);
   }

   /** Retrieves the i<sup>th</sup> planar region of this list and removes it from this list. */
   public PlanarRegion pollPlanarRegion(int index)
   {
      return regions.remove(index);
   }

   /**
    * Retrieves the last planar region of this list and removes it from this list. Special case:
    * returns null when the list is empty.
    */
   public PlanarRegion pollLastPlanarRegion()
   {
      if (isEmpty())
         return null;
      else
         return pollPlanarRegion(getNumberOfPlanarRegions() - 1);
   }

   /**
    * @return a full depth copy of this list of planar regions. The copy can be entirely modified
    *         without interfering with this.
    */
   public PlanarRegionsList copy()
   {
      List<PlanarRegion> planarRegionsCopy = new ArrayList<>();

      for (int i = 0; i < getNumberOfPlanarRegions(); i++)
         planarRegionsCopy.add(regions.get(i).copy());

      return new PlanarRegionsList(planarRegionsCopy);
   }

   /**
    * Transforms the planar regions list
    * 
    * @param rigidBodyTransform transform from current frame to desired frame
    */
   public void transform(RigidBodyTransform rigidBodyTransform)
   {
      for (int i = 0; i < regions.size(); i++)
      {
         regions.get(i).transform(rigidBodyTransform);
      }
   }
}
