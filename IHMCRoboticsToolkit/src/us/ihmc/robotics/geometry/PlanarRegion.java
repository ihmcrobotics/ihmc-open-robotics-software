package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.MathTools;

public class PlanarRegion
{
   private final RigidBodyTransform fromLocalToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform fromWorldToLocalTransform = new RigidBodyTransform();
   /**
    * List of the convex polygons representing this planar region.
    * They are in the local frame of the plane.
    */
   private final List<ConvexPolygon2d> convexPolygons;

   /**
    * Create a new planar region.
    * @param transformToWorld transform from the region local coordinate system to world.
    * @param planarRegionConvexPolygons the list of convex polygon that represents the planar region. Expressed in local coordinate system.
    */
   public PlanarRegion(RigidBodyTransform transformToWorld, List<ConvexPolygon2d> planarRegionConvexPolygons)
   {
      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.invert(fromLocalToWorldTransform);
      convexPolygons = planarRegionConvexPolygons;
   }

   /**
    * Create a new planar region.
    * @param transformToWorld transform from the region local coordinate system to world.
    * @param convexPolygon a single convex polygon that represents the planar region. Expressed in local coordinate system.
    */
   public PlanarRegion(RigidBodyTransform transformToWorld, ConvexPolygon2d convexPolygon)
   {
      convexPolygons = new ArrayList<>();
      convexPolygons.add(convexPolygon);
      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.invert(fromLocalToWorldTransform);
   }

   /**
    * Verify if the given polygon intersects this region projected onto the XY-plane.
    * @param convexPolygon2d
    * @return
    */
   public boolean isPolygonIntersecting(ConvexPolygon2d convexPolygon2d)
   {
      // Instead of projecting all the polygons of this region onto the world XY-plane,
      // the given convex polygon is projected along the z-world axis to be snapped onto plane.
      ConvexPolygon2d projectedPolygon = projectPolygonVerticallyToRegion(convexPolygon2d);
      ConvexPolygon2d dummyPolygon = new ConvexPolygon2d();

      // Now, just need to go through each polygon of this region and see there is at least one intersection
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         if (convexPolygons.get(i).intersectionWith(projectedPolygon, dummyPolygon))
            return true;
      }
      // Did not find any intersection
      return false;
   }

   /**
    * Returns all of the intersections when the convexPolygon is projected vertically onto this PlanarRegion.
    * @param convexPolygon2d Polygon to project vertically.
    * @param intersectionsToPack ArrayList of ConvexPolygon2d to pack with the intersections.
    */
   public void getPolygonIntersections(ConvexPolygon2d convexPolygon2d, ArrayList<ConvexPolygon2d> intersectionsToPack)
   {
      // Instead of projecting all the polygons of this region onto the world XY-plane,
      // the given convex polygon is projected along the z-world axis to be snapped onto plane.
      ConvexPolygon2d projectedPolygon = projectPolygonVerticallyToRegion(convexPolygon2d);

      // Now, just need to go through each polygon of this region and see there is at least one intersection
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2d intersectingPolygon = convexPolygons.get(i).intersectionWith(projectedPolygon);

         if (intersectingPolygon != null)
         {
            intersectionsToPack.add(intersectingPolygon);
         }
      }
   }

   /**
    * Projects the input ConvexPolygon2d to the plane defined by this PlanarRegion by translating each vertex in world z.
    * Then puts each vertex in local frame. In doing so, the area of the rotated polygon will actually increase on tilted PlanarRegions.
    * @param convexPolygon2d Polygon to project
    * @return new projected ConvexPolygon2d
    */
   private ConvexPolygon2d projectPolygonVerticallyToRegion(ConvexPolygon2d convexPolygon2d)
   {
      ConvexPolygon2d projectedPolygon = new ConvexPolygon2d();

      Point3d snappedVertex3d = new Point3d();

      for (int i = 0; i < convexPolygon2d.getNumberOfVertices(); i++)
      {
         Point2d originalVertex = convexPolygon2d.getVertex(i);
         // Find the vertex 3d that is snapped to the plane following z-world.
         snappedVertex3d.setX(originalVertex.getX());
         snappedVertex3d.setY(originalVertex.getY());
         snappedVertex3d.setZ(getPlaneZGivenXY(originalVertex.getX(), originalVertex.getY()));

         // Transform to local coordinates
         fromWorldToLocalTransform.transform(snappedVertex3d);
         // Add the snapped vertex to the snapped polygon
         projectedPolygon.addVertex(snappedVertex3d.getX(), snappedVertex3d.getY());
      }
      projectedPolygon.update();
      return projectedPolygon;
   }

   /**
    * Computes if the point is in the region projected onto the world xy-plane.
    * Note that the z-coordinate of the query is ignored.
    * @param point3d query coordinates.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInsideByProjectionOntoXYPlane(Point3d point3d)
   {
      return isPointInsideByProjectionOntoXYPlane(point3d.getX(), point3d.getY());
   }

   /**
    * Computes if the point is in the region projected onto the world xy-plane.
    * @param point2d query coordinates.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInsideByProjectionOntoXYPlane(Point2d point2d)
   {
      return isPointInsideByProjectionOntoXYPlane(point2d.getX(), point2d.getY());
   }

   /**
    * Computes if the point is in the region projected onto the world xy-plane.
    * @param x x-coordinate of the query.
    * @param y y-coordinate of the query.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInsideByProjectionOntoXYPlane(double x, double y)
   {
      Point3d localPoint = new Point3d();
      localPoint.setX(x);
      localPoint.setY(y);
      localPoint.setZ(getPlaneZGivenXY(x, y));

      fromWorldToLocalTransform.transform(localPoint);

      return isPointInside(new Point2d(localPoint.getX(), localPoint.getY()));
   }

   /**
    * Given a 3D point in world coordinates, computes whether the point is in this region.
    * @param point3dInWorld query expressed in world coordinates.
    * @param maximumOrthogonalDistance tolerance expressed as maximum orthogonal distance from the region.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInside(Point3d point3dInWorld, double maximumOrthogonalDistance)
   {
      Point3d localPoint = new Point3d();
      fromWorldToLocalTransform.transform(point3dInWorld, localPoint);

      if (!MathTools.isInsideBoundsInclusive(localPoint.getZ(), maximumOrthogonalDistance))
         return false;
      else
         return isPointInside(new Point2d(localPoint.getX(), localPoint.getY()));
   }

   /**
    * Given a 2D point expressed in the plane local frame, computes whether the point is in this region.
    * @param point2dInLocal query expressed in local coordinates.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInside(Point2d point2dInLocal)
   {
      for (int i = 0; i < convexPolygons.size(); i++)
      {
         if (convexPolygons.get(i).isPointInside(point2dInLocal))
            return true;
      }
      return false;
   }

   /**
    * Computes the z-coordinate in world of the plane for a given xy-coordinates in world.
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

   /** Returns the number of convex polygons representing this region. */
   public int getNumberOfConvexPolygons()
   {
      return convexPolygons.size();
   }

   /**
    * Returns the i<sup>th</sup> convex polygon representing a portion of this region.
    * The polygon is expressed in the region local coordinates.
    */
   public ConvexPolygon2d getConvexPolygon(int i)
   {
      return convexPolygons.get(i);
   }

   /**
    * Retrieves the normal of this planar region and stores it in the given {@link Vector3d}.
    * @param normalToPack used to store the normal of this planar region.
    */
   public void getNormal(Vector3d normalToPack)
   {
      normalToPack.setX(fromLocalToWorldTransform.getM02());
      normalToPack.setY(fromLocalToWorldTransform.getM12());
      normalToPack.setZ(fromLocalToWorldTransform.getM22());
   }

   /**
    * Retrieves a point that lies in this planar region.
    * This point is also used as the origin of the local coordinate system of this planar region.
    * @param pointToPack used to store the point coordinates.
    */
   public void getPointInRegion(Point3d pointToPack)
   {
      fromLocalToWorldTransform.getTranslation(pointToPack);
   }

   /**
    * Get the transform from local coordinates to world coordinates.
    * @param transformToPack used to store the transform.
    */
   public void getTransformToWorld(RigidBodyTransform transformToPack)
   {
      transformToPack.set(fromLocalToWorldTransform);
   }

   public boolean epsilonEquals(PlanarRegion other, double epsilon)
   {
      if (!fromLocalToWorldTransform.epsilonEquals(other.fromLocalToWorldTransform, epsilon))
         return false;
      // Not necessary, but just in case
      if (!fromWorldToLocalTransform.epsilonEquals(other.fromWorldToLocalTransform, epsilon))
         return false;

      if (getNumberOfConvexPolygons() != other.getNumberOfConvexPolygons())
         return false;

      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         if (!convexPolygons.get(i).epsilonEquals(other.convexPolygons.get(i), epsilon))
            return false;
      }
      return true;
   }
}
