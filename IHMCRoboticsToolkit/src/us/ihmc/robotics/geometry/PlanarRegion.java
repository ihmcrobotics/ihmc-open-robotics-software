package us.ihmc.robotics.geometry;

import java.util.List;

import javax.vecmath.AxisAngle4d;
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
    * @param pointOnPlane 
    * @param planeNormal
    * @param planarRegionConvexPolygons
    */
   public PlanarRegion(Point3d pointOnPlane, Vector3d planeNormal, List<ConvexPolygon2d> planarRegionConvexPolygons)
   {
      AxisAngle4d orientation = GeometryTools.getRotationBasedOnNormal(planeNormal);
      fromLocalToWorldTransform.setRotation(orientation);
      fromLocalToWorldTransform.setTranslation(pointOnPlane.getX(), pointOnPlane.getY(), pointOnPlane.getZ());
      fromWorldToLocalTransform.invert(fromLocalToWorldTransform);
      convexPolygons = planarRegionConvexPolygons;
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
      ConvexPolygon2d snappedPolygon = new ConvexPolygon2d();

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
         snappedPolygon.addVertex(snappedVertex3d.getX(), snappedVertex3d.getY());
      }
      snappedPolygon.update();

      ConvexPolygon2d dummyPolygon = new ConvexPolygon2d();

      // Now, just need to go through each polygon of this region and see there is at least one intersection
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         if (convexPolygons.get(i).intersectionWith(snappedPolygon, dummyPolygon))
            return true;
      }
      // Did not find any intersection
      return false;
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
    * @param epsilon tolerance expressed as maximum orthogonal distance from the region.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInside(Point3d point3dInWorld, double epsilon)
   {
      Point3d localPoint = new Point3d();
      fromWorldToLocalTransform.transform(point3dInWorld, localPoint);

      if (!MathTools.isInsideBoundsInclusive(localPoint.getZ(), epsilon))
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
}
