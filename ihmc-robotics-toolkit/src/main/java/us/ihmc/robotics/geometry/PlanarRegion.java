package us.ihmc.robotics.geometry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.random.RandomGeometry;

public class PlanarRegion implements SupportingVertexHolder
{
   public static final int NO_REGION_ID = -1;
   public static final double DEFAULT_BOUNDING_BOX_EPSILON = 0.0;

   private int regionId = NO_REGION_ID;
   private final RigidBodyTransform fromLocalToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform fromWorldToLocalTransform = new RigidBodyTransform();
   private final Point2D[] concaveHullsVertices;
   /**
    * List of the convex polygons representing this planar region. They are in the local frame of
    * the plane.
    */
   private final List<ConvexPolygon2D> convexPolygons;

   private final BoundingBox3D boundingBox3dInWorld = new BoundingBox3D(new Point3D(Double.NaN, Double.NaN, Double.NaN),
                                                                        new Point3D(Double.NaN, Double.NaN, Double.NaN));
   private double boundingBoxEpsilon = DEFAULT_BOUNDING_BOX_EPSILON;
   private final Point3D tempPointForConvexPolygonProjection = new Point3D();

   private final ConvexPolygon2D convexHull = new ConvexPolygon2D();

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   /**
    * Create a new, empty planar region.
    */
   public PlanarRegion()
   {
      concaveHullsVertices = new Point2D[0];
      convexPolygons = new ArrayList<>();
      updateBoundingBox();
      updateConvexHull();
   }

   /**
    * Create a new planar region.
    *
    * @param transformToWorld transform from the region local coordinate system to world.
    * @param planarRegionConvexPolygons the list of convex polygon that represents the planar
    *           region. Expressed in local coordinate system.
    */
   public PlanarRegion(RigidBodyTransform transformToWorld, List<ConvexPolygon2D> planarRegionConvexPolygons)
   {
      PrintTools.warn("This constructor does not set the concave hull.");

      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);
      concaveHullsVertices = new Point2D[0];
      convexPolygons = planarRegionConvexPolygons;
      updateBoundingBox();
      updateConvexHull();
   }

   /**
    * Create a new planar region.
    *
    * @param transformToWorld transform from the region local coordinate system to world.
    * @param concaveHullVertices vertices of the concave hull of the region.
    * @param planarRegionConvexPolygons the list of convex polygon that represents the planar
    *           region. Expressed in local coordinate system.
    */
   public PlanarRegion(RigidBodyTransform transformToWorld, Point2D[] concaveHullVertices, List<ConvexPolygon2D> planarRegionConvexPolygons)
   {
      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);
      this.concaveHullsVertices = concaveHullVertices;
      //TODO: Remove repeat vertices if you have them, or fix upstream so they don't create them.
      checkConcaveHullRepeatVertices(false);

      convexPolygons = planarRegionConvexPolygons;
      updateBoundingBox();
      updateConvexHull();
   }

   /**
    * Create a new planar region.
    *
    * @param transformToWorld transform from the region local coordinate system to world.
    * @param convexPolygon a single convex polygon that represents the planar region. Expressed in
    *           local coordinate system.
    */
   public PlanarRegion(RigidBodyTransform transformToWorld, ConvexPolygon2D convexPolygon)
   {
      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);
      concaveHullsVertices = new Point2D[convexPolygon.getNumberOfVertices()];
      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         concaveHullsVertices[i] = new Point2D(convexPolygon.getVertex(i));
      }
      checkConcaveHullRepeatVertices(false);

      convexPolygons = new ArrayList<>();
      convexPolygons.add(convexPolygon);
      updateBoundingBox();
      updateConvexHull();
   }

   public void set(RigidBodyTransform transformToWorld, List<ConvexPolygon2D> planarRegionConvexPolygons)
   {
      this.set(transformToWorld, planarRegionConvexPolygons, NO_REGION_ID);
   }

   public void set(RigidBodyTransform transformToWorld, List<ConvexPolygon2D> planarRegionConvexPolygons, int newRegionId)
   {
      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);

      convexPolygons.clear();
      convexPolygons.addAll(planarRegionConvexPolygons);

      updateBoundingBox();
      updateConvexHull();

      regionId = newRegionId;
   }

   /**
    * Check if the given lineSegment intersects this region projected onto the XY-plane.
    *
    * @param lineSegmentInWorld
    * @return true if the lineSegment intersects this PlanarRegion.
    */
   public boolean isLineSegmentIntersecting(LineSegment2DReadOnly lineSegmentInWorld)
   {
      // Instead of projecting all the polygons of this region onto the world XY-plane,
      // the given lineSegment is projected along the z-world axis to be snapped onto plane.
      LineSegment2D projectedLineSegment = projectLineSegmentVerticallyToRegion(lineSegmentInWorld);

      // Now, just need to go through each polygon of this region and see there is at least one intersection
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2D polygonToCheck = convexPolygons.get(i);
         Point2DBasics[] intersectionPoints = polygonToCheck.intersectionWith(projectedLineSegment);
         if ((intersectionPoints != null) && (intersectionPoints.length > 0) && (intersectionPoints[0] != null))
            return true;
      }
      // Did not find any intersection
      return false;
   }

   /**
    * Returns all of the intersections when the convexPolygon is projected vertically onto this
    * PlanarRegion.
    *
    * @param lineSegmentInWorld Line segment to project vertically.
    * @param intersectionsInPlaneFrameToPack ArrayList of ConvexPolygon2d to pack with the
    *           intersections.
    */
   public void getLineSegmentIntersectionsWhenProjectedVertically(LineSegment2D lineSegmentInWorld, List<Point2DBasics[]> intersectionsInPlaneFrameToPack)
   {
      // Instead of projecting all the polygons of this region onto the world XY-plane,
      // the given lineSegment is projected along the z-world axis to be snapped onto plane.
      LineSegment2D projectedLineSegment = projectLineSegmentVerticallyToRegion(lineSegmentInWorld);

      // Now, just need to go through each polygon of this region and see there is at least one intersection
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         Point2DBasics[] intersectionPoints = convexPolygons.get(i).intersectionWith(projectedLineSegment);

         if ((intersectionPoints != null) && (intersectionPoints.length > 0) && (intersectionPoints[0] != null))
         {
            intersectionsInPlaneFrameToPack.add(intersectionPoints);
         }
      }
   }

   /**
    * Check if the given polygon intersects this region projected onto the XY-plane.
    *
    * @param convexPolygon2d
    * @return true if the polygon intersects this PlanarRegion.
    */
   public boolean isPolygonIntersecting(ConvexPolygon2DReadOnly convexPolygon2d)
   {
      BoundingBox2DReadOnly polygonBoundingBox = convexPolygon2d.getBoundingBox();
      if (!boundingBox3dInWorld.intersectsInclusiveInXYPlane(polygonBoundingBox))
         return false;

      // Instead of projecting all the polygons of this region onto the world XY-plane,
      // the given convex polygon is projected along the z-world axis to be snapped onto plane.
      ConvexPolygon2D projectedPolygon = projectPolygonVerticallyToRegion(convexPolygon2d);
      ConvexPolygon2D dummyPolygon = new ConvexPolygon2D();

      // Now, just need to go through each polygon of this region and see there is at least one intersection
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2D polygonToCheck = convexPolygons.get(i);
         boolean hasIntersection = convexPolygonTools.computeIntersectionOfPolygons(polygonToCheck, projectedPolygon, dummyPolygon);
         if (hasIntersection)
            return true;
      }
      // Did not find any intersection
      return false;
   }

   /**
    * Returns all of the intersections when the convexPolygon is projected vertically onto this
    * PlanarRegion.
    *
    * @param convexPolygon2DBasics Polygon to project vertically.
    * @param intersectionsInPlaneFrameToPack ArrayList of ConvexPolygon2d to pack with the
    *           intersections.
    */
   public void getPolygonIntersectionsWhenProjectedVertically(ConvexPolygon2DReadOnly convexPolygon2DBasics,
                                                              ArrayList<ConvexPolygon2D> intersectionsInPlaneFrameToPack)
   {
      // Instead of projecting all the polygons of this region onto the world XY-plane,
      // the given convex polygon is projected along the z-world axis to be snapped onto plane.
      ConvexPolygon2D projectedPolygon = projectPolygonVerticallyToRegion(convexPolygon2DBasics);

      // Now, just need to go through each polygon of this region and see there is at least one intersection
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2D intersectingPolygon = new ConvexPolygon2D();
         if (convexPolygonTools.computeIntersectionOfPolygons(convexPolygons.get(i), projectedPolygon, intersectingPolygon))
         {
            intersectionsInPlaneFrameToPack.add(intersectingPolygon);
         }
      }
   }

   /**
    * Returns all of the intersections when the convexPolygon is snapped onto this PlanarRegion with
    * the snappingTransform.
    *
    * @param convexPolygon2d Polygon to snap.
    * @param snappingTransform RigidBodyTransform that snaps the polygon onto this region. Must have
    *           same surface normal as this region.
    * @return intersectionArea Total area of intersections
    */
   public double getPolygonIntersectionAreaWhenSnapped(ConvexPolygon2D convexPolygon2d, RigidBodyTransform snappingTransform)
   {
      return getPolygonIntersectionAreaWhenSnapped(convexPolygon2d, snappingTransform, null);
   }

   /**
    * Returns all of the intersections when the convexPolygon is snapped onto this PlanarRegion with
    * the snappingTransform.
    *
    * @param convexPolygon2d Polygon to snap.
    * @param snappingTransform RigidBodyTransform that snaps the polygon onto this region. Must have
    *           same surface normal as this region.
    * @param intersectionPolygonToPack ArrayList of ConvexPolygon2d to pack with the intersections.
    * @return intersectionArea Total area of intersections
    */
   public double getPolygonIntersectionAreaWhenSnapped(ConvexPolygon2D convexPolygon2d, RigidBodyTransform snappingTransform,
                                                       ConvexPolygon2D intersectionPolygonToPack)
   {
      ConvexPolygon2D projectedPolygon = snapPolygonIntoRegionAndChangeFrameToRegionFrame(convexPolygon2d, snappingTransform);
      double intersectionArea = 0.0;

      if (intersectionPolygonToPack != null)
      {
         intersectionPolygonToPack.clear();
      }

      RigidBodyTransform inverseSnappingTransform = new RigidBodyTransform(snappingTransform);
      inverseSnappingTransform.invert();
      RigidBodyTransform regionToPolygonTransform = new RigidBodyTransform();
      regionToPolygonTransform.set(inverseSnappingTransform);
      regionToPolygonTransform.multiply(fromLocalToWorldTransform);

      // Now, just need to go through each polygon of this region and see there is at least one intersection
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2D intersectingPolygon = new ConvexPolygon2D();
         convexPolygonTools.computeIntersectionOfPolygons(convexPolygons.get(i), projectedPolygon, intersectingPolygon);

         if (intersectingPolygon != null)
         {
            intersectionArea += intersectingPolygon.getArea();

            if (intersectionPolygonToPack != null)
            {
               intersectingPolygon.applyTransform(regionToPolygonTransform, false);
               intersectionPolygonToPack.addVertices(intersectingPolygon);
            }
         }
      }

      if (intersectionPolygonToPack != null)
      {
         intersectionPolygonToPack.update();
      }

      return intersectionArea;
   }

   /**
    * Snaps the given polygon to the frame of this planar region and then transforms it to be in
    * this planar region. If the snappingTransform is not consistent with this PlanarRegion, then it
    * prints an error message.
    *
    * @param polygonToSnap
    * @param snappingTransform
    * @return ConvexPolygon2d Snapped polygon in the frame of this PlanarRegion.
    */
   public ConvexPolygon2D snapPolygonIntoRegionAndChangeFrameToRegionFrame(ConvexPolygon2D polygonToSnap, RigidBodyTransform snappingTransform)
   {
      RigidBodyTransform fromPolygonToPlanarRegionTransform = new RigidBodyTransform();
      fromPolygonToPlanarRegionTransform.set(fromWorldToLocalTransform);
      fromPolygonToPlanarRegionTransform.multiply(snappingTransform);

      double m02 = Math.abs(fromPolygonToPlanarRegionTransform.getM02());
      double m12 = Math.abs(fromPolygonToPlanarRegionTransform.getM12());

      if ((Math.abs(m02) > 1e-4) || (Math.abs(m12) > 1e-4))
      {
         System.err.println("Snapping transform does not seem consistent with PlanarRegion transform!");
      }

      ConvexPolygon2D snappedPolygonToReturn = new ConvexPolygon2D(polygonToSnap);
      snappedPolygonToReturn.applyTransform(fromPolygonToPlanarRegionTransform, false);

      return snappedPolygonToReturn;
   }

   /**
    * Projects the input ConvexPolygon2d to the plane defined by this PlanarRegion by translating
    * each vertex in world z. Then puts each vertex in local frame. In doing so, the area of the
    * rotated polygon will actually increase on tilted PlanarRegions.
    *
    * @param convexPolygonInWorld Polygon to project
    * @return new projected ConvexPolygon2d
    */
   private ConvexPolygon2D projectPolygonVerticallyToRegion(ConvexPolygon2DReadOnly convexPolygonInWorld)
   {
      ConvexPolygon2D projectedPolygon = new ConvexPolygon2D();

      Point3D snappedVertex3d = new Point3D();

      for (int i = 0; i < convexPolygonInWorld.getNumberOfVertices(); i++)
      {
         Point2DReadOnly originalVertex = convexPolygonInWorld.getVertex(i);
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
    * Projects the input LineSegment2d to the plane defined by this PlanarRegion by translating each
    * vertex in world z. Then puts each vertex in local frame. In doing so, the length of the
    * rotated lineSegment will actually increase on tilted PlanarRegions.
    *
    * @param lineSegmentInWorld LineSegment2d to project
    * @return new projected LineSegment2d
    */
   private LineSegment2D projectLineSegmentVerticallyToRegion(LineSegment2DReadOnly lineSegmentInWorld)
   {
      Point2DReadOnly originalVertex = lineSegmentInWorld.getFirstEndpoint();
      Point3D snappedVertex3d = new Point3D();

      // Find the vertex 3d that is snapped to the plane following z-world.
      snappedVertex3d.setX(originalVertex.getX());
      snappedVertex3d.setY(originalVertex.getY());
      snappedVertex3d.setZ(getPlaneZGivenXY(originalVertex.getX(), originalVertex.getY()));

      // Transform to local coordinates
      fromWorldToLocalTransform.transform(snappedVertex3d);
      Point2D snappedFirstEndpoint = new Point2D(snappedVertex3d.getX(), snappedVertex3d.getY());

      originalVertex = lineSegmentInWorld.getSecondEndpoint();

      // Find the vertex 3d that is snapped to the plane following z-world.
      snappedVertex3d.setX(originalVertex.getX());
      snappedVertex3d.setY(originalVertex.getY());
      snappedVertex3d.setZ(getPlaneZGivenXY(originalVertex.getX(), originalVertex.getY()));

      // Transform to local coordinates
      fromWorldToLocalTransform.transform(snappedVertex3d);
      Point2D snappedSecondEndpoint = new Point2D(snappedVertex3d.getX(), snappedVertex3d.getY());

      LineSegment2D projectedLineSegment = new LineSegment2D(snappedFirstEndpoint, snappedSecondEndpoint);
      return projectedLineSegment;
   }

   /**
    * Computes if the point is in the region projected onto the world xy-plane. Note that the
    * z-coordinate of the query is ignored.
    *
    * @param point3d query coordinates.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInsideByProjectionOntoXYPlane(Point3DReadOnly point3d)
   {
      return isPointInsideByProjectionOntoXYPlane(point3d.getX(), point3d.getY());
   }

   /**
    * Computes if the point is in the region projected onto the world xy-plane.
    *
    * @param point2d query coordinates.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInsideByProjectionOntoXYPlane(Point2DReadOnly point2d)
   {
      return isPointInsideByProjectionOntoXYPlane(point2d.getX(), point2d.getY());
   }

   /**
    * Computes if the point is in the region projected onto the world xy-plane.
    *
    * @param x x-coordinate of the query.
    * @param y y-coordinate of the query.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInsideByProjectionOntoXYPlane(double x, double y)
   {
      Point3D localPoint = new Point3D();
      localPoint.setX(x);
      localPoint.setY(y);
      localPoint.setZ(getPlaneZGivenXY(x, y));

      fromWorldToLocalTransform.transform(localPoint);

      return isPointInside(localPoint.getX(), localPoint.getY());
   }

   public boolean isPointInsideByVerticalLineIntersection(double x, double y)
   {
      Line3D verticalLine = new Line3D(x, y, 0.0, 0.0, 0.0, 1.0);
      return intersectWithLine(verticalLine) != null;
   }

   /**
    * Will return the intersection point between a line and a single planar region. If the line does
    * not intersect the region this method will return null.
    */
   public Point3D intersectWithLine(Line3D projectionLineInWorld)
   {
      Vector3DReadOnly planeNormal = new Vector3D(0.0, 0.0, 1.0);
      Point3DReadOnly pointOnPlane = new Point3D(getConvexPolygon(0).getVertex(0));

      Point3DBasics pointOnLineInLocal = new Point3D(projectionLineInWorld.getPoint());
      Vector3DBasics directionOfLineInLocal = new Vector3D(projectionLineInWorld.getDirection());

      transformFromWorldToLocal(pointOnLineInLocal);
      transformFromWorldToLocal(directionOfLineInLocal);

      Point3D intersectionWithPlaneInLocal = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pointOnPlane,
                                                                                                     planeNormal,
                                                                                                     pointOnLineInLocal,
                                                                                                     directionOfLineInLocal);
      if (intersectionWithPlaneInLocal == null) // line was parallel to plane
      {
         return null;
      }

      if (isPointInside(intersectionWithPlaneInLocal.getX(), intersectionWithPlaneInLocal.getY()))
      {
         transformFromLocalToWorld(intersectionWithPlaneInLocal);
         return intersectionWithPlaneInLocal;
      }

      return null; // line does not intersect
   }

   /**
    * Computes the distance of the point to the region projected onto the world xy-plane.
    *
    * @param point2d query coordinates.
    * @return distance to this region. If 0.0, point is in the region.
    */
   public double distanceToPointByProjectionOntoXYPlane(Point2DReadOnly point2d)
   {
      return distanceToPointByProjectionOntoXYPlane(point2d.getX(), point2d.getY());
   }

   private final Point3D localPoint = new Point3D();
   private final Point2D localPoint2D = new Point2D();

   /**
    * Computes the distance of the point to the region projected onto the world xy-plane.
    *
    * @param x x-coordinate of the query.
    * @param y y-coordinate of the query.
    * @return distance to this region. If 0.0, point is in the region.
    */
   public double distanceToPointByProjectionOntoXYPlane(double x, double y)
   {
      localPoint.setX(x);
      localPoint.setY(y);
      localPoint.setZ(getPlaneZGivenXY(x, y));

      fromWorldToLocalTransform.transform(localPoint);
      localPoint2D.set(localPoint);

      return distanceToPoint(localPoint2D);
   }

   /**
    * Given a 3D point in world coordinates, computes whether the point is in this region.
    *
    * @param point3dInWorld query expressed in world coordinates.
    * @param maximumOrthogonalDistance tolerance expressed as maximum orthogonal distance from the
    *           region.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInside(Point3DReadOnly point3dInWorld, double maximumOrthogonalDistance)
   {
      Point3D localPoint = new Point3D();
      fromWorldToLocalTransform.transform(point3dInWorld, localPoint);

      if (!MathTools.intervalContains(localPoint.getZ(), maximumOrthogonalDistance))
         return false;
      else
         return isPointInside(localPoint.getX(), localPoint.getY());
   }

   /**
    * Checks to see if a given point is on the plane or above it by the specified distance.
    *
    * @param point3dInWorld the point to check
    * @return True if the point is on the plane or no more than distanceFromPlane above it.
    */
   public boolean isPointInWorld2DInside(Point3DReadOnly point3dInWorld)
   {
      return isPointInWorld2DInside(point3dInWorld, 0.0);
   }

   /**
    * Checks to see if a given point is on the plane or above it by the specified distance.
    *
    * @param point3dInWorld the point to check
    * @param epsilon epsilon to use in the test
    * @return True if the point is on the plane or no more than distanceFromPlane above it.
    */
   public boolean isPointInWorld2DInside(Point3DReadOnly point3dInWorld, double epsilon)
   {
      Point3D localPoint = new Point3D();
      fromWorldToLocalTransform.transform(point3dInWorld, localPoint);

      return isPointInside(localPoint.getX(), localPoint.getY(), epsilon);
   }

   /**
    * Checks to see if a given point is on the plane or above it by the specified distance.
    *
    * @param point3dInWorld the point to check
    * @param distanceFromPlane The distance above the plane that the point is allowed to be
    * @return True if the point is on the plane or no more than distanceFromPlane above it.
    */
   public boolean isPointOnOrSlightlyAbove(Point3DReadOnly point3dInWorld, double distanceFromPlane)
   {
      MathTools.checkPositive(distanceFromPlane);
      Point3D localPoint = new Point3D();
      fromWorldToLocalTransform.transform(point3dInWorld, localPoint);

      boolean onOrAbove = localPoint.getZ() >= 0.0;
      boolean withinDistance = localPoint.getZ() < distanceFromPlane;
      boolean isInsideXY = isPointInside(localPoint.getX(), localPoint.getY());

      return onOrAbove && withinDistance && isInsideXY;
   }

   /**
    * Checks to see if a given point is on the plane or below it by the specified distance.
    *
    * @param point3dInWorld the point to check
    * @param distanceFromPlane The distance below the plane that the point is allowed to be
    * @return True if the point is on the plane or no more than distanceFromPlane below it.
    */
   public boolean isPointOnOrSlightlyBelow(Point3DReadOnly point3dInWorld, double distanceFromPlane)
   {
      MathTools.checkPositive(distanceFromPlane);
      Point3D localPoint = new Point3D();
      fromWorldToLocalTransform.transform(point3dInWorld, localPoint);

      boolean onOrBelow = localPoint.getZ() <= 0.0;
      boolean withinDistance = localPoint.getZ() > (distanceFromPlane * -1.0);
      boolean isInsideXY = isPointInside(localPoint.getX(), localPoint.getY());

      return onOrBelow && withinDistance && isInsideXY;
   }

   /**
    * Given a 2D point expressed in the plane local frame, computes whether the point is in this
    * region.
    *
    * @param point2dInLocal query expressed in local coordinates.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInside(Point2DReadOnly point2dInLocal)
   {
      return isPointInside(point2dInLocal.getX(), point2dInLocal.getY());
   }

   /**
    * Given a 2D point expressed in the plane local frame, computes whether the point is in this
    * region.
    *
    * @param point2dInLocal query expressed in local coordinates.
    * @param epsilon the tolerance to use during the test.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInside(Point2DReadOnly point2dInLocal, double epsilon)
   {
      return isPointInside(point2dInLocal.getX(), point2dInLocal.getY(), epsilon);
   }

   /**
    * Given a 2D point expressed in the plane local frame, computes whether the point is in this
    * region.
    *
    * @param xCoordinateInLocal x Coordinate of the 2D point in planar region local frame
    * @param yCoordinateInLocal y Coordinate of the 2D point in planar region local frame
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInside(double xCoordinateInLocal, double yCoordinateInLocal)
   {
      for (int i = 0; i < convexPolygons.size(); i++)
      {
         if (convexPolygons.get(i).isPointInside(xCoordinateInLocal, yCoordinateInLocal))
            return true;
      }
      return false;
   }

   /**
    * Given a 2D point expressed in the plane local frame, computes whether the point is in this
    * region.
    *
    * @param xCoordinateInLocal x Coordinate of the 2D point in planar region local frame
    * @param yCoordinateInLocal y Coordinate of the 2D point in planar region local frame
    * @param epsilon the tolerance to use during the test.
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInside(double xCoordinateInLocal, double yCoordinateInLocal, double epsilon)
   {
      for (int i = 0; i < convexPolygons.size(); i++)
      {
         if (convexPolygons.get(i).isPointInside(xCoordinateInLocal, yCoordinateInLocal, epsilon))
            return true;
      }
      return false;
   }

   /**
    * Given a 2D point expressed in the plane local frame, computes whether the point is in this
    * region.
    *
    * @param localPoint Coordinate of the 2D point in planar region local frame
    * @return shortest distance from the point to the planar region
    */
   public double distanceToPoint(Point2DReadOnly localPoint)
   {
      double shortestDistanceToPoint = Double.POSITIVE_INFINITY;
      for (int i = 0; i < convexPolygons.size(); i++)
      {
         double distance = convexPolygons.get(i).distance(localPoint);
         if (distance < shortestDistanceToPoint)
            shortestDistanceToPoint = distance;
      }
      return shortestDistanceToPoint;
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

   /**
    * Every can be given a unique. The default value is {@value #NO_REGION_ID} which corresponds to
    * no id.
    *
    * @param regionId set the unique id of this region.
    */
   public void setRegionId(int regionId)
   {
      this.regionId = regionId;
   }

   /**
    * @return the unique id of this regions. It is equal to {@value #NO_REGION_ID} when no id has
    *         been attributed.
    */
   public int getRegionId()
   {
      return regionId;
   }

   /**
    * @return whether a unique id has been attributed to this region or not.
    */
   public boolean hasARegionId()
   {
      return regionId != NO_REGION_ID;
   }

   /**
    * Returns true only if there is no polygons in this planar region. Does not check for empty
    * polygons.
    */
   public boolean isEmpty()
   {
      return convexPolygons.isEmpty();
   }

   public Point2D[] getConcaveHull()
   {
      return concaveHullsVertices;
   }

   private void checkConcaveHullRepeatVertices(boolean throwException)
   {
      for (int i=0; i<concaveHullsVertices.length; i++)
      {
         int nextIndex = (i + 1) % concaveHullsVertices.length;

         Point2D vertex = concaveHullsVertices[i];
         Point2D nextVertex = concaveHullsVertices[nextIndex];

         if (vertex.distance(nextVertex) < 1e-7)
         {
            LogTools.error("Setting concave hull with repeat vertices" + vertex);
            if (throwException)
            {
               throw new RuntimeException("Setting concave hull with repeat vertices" + vertex);
            }
         }
      }
   }

   public Point2D getConcaveHullVertex(int i)
   {
      return concaveHullsVertices[i];
   }

   public int getConcaveHullSize()
   {
      return concaveHullsVertices.length;
   }

   /** Returns the number of convex polygons representing this region. */
   public int getNumberOfConvexPolygons()
   {
      return convexPolygons.size();
   }

   public List<ConvexPolygon2D> getConvexPolygons()
   {
      return convexPolygons;
   }

   /**
    * Returns the i<sup>th</sup> convex polygon representing a portion of this region. The polygon
    * is expressed in the region local coordinates.
    */
   public ConvexPolygon2D getConvexPolygon(int i)
   {
      return convexPolygons.get(i);
   }

   /**
    * Returns the last convex polygon representing a portion of this region. Special case: returns
    * null when this region is empty. The polygon is expressed in the region local coordinates.
    */
   public ConvexPolygon2D getLastConvexPolygon()
   {
      if (isEmpty())
         return null;
      else
         return getConvexPolygon(getNumberOfConvexPolygons() - 1);
   }

   /**
    * Returns the i<sup>th</sup> convex polygon representing a portion of this region and removes it
    * from this planar region. The polygon is expressed in the region local coordinates.
    */
   public ConvexPolygon2D pollConvexPolygon(int i)
   {
      ConvexPolygon2D polledPolygon = convexPolygons.remove(i);
      updateBoundingBox();
      updateConvexHull();
      return polledPolygon;
   }

   /**
    * Returns the last convex polygon representing a portion of this region and removes it from this
    * planar region. Special case: returns null when this region is empty. The polygon is expressed
    * in the region local coordinates.
    */
   public ConvexPolygon2D pollLastConvexPolygon()
   {
      if (isEmpty())
         return null;
      else
         return pollConvexPolygon(getNumberOfConvexPolygons() - 1);
   }

   /**
    * Retrieves and returns a copy of the normal in world frame of this planar region.
    */
   public Vector3D getNormal()
   {
      Vector3D normal = new Vector3D();
      getNormal(normal);
      return normal;
   }

   /**
    * Retrieves the normal of this planar region in the world frame and stores it in the given {@link Vector3D}.
    *
    * @param normalToPack used to store the normal of this planar region.
    */
   public void getNormal(Vector3DBasics normalToPack)
   {
      normalToPack.setX(fromLocalToWorldTransform.getM02());
      normalToPack.setY(fromLocalToWorldTransform.getM12());
      normalToPack.setZ(fromLocalToWorldTransform.getM22());
   }

   /**
    * Returns true if this PlanarRegion is purely vertical, as far as numerical roundoff is
    * concerned. Checks z component of surface normal. If absolute value is really small, then
    * returns true.
    *
    * @return true if vertical. false otherwise.
    */
   public boolean isVertical()
   {
      return (Math.abs(fromLocalToWorldTransform.getM22()) < 1e-10);
   }

   /**
    * Retrieves a point that lies in this planar region. This point is also used as the origin of
    * the local coordinate system of this planar region.
    *
    * @param pointToPack used to store the point coordinates.
    */
   public void getPointInRegion(Point3DBasics pointToPack)
   {
      fromLocalToWorldTransform.getTranslation(pointToPack);
   }

   /**
    * Get the transform from local coordinates to world coordinates.
    *
    * @param transformToPack used to store the transform.
    */
   public void getTransformToWorld(RigidBodyTransform transformToPack)
   {
      transformToPack.set(fromLocalToWorldTransform);
   }

   /**
    * Get the transform from world coordinates to local coordinates.
    *
    * @param transformToPack used to store the transform.
    */
   public void getTransformToLocal(RigidBodyTransform transformToPack)
   {
      transformToPack.set(fromWorldToLocalTransform);
   }

   /**
    * Get a reference to the PlanarRegion's axis-aligned minimal bounding box (AABB) in world.
    *
    * @return the axis-aligned minimal bounding box for the planar region, in world coordinates.
    */
   public BoundingBox3D getBoundingBox3dInWorld()
   {
      return this.boundingBox3dInWorld;
   }

   /**
    * Get a deep copy of this PlanarRegion's axis-aligned minimal bounding box (AABB) in world
    *
    * @return a deep copy of the axis-aligned minimal bounding box for the planar region, in world
    *         coordinates.
    */
   public BoundingBox3D getBoundingBox3dInWorldCopy()
   {
      return new BoundingBox3D(this.boundingBox3dInWorld);
   }

   /**
    * Set defining points of the passed-in BoundingBox3D to the same as those in this PlanarRegion's
    * axis-aligned minimal bounding box (AABB) in world coordinates.
    *
    * @param boundingBox3dToPack the bounding box that will be updated to reflect this
    *           PlanarRegion's AABB
    */
   public void getBoundingBox3dInWorld(BoundingBox3D boundingBox3dToPack)
   {
      boundingBox3dToPack.set(this.boundingBox3dInWorld);
   }

   @Override
   public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      if (convexHull.isEmpty())
      {
         return false;
      }
      else if (convexHull.getNumberOfVertices() == 1)
      {
         supportingVertexToPack.set(convexHull.getVertex(0), 0.0);
         transformFromLocalToWorld(supportingVertexToPack);
         return true;
      }

      supportingVertexToPack.set(supportDirection);
      fromWorldToLocalTransform.getRotation().transform(supportingVertexToPack);

      double vx = supportingVertexToPack.getX();
      double vy = supportingVertexToPack.getY();

      double dotProduct0 = vx * convexHull.getVertex(0).getX() + vy * convexHull.getVertex(0).getY();
      double dotProductCW = vx * convexHull.getVertex(1).getX() + vy * convexHull.getVertex(1).getY();
      double dotProductCCW =
            vx * convexHull.getVertex(convexHull.getNumberOfVertices() - 1).getX() + vy * convexHull.getVertex(convexHull.getNumberOfVertices() - 1).getY();

      int bestVertexIndex = 0;
      if (convexHull.getNumberOfVertices() == 2)
      {
         bestVertexIndex = dotProduct0 > dotProductCW ? 0 : 1;
      }
      else if (dotProduct0 < Math.max(dotProductCW, dotProductCCW))
      {
         boolean iterateClockwise = dotProductCW > dotProductCCW;
         double previousDotProduct = iterateClockwise ? dotProductCW : dotProductCCW;
         int indexToCheck = iterateClockwise ? 2 : convexHull.getNumberOfVertices() - 2;

         while (true)
         {
            double dotProduct = vx * convexHull.getVertex(indexToCheck).getX() + vy * convexHull.getVertex(indexToCheck).getY();
            if (dotProduct > previousDotProduct)
            {
               previousDotProduct = dotProduct;
               indexToCheck = iterateClockwise ? convexHull.getNextVertexIndex(indexToCheck) : convexHull.getPreviousVertexIndex(indexToCheck);
            }
            else
            {
               bestVertexIndex = iterateClockwise ? convexHull.getPreviousVertexIndex(indexToCheck) : convexHull.getNextVertexIndex(indexToCheck);
               break;
            }
         }
      }

      supportingVertexToPack.set(convexHull.getVertex(bestVertexIndex), 0.0);
      transformFromLocalToWorld(supportingVertexToPack);
      return true;
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

   public void set(PlanarRegion other)
   {
      fromLocalToWorldTransform.set(other.fromLocalToWorldTransform);
      fromWorldToLocalTransform.set(other.fromWorldToLocalTransform);
      convexPolygons.clear();
      for (int i = 0; i < other.getNumberOfConvexPolygons(); i++)
         convexPolygons.add(new ConvexPolygon2D(other.convexPolygons.get(i)));

      updateBoundingBox();
      convexHull.set(other.convexHull);
   }

   public void setBoundingBoxEpsilon(double epsilon)
   {
      boundingBoxEpsilon = epsilon;
      updateBoundingBox();
   }

   private void updateBoundingBox()
   {
      boundingBox3dInWorld.set(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
      for (int i = 0; i < this.getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2D convexPolygon = this.getConvexPolygon(i);

         for (int j = 0; j < convexPolygon.getNumberOfVertices(); j++)
         {
            Point2DReadOnly vertex = convexPolygon.getVertex(j);
            tempPointForConvexPolygonProjection.set(vertex.getX(), vertex.getY(), 0.0);
            fromLocalToWorldTransform.transform(tempPointForConvexPolygonProjection);

            this.boundingBox3dInWorld.updateToIncludePoint(tempPointForConvexPolygonProjection);
         }
      }

      Point3DReadOnly minPoint = boundingBox3dInWorld.getMinPoint();
      Point3DReadOnly maxPoint = boundingBox3dInWorld.getMaxPoint();

      this.boundingBox3dInWorld.setMin(minPoint.getX() - boundingBoxEpsilon, minPoint.getY() - boundingBoxEpsilon, minPoint.getZ() - boundingBoxEpsilon);
      this.boundingBox3dInWorld.setMax(maxPoint.getX() + boundingBoxEpsilon, maxPoint.getY() + boundingBoxEpsilon, maxPoint.getZ() + boundingBoxEpsilon);
   }

   private void updateConvexHull()
   {
      convexHull.clear();
      for (int i = 0; i < this.getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2D convexPolygon = this.getConvexPolygon(i);
         for (int j = 0; j < convexPolygon.getNumberOfVertices(); j++)
            convexHull.addVertex(convexPolygon.getVertex(j));
      }
      convexHull.update();
   }

   /**
    * @return a full depth copy of this region. The copy can be entirely modified without
    *         interfering with this region.
    */
   public PlanarRegion copy()
   {
      RigidBodyTransform transformToWorldCopy = new RigidBodyTransform(fromLocalToWorldTransform);
      Point2D[] concaveHullCopy = new Point2D[concaveHullsVertices.length];
      for (int i = 0; i < concaveHullsVertices.length; i++)
         concaveHullCopy[i] = new Point2D(concaveHullsVertices[i]);

      List<ConvexPolygon2D> convexPolygonsCopy = new ArrayList<>();
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
         convexPolygonsCopy.add(new ConvexPolygon2D(convexPolygons.get(i)));

      PlanarRegion planarRegion = new PlanarRegion(transformToWorldCopy, concaveHullCopy, convexPolygonsCopy);
      planarRegion.setRegionId(regionId);
      return planarRegion;
   }

   /**
    * @return the convex hull of the region.
    */
   public ConvexPolygon2D getConvexHull()
   {
      return convexHull;
   }

   public static PlanarRegion generatePlanarRegionFromRandomPolygonsWithRandomTransform(Random random, int numberOfRandomlyGeneratedPolygons,
                                                                                        double maxAbsoluteXYForPolygons, int numberOfPossiblePointsForPolygons)
   {
      List<ConvexPolygon2D> regionConvexPolygons = new ArrayList<>();

      for (int i = 0; i < numberOfRandomlyGeneratedPolygons; i++)
      {
         ConvexPolygon2D randomPolygon = EuclidGeometryRandomTools.nextConvexPolygon2D(random, maxAbsoluteXYForPolygons, numberOfPossiblePointsForPolygons);
         regionConvexPolygons.add(randomPolygon);
      }

      for (ConvexPolygon2D convexPolygon : regionConvexPolygons)
         convexPolygon.update();

      Vector3D randomTranslation = RandomGeometry.nextVector3D(random, 10.0);
      Quaternion randomOrientation = RandomGeometry.nextQuaternion(random, Math.toRadians(45.0));
      RigidBodyTransform regionTransform = new RigidBodyTransform(randomOrientation, randomTranslation);

      return new PlanarRegion(regionTransform, regionConvexPolygons);
   }

   /**
    * Transforms the planar region
    *
    * @param fromDesiredToCurrentTransform transform from current frame to desired frame
    */
   public void transform(RigidBodyTransform fromDesiredToCurrentTransform)
   {
      fromLocalToWorldTransform.multiply(fromDesiredToCurrentTransform);
      fromWorldToLocalTransform.set(fromLocalToWorldTransform);
      fromWorldToLocalTransform.invert();

      updateBoundingBox();
      updateConvexHull();
   }

   public void update()
   {
      updateBoundingBox();
      updateConvexHull();
   }

   /**
    * Intersect this region's plane with 3D plane resulting in a Line3D in world.
    *
    * @param plane
    * @return line3D in world
    */
   public Line3D intersectionWith(Plane3D plane)
   {
      Plane3D thisPlane = getPlane();

      Point3D intersectionPoint = new Point3D();
      Vector3D intersectionDirection = new Vector3D();
      EuclidGeometryTools.intersectionBetweenTwoPlane3Ds(thisPlane.getPoint(),
                                                         thisPlane.getNormal(),
                                                         plane.getPoint(),
                                                         plane.getNormal(),
                                                         intersectionPoint,
                                                         intersectionDirection);
      return new Line3D(intersectionPoint, intersectionDirection);
   }

   /**
    * Returns the intersection between this region and the provided region. Since a planar region
    * is not always convex the result is a list of line segments.
    */
   public List<LineSegment3D> intersect(PlanarRegion other)
   {
      List<LineSegment3D> ret = new ArrayList<>();
      if (!boundingBox3dInWorld.intersectsInclusive(other.boundingBox3dInWorld))
      {
         return ret;
      }

      Line3D fullIntersectionLine = intersectionWith(other.getPlane());

      List<LineSegment3D> intersectionsWithThis = projectAndIntersect(fullIntersectionLine);
      List<LineSegment3D> intersectionsWithOther = other.projectAndIntersect(fullIntersectionLine);

      for (LineSegment3D intersectionWithThis : intersectionsWithThis)
      {
         Point3DBasics start = intersectionWithThis.getFirstEndpoint();
         Point3DBasics end = intersectionWithThis.getSecondEndpoint();
         for (LineSegment3D intersectionWithOther : intersectionsWithOther)
         {
            if (intersectionWithThis.getDirection(false).dot(intersectionWithOther.getDirection(false)) < 0.0)
            {
               intersectionWithOther.flipDirection();
            }

            double startPercent = EuclidGeometryTools.percentageAlongLineSegment3D(start, intersectionWithOther.getFirstEndpoint(),
                                                                                   intersectionWithOther.getSecondEndpoint());
            double endPercent = EuclidGeometryTools.percentageAlongLineSegment3D(end, intersectionWithOther.getFirstEndpoint(),
                                                                                 intersectionWithOther.getSecondEndpoint());

            if (startPercent < 0.0 && endPercent < 0.0)
            {
               continue;
            }
            else if (startPercent > 1.0 && endPercent > 1.0)
            {
               continue;
            }

            boolean startThisInOther = MathTools.intervalContains(startPercent, 0.0, 1.0);
            boolean endThisInOther = MathTools.intervalContains(endPercent, 0.0, 1.0);
            if (startThisInOther && endThisInOther)
            {
               ret.add(intersectionWithThis);
            }
            else if (!startThisInOther && !endThisInOther)
            {
               ret.add(intersectionWithOther);
            }
            else if (startThisInOther && !endThisInOther)
            {
               ret.add(new LineSegment3D(start, intersectionWithOther.getSecondEndpoint()));
            }
            else if (!startThisInOther && endThisInOther)
            {
               ret.add(new LineSegment3D(intersectionWithOther.getFirstEndpoint(), end));
            }
            else
            {
               throw new RuntimeException("Mistake in Algorithm.");
            }
         }
      }

      return ret;
   }

   /**
    * Returns the intersection between this and the line when projected onto the region.
    * <p>
    * The provided line is projected onto the region in the direction of the z axis of the
    * region frame (the plane normal). Then all intersections of this projected line and
    * the plane are computed.
    * <p>
    * Since a planar region is not always convex the result is a list of line segments.
    */
   public List<LineSegment3D> projectAndIntersect(Line3D line)
   {
      Line3D localLine = new Line3D(line);
      localLine.applyTransform(fromWorldToLocalTransform);
      Line2D lineInPlane = new Line2D(localLine.getPointX(), localLine.getPointY(), localLine.getDirectionX(), localLine.getDirectionY());

      List<LineSegment3D> ret = new ArrayList<>();
      for (ConvexPolygon2D polygon : convexPolygons)
      {
         Point2DBasics[] intersectionPoints = polygon.intersectionWith(lineInPlane);
         if (intersectionPoints == null || intersectionPoints.length < 2)
         {
            continue;
         }

         Point3D point0 = new Point3D(intersectionPoints[0]);
         Point3D point1 = new Point3D(intersectionPoints[1]);
         LineSegment3D intersection = new LineSegment3D(point0, point1);
         intersection.applyTransform(fromLocalToWorldTransform);
         ret.add(intersection);
      }

      return ret;
   }

   /**
    * Creates and returns the Plane3D that this planar region lies on.
    */
   public Plane3D getPlane()
   {
      Plane3D ret = new Plane3D();
      ret.setPoint(fromLocalToWorldTransform.getM03(), fromLocalToWorldTransform.getM13(), fromLocalToWorldTransform.getM23());
      ret.setNormal(fromLocalToWorldTransform.getM02(), fromLocalToWorldTransform.getM12(), fromLocalToWorldTransform.getM22());
      return ret;
   }

   /**
    * Transforms the given object in the local coordinates of this planar region.
    * <p>
    * Assumes the object is originally expressed in world coordinates.
    * </p>
    *
    * @param objectToTransform the object to be transformed. Modified.
    */
   public void transformFromWorldToLocal(Transformable objectToTransform)
   {
      objectToTransform.applyTransform(fromWorldToLocalTransform);
   }

   /**
    * Transforms the given object in the world coordinates.
    * <p>
    * Assumes the object is originally expressed in local coordinates of this planar region.
    * </p>
    *
    * @param objectToTransform the object to be transformed. Modified.
    */
   public void transformFromLocalToWorld(Transformable objectToTransform)
   {
      objectToTransform.applyTransform(fromLocalToWorldTransform);
   }

   @Override
   public String toString()
   {
      StringBuffer buffer = new StringBuffer();

      buffer.append("boundingBox: " + boundingBox3dInWorld.toString() + "\n");
      buffer.append("number of polygons: " + convexPolygons.size() + "\n");

      buffer.append("transformToWorld:\n" + fromLocalToWorldTransform + "\n");

      int maxNumberOfPolygonsToPrint = 5;
      for (int i = 0; i < Math.min(maxNumberOfPolygonsToPrint, convexPolygons.size()); i++)
      {
         buffer.append(convexPolygons.get(i) + "\n");
      }
      if (convexPolygons.size() > maxNumberOfPolygonsToPrint)
      {
         buffer.append("...\n");
      }

      return buffer.toString();
   }
}