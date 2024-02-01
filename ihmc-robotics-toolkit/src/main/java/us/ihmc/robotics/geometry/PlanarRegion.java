package us.ihmc.robotics.geometry;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.geometry.interfaces.*;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.*;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Vector4D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.RegionInWorldInterface;
import us.ihmc.robotics.random.RandomGeometry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class PlanarRegion implements SupportingVertexHolder, RegionInWorldInterface<PlanarRegion>
{
   public static final int NO_REGION_ID = -1;
   public static final double DEFAULT_BOUNDING_BOX_EPSILON = 0.0;

   private int regionId = NO_REGION_ID;
   private int numberOfTimesMatched = 0;
   private int tickOfLastMeasurement = 0;

   private double area = 0;

   /**
    * This transform also represents the pose of the PlanarRegion.
    */
   private final RigidBodyTransform fromLocalToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform fromWorldToLocalTransform = new RigidBodyTransform();

   private final RecyclingArrayList<Point2D> concaveHullsVertices = new RecyclingArrayList<>(Point2D::new);
   /**
    * List of the convex polygons representing this planar region. They are in the local frame of
    * the plane.
    */
   private final RecyclingArrayList<ConvexPolygon2D> convexPolygons = new RecyclingArrayList<>(ConvexPolygon2D::new);
   /** To detect concave hull separation */
   private final List<Boolean> visited = new ArrayList<>();

   private final BoundingBox3D boundingBox3dInWorld = new BoundingBox3D(new Point3D(Double.NaN, Double.NaN, Double.NaN),
                                                                        new Point3D(Double.NaN, Double.NaN, Double.NaN));
   private double boundingBoxEpsilon = DEFAULT_BOUNDING_BOX_EPSILON;
   private final Point3D tempPointForConvexPolygonProjection = new Point3D();

   private final ConvexPolygon2D convexHull = new ConvexPolygon2D();

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   private final PlanarRegionOrigin origin = new PlanarRegionOrigin(fromLocalToWorldTransform);
   private final PlanarRegionNormal normal = new PlanarRegionNormal(fromLocalToWorldTransform);

   /**
    * Internal variables for math purposes
    */
   private final ConvexPolygon2D tempPolygon = new ConvexPolygon2D();
   private final Point2D tempPoint2D = new Point2D();
   private final Point3D tempPoint3D = new Point3D();

   /**
    * Create a new, empty planar region.
    */
   public PlanarRegion()
   {
      this(new RigidBodyTransform(), new ArrayList<>());
   }

   /**
    * Create a new planar region.
    *
    * @param transformToWorld transform from the region local coordinate system to world.
    * @param planarRegionConvexPolygons the list of convex polygon that represents the planar
    *           region. Expressed in local coordinate system.
    */
   public PlanarRegion(RigidBodyTransformReadOnly transformToWorld, List<ConvexPolygon2D> planarRegionConvexPolygons)
   {
      this(transformToWorld, new ArrayList<>(), planarRegionConvexPolygons);
      updateConcaveHull();
   }
   
   /**
    * Create a new planar region.
    *
    * @param transformToWorld transform from the region local coordinate system to world.
    * @param concaveHullVertices vertices of the concave hull of the region.
    * @param planarRegionConvexPolygons the list of convex polygon that represents the planar
    *           region. Expressed in local coordinate system.
    */
   public PlanarRegion(RigidBodyTransformReadOnly transformToWorld, List<? extends Point2DBasics> concaveHullVertices, List<ConvexPolygon2D> planarRegionConvexPolygons)
   {
      set(transformToWorld, planarRegionConvexPolygons, concaveHullVertices, regionId);
   }

   /**
    * Create a new planar region.
    *
    * @param transformToWorld transform from the region local coordinate system to world.
    * @param convexPolygon a single convex polygon that represents the planar region. Expressed in
    *           local coordinate system.
    */
   public PlanarRegion(RigidBodyTransformReadOnly transformToWorld, Vertex2DSupplier convexPolygon)
   {
      set(transformToWorld, convexPolygon);
   }

   public void set(RigidBodyTransformReadOnly transformToWorld, Vertex2DSupplier convexPolygon)
   {
      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);

      concaveHullsVertices.clear();
      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
         concaveHullsVertices.add().set(convexPolygon.getVertex(i));

      //TODO: Remove repeat vertices if you have them, or fix upstream so they don't create them.
      checkConcaveHullRepeatVertices(false);

      convexPolygons.clear();
      convexPolygons.add().set(convexPolygon);
      updateBoundingBox();
      updateConvexHull();
      updateArea();
   }

   public void set(RigidBodyTransformReadOnly transformToWorld, List<ConvexPolygon2D> planarRegionConvexPolygons)
   {
      this.set(transformToWorld, planarRegionConvexPolygons, NO_REGION_ID);
   }

   public void set(RigidBodyTransformReadOnly transformToWorld, List<ConvexPolygon2D> planarRegionConvexPolygons, int newRegionId)
   {
      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);

      convexPolygons.clear();
      for (int i = 0; i < planarRegionConvexPolygons.size(); i++)
         convexPolygons.add().set(planarRegionConvexPolygons.get(i));

      updateBoundingBox();
      updateConvexHull();
      updateConcaveHull();
      updateArea();

      regionId = newRegionId;
   }

   public void set(RigidBodyTransformReadOnly transformToWorld, List<ConvexPolygon2D> planarRegionConvexPolygons, List<? extends Point2DReadOnly> concaveHullVertices, int newRegionId)
   {
      fromLocalToWorldTransform.set(transformToWorld);
      fromWorldToLocalTransform.setAndInvert(fromLocalToWorldTransform);

      this.concaveHullsVertices.clear();
      for (int i = 0; i < concaveHullVertices.size(); i++)
         this.concaveHullsVertices.add().set(concaveHullVertices.get(i));

      convexPolygons.clear();
      for (int i = 0; i < planarRegionConvexPolygons.size(); i++)
         convexPolygons.add().set(planarRegionConvexPolygons.get(i));

      updateConvexHull();
      updateBoundingBox();
      updateArea();

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

   private final ConvexPolygon2D projectedPolygonTemp = new ConvexPolygon2D();
   private final ConvexPolygon2D polygonIntersectionTemp = new ConvexPolygon2D();
   /**
    * Check if the given polygon intersects this region projected onto the XY-plane.
    *
    * @param convexPolygonInWorld
    * @return true if the polygon intersects this PlanarRegion.
    */
   public boolean isPolygonIntersecting(ConvexPolygon2DReadOnly convexPolygonInWorld)
   {
      BoundingBox2DReadOnly polygonBoundingBox = convexPolygonInWorld.getBoundingBox();
      if (!boundingBox3dInWorld.intersectsInclusiveInXYPlane(polygonBoundingBox))
         return false;

      // Instead of projecting all the polygons of this region onto the world XY-plane,
      // the given convex polygon is projected along the z-world axis to be snapped onto plane.
      projectPolygonVerticallyToRegion(convexPolygonInWorld, projectedPolygonTemp);

      if (!convexHull.getBoundingBox().intersectsInclusive(projectedPolygonTemp.getBoundingBox()))
         return false;

      // Now, just need to go through each polygon of this region and see there is at least one intersection
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2D polygonToCheck = convexPolygons.get(i);
         if (!polygonToCheck.getBoundingBox().intersectsExclusive(projectedPolygonTemp.getBoundingBox()))
            continue;

         boolean hasIntersection = convexPolygonTools.doPolygonsIntersect(polygonToCheck, projectedPolygonTemp);
         if (hasIntersection)
            return true;
      }
      // Did not find any intersection
      return false;
   }

   /**
    * Compute the total intersection area of the given polygon with this region projected onto the XY-plane.
    *
    * @param convexPolygonInWorld
    * @return Total intersecting area. Greater than 0.0 if intersecting
    */
   public double computeIntersectingArea(ConvexPolygon2DReadOnly convexPolygonInWorld)
   {
      BoundingBox2DReadOnly polygonBoundingBox = convexPolygonInWorld.getBoundingBox();
      if (!boundingBox3dInWorld.intersectsInclusiveInXYPlane(polygonBoundingBox))
         return 0.0;

      // Instead of projecting all the polygons of this region onto the world XY-plane,
      // the given convex polygon is projected along the z-world axis to be snapped onto plane.
      projectPolygonVerticallyToRegion(convexPolygonInWorld, projectedPolygonTemp);

      if (!convexHull.getBoundingBox().intersectsExclusive(projectedPolygonTemp.getBoundingBox()))
         return 0.0;

      double intersectionArea = 0.0;
      // Now, just need to go through each polygon of this region and see there is at least one intersection
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2D polygonToCheck = convexPolygons.get(i);
         if (!polygonToCheck.getBoundingBox().intersectsExclusive(projectedPolygonTemp.getBoundingBox()))
            continue;

         boolean hasIntersection = convexPolygonTools.computeIntersectionOfPolygons(polygonToCheck, projectedPolygonTemp, polygonIntersectionTemp);
         if (hasIntersection)
            intersectionArea += polygonIntersectionTemp.getArea();
      }

      // Did not find any intersection
      return intersectionArea;
   }


   /**
    * Returns all the intersections when the convexPolygon is projected vertically onto this
    * PlanarRegion.
    *
    * WARNING generates garbage
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
      projectPolygonVerticallyToRegion(convexPolygon2DBasics, tempPolygon);

      // Now, just need to go through each polygon of this region and see there is at least one intersection
      for (int i = 0; i < getNumberOfConvexPolygons(); i++)
      {
         ConvexPolygon2D intersectingPolygon = new ConvexPolygon2D();
         if (convexPolygonTools.computeIntersectionOfPolygons(convexPolygons.get(i), tempPolygon, intersectingPolygon))
         {
            intersectionsInPlaneFrameToPack.add(intersectingPolygon);
         }
      }
   }

   public boolean getPolygonIntersection(int convexPolygonIndex, ConvexPolygon2DReadOnly polygonToIntersect, ConvexPolygon2DBasics intersectingPolygonToPack)
   {
      return convexPolygonTools.computeIntersectionOfPolygons(convexPolygons.get(convexPolygonIndex), polygonToIntersect, intersectingPolygonToPack);
   }

   /**
    * Returns all of the intersections when the convexPolygon is snapped onto this PlanarRegion with
    * the snappingTransform.
    *
    * WARNING generates garbage
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
    * WARNING generates garbage
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
    *     * WARNING generates garbage
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
    * WARNING generates garbage
    *
    * @param convexPolygonInWorld Polygon to project
    * @return new projected ConvexPolygon2d
    */
   public ConvexPolygon2D projectPolygonVerticallyToRegion(ConvexPolygon2DReadOnly convexPolygonInWorld)
   {
      ConvexPolygon2D projectedPolygon = new ConvexPolygon2D();

      projectPolygonVerticallyToRegion(convexPolygonInWorld, projectedPolygon);

      return projectedPolygon;
   }

   /**
    * Projects the input ConvexPolygon2d to the plane defined by this PlanarRegion by translating
    * each vertex in world z. Then puts each vertex in local frame. In doing so, the area of the
    * rotated polygon will actually increase on tilted PlanarRegions.
    *
    * @param convexPolygonInWorld Polygon to project
    * @return new projected ConvexPolygon2d
    */
   public void projectPolygonVerticallyToRegion(ConvexPolygon2DReadOnly convexPolygonInWorld, ConvexPolygon2DBasics projectedPolygonToPack)
   {
      projectedPolygonToPack.clear();

      for (int i = 0; i < convexPolygonInWorld.getNumberOfVertices(); i++)
      {
         Point2DReadOnly originalVertex = convexPolygonInWorld.getVertex(i);
         // Find the vertex 3d that is snapped to the plane following z-world.
         tempPoint3D.setX(originalVertex.getX());
         tempPoint3D.setY(originalVertex.getY());
         tempPoint3D.setZ(getPlaneZGivenXY(originalVertex.getX(), originalVertex.getY()));

         // Transform to local coordinates
         fromWorldToLocalTransform.transform(tempPoint3D);
         // Add the snapped vertex to the snapped polygon
         projectedPolygonToPack.addVertex(tempPoint3D);
      }

      projectedPolygonToPack.update();
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
      return isPointInWorld2DInside(point3dInWorld, 1e-7);
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
      boolean isInsideXY = isPointInsideExclusive(localPoint.getX(), localPoint.getY());

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
      boolean isInsideXY = isPointInsideExclusive(localPoint.getX(), localPoint.getY());

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
    * region. Note that a point on the edge using this method counts as inside. If you would
    * like this to return false, use {@link #isPointInside(Point3DReadOnly, double)}
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
    * region. Note that a point on the edge using this method does not count as inside. If you would
    * like this to return true, use {@link #isPointInside(Point3DReadOnly, double)}
    *
    * @param xCoordinateInLocal x Coordinate of the 2D point in planar region local frame
    * @param yCoordinateInLocal y Coordinate of the 2D point in planar region local frame
    * @return true if the point is inside this region, false otherwise.
    */
   public boolean isPointInsideExclusive(double xCoordinateInLocal, double yCoordinateInLocal)
   {
      return PlanarRegionTools.isPointInLocalInsidePlanarRegion(this, xCoordinateInLocal, yCoordinateInLocal, 0.0);
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
      return PlanarRegionTools.isPointInLocalInsidePlanarRegion(this, xCoordinateInLocal, yCoordinateInLocal, 1e-7);
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
      return PlanarRegionTools.isPointInLocalInsidePlanarRegion(this, xCoordinateInLocal, yCoordinateInLocal, epsilon);
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

   public boolean containsNaN()
   {
      boolean containsNaN = getBoundingBox3dInWorld().containsNaN();
      // TODO: containsNaN |= more stuff possibly
      if (containsNaN)
      {
         LogTools.error("Region bounding box contained NaN");
      }
      return containsNaN;
   }

   public List<Point2D> getConcaveHull()
   {
      return concaveHullsVertices;
   }

   private void checkConcaveHullRepeatVertices(boolean throwException)
   {
      if (concaveHullsVertices.size() < 2)
         return;

      for (int i=0; i< concaveHullsVertices.size(); i++)
      {
         int nextIndex = (i + 1) % concaveHullsVertices.size();

         Point2DReadOnly vertex = concaveHullsVertices.get(i);
         Point2DReadOnly nextVertex = concaveHullsVertices.get(nextIndex);

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

   public Point2DReadOnly getConcaveHullVertex(int i)
   {
      return concaveHullsVertices.get(i);
   }

   public int getConcaveHullSize()
   {
      return concaveHullsVertices.size();
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
      pointToPack.set(fromLocalToWorldTransform.getTranslation());
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

   public RigidBodyTransformReadOnly getTransformToLocal()
   {
      return fromWorldToLocalTransform;
   }

   public RigidBodyTransformReadOnly getTransformToWorld()
   {
      return fromLocalToWorldTransform;
   }

   public RigidBodyTransform getTransformToWorldCopy()
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      getTransformToWorld(transformToWorld);
      return transformToWorld;
   }

   public RigidBodyTransform getTransformToLocalCopy()
   {
      RigidBodyTransform transformToLocal = new RigidBodyTransform();
      getTransformToLocal(transformToLocal);
      return transformToLocal;
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
      regionId = other.regionId;
      fromLocalToWorldTransform.set(other.fromLocalToWorldTransform);
      fromWorldToLocalTransform.set(other.fromWorldToLocalTransform);
      convexPolygons.clear();
      for (int i = 0; i < other.getNumberOfConvexPolygons(); i++)
         convexPolygons.add().set(other.convexPolygons.get(i));
      concaveHullsVertices.clear();
      for (int i = 0; i < other.getConcaveHullSize(); i++)
         concaveHullsVertices.add().set(other.getConcaveHull().get(i));

      updateBoundingBox();
      convexHull.set(other.convexHull);

      this.area = other.area;
      this.numberOfTimesMatched = other.numberOfTimesMatched;
      this.tickOfLastMeasurement = other.tickOfLastMeasurement;
   }

   public void setTransformOnly(PlanarRegion other)
   {
      fromLocalToWorldTransform.set(other.fromLocalToWorldTransform);
      fromWorldToLocalTransform.set(other.fromWorldToLocalTransform);
   }

   public void setBoundingBoxEpsilon(double epsilon)
   {
      boundingBoxEpsilon = epsilon;
      updateBoundingBox();
   }

   public void updateBoundingBox()
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

   public void updateConvexHull()
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

   private void updateConcaveHull()
   {
      this.concaveHullsVertices.clear();

      if (convexPolygons.isEmpty())
      {
         return;
      }
      else if (convexPolygons.size() == 1)
      {
         for (int i = 0; i < convexPolygons.get(0).getNumberOfVertices(); i++)
            concaveHullsVertices.add().set(convexPolygons.get(0).getVertex(i));
         return;
      }

      visited.clear(); // for concave hull separation detection

      int maximumIterations = 0;

      double minX = Double.MAX_VALUE;
      int minXPolygonIndex = -1;

      for (int i = 0; i < convexPolygons.size(); i++)
      {
         ConvexPolygon2D polygon = convexPolygons.get(i);
         visited.add(false);

         // Concave hull generation breaks regions contain empty, point or line sub-polygons
         if (polygon.getNumberOfVertices() < 3)
         {
            return;
         }

         if (polygon.getVertex(0).getX() < minX)
         {
            minX = polygon.getVertex(0).getX();
            minXPolygonIndex = i;
         }

         maximumIterations += polygon.getNumberOfVertices();
      }

      // Loop through again in case multiple convex polygons share this vertex. We want to start on the one whose edge between this vertex
      // and it's previous vertex is along the concave perimeter. Find by taking largest dot product with y-axis
      Point2DReadOnly vertex = convexPolygons.get(minXPolygonIndex).getVertex(0);
      double maxDotProduct = Double.NEGATIVE_INFINITY;
      for (int i = 0; i < convexPolygons.size(); i++)
      {
         if (convexPolygons.get(i).getVertex(0).epsilonEquals(vertex, 1e-7))
         {
            double vx = vertex.getX() - convexPolygons.get(i).getPreviousVertex(0).getX();
            double vy = vertex.getY() - convexPolygons.get(i).getPreviousVertex(0).getY();
            double dotProduct = vy / EuclidCoreTools.norm(vx, vy);
            if (dotProduct > maxDotProduct)
            {
               maxDotProduct = dotProduct;
               minXPolygonIndex = i;
            }
         }
      }

      concaveHullsVertices.add().set(convexPolygons.get(minXPolygonIndex).getVertex(0));
      visited.set(minXPolygonIndex, true);
      int polygonIndex = minXPolygonIndex;
      int vertexIndex = 0;
      int iterations = 0;

      while (iterations < maximumIterations)
      {
         int[] neighborRegionAndVertexIndices = findNeighborRegionSharingConcaveHullVertex(polygonIndex, vertexIndex);
         if (neighborRegionAndVertexIndices == null)
         {
            vertexIndex = convexPolygons.get(polygonIndex).getNextVertexIndex(vertexIndex);
         }
         else
         {
            polygonIndex = neighborRegionAndVertexIndices[0];
            vertexIndex = convexPolygons.get(polygonIndex).getNextVertexIndex(neighborRegionAndVertexIndices[1]);
         }

         if (polygonIndex == minXPolygonIndex && vertexIndex == 0)
         {
            break;
         }
         else
         {
            concaveHullsVertices.add().set(convexPolygons.get(polygonIndex).getVertex(vertexIndex));
            visited.set(polygonIndex, true);
         }

         iterations++;
      }

      if (iterations == maximumIterations)
      {
         LogTools.error("Unable to solve for concave hull. region " + regionId);
         concaveHullsVertices.clear();
      }
   }

   public void checkConcaveHullIsNotSeparated()
   {
      boolean allVisited = true;
      for (Boolean value : visited)
      {
         allVisited &= value;
      }
      if (!allVisited)
      {
         LogTools.error("Concave hull is separated. Not all convex polygons were visited. This planar region is invalid. " + visited);
         throw new RuntimeException("Concave hull is separated. Not all convex polygons were visited. This planar region is invalid. " + visited);
      }
   }

   /**
    * Finds the index of the convex polygon that shares the given vertex. If multiple matches are found, the one with the smallest clockwise
    * angle from the given vertex and it's previous vertex is returned. If no match is found, null is returned.
    */
   private int[] findNeighborRegionSharingConcaveHullVertex(int polygonIndex, int vertexIndex)
   {
      ConvexPolygon2D inputPolygon = convexPolygons.get(polygonIndex);
      Point2DReadOnly vertex = inputPolygon.getVertex(vertexIndex);
      Point2DReadOnly previousVertex = inputPolygon.getPreviousVertex(vertexIndex);

      double vx = previousVertex.getX() - vertex.getX();
      double vy = previousVertex.getY() - vertex.getY();

      double maxDotProduct = Double.NEGATIVE_INFINITY;
      int neighborPolygonIndex = -1;
      int neighborPolygonVertexIndex = -1;
      double epsilon = 1e-7;

      convexPolygonLoop:
      for (int i = 0; i < convexPolygons.size(); i++)
      {
         if (i == polygonIndex)
         {
            continue;
         }

         ConvexPolygon2D polygon = convexPolygons.get(i);
         for (int j = 0; j < polygon.getNumberOfVertices(); j++)
         {
            Point2DReadOnly candidateVertex = polygon.getVertex(j);
            if (candidateVertex.epsilonEquals(vertex, epsilon))
            {
               Point2DReadOnly nextCandidateVertex = polygon.getNextVertex(j);
               double ux = nextCandidateVertex.getX() - candidateVertex.getX();
               double uy = nextCandidateVertex.getY() - candidateVertex.getY();

               double dotProduct = (ux * vx + uy * vy) / EuclidCoreTools.norm(ux, uy);
               double crossProduct = ux * vy - uy * vx;
               if (crossProduct < 0.0)
               {
                  dotProduct = -2.0 - dotProduct;
               }

               if (dotProduct > maxDotProduct)
               {
                  maxDotProduct = dotProduct;
                  neighborPolygonIndex = i;
                  neighborPolygonVertexIndex = j;
               }

               continue convexPolygonLoop;
            }
         }
      }

      if (neighborPolygonIndex >= 0)
      {
         return new int[]{neighborPolygonIndex, neighborPolygonVertexIndex};
      }
      else
      {
         return null;
      }
   }

   /**
    * @return a full depth copy of this region. The copy can be entirely modified without
    *         interfering with this region.
    */
   public PlanarRegion copy()
   {
      PlanarRegion planarRegion = new PlanarRegion();
      planarRegion.set(this);
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
    * @param transform transform from current frame to desired frame
    */
   public void applyTransform(RigidBodyTransformReadOnly transform)
   {
      transform.transform(fromLocalToWorldTransform);
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
      ret.getPoint().set(fromLocalToWorldTransform.getM03(), fromLocalToWorldTransform.getM13(), fromLocalToWorldTransform.getM23());
      ret.getNormal().set(fromLocalToWorldTransform.getM02(), fromLocalToWorldTransform.getM12(), fromLocalToWorldTransform.getM22());
      return ret;
   }

   public void projectOntoPlane(Vector4D plane)
   {
      // Update Map Region Normal and Origin
      UnitVector3DReadOnly futureNormal = new UnitVector3D(plane.getX(), plane.getY(), plane.getZ());
      Point3DReadOnly futureOrigin = GeometryTools.projectPointOntoPlane(plane, getPoint());

      Vector3D axis = new Vector3D();
      axis.cross(getNormal(), futureNormal);
      double angle = getNormal().angle(futureNormal);

      AxisAngle rotationToFutureRegion = new AxisAngle(axis, angle);
      Vector3D translationToFutureRegion = new Vector3D();
      translationToFutureRegion.sub(futureOrigin, getPoint());

      RigidBodyTransform transform = new RigidBodyTransform();
      transform.appendOrientation(rotationToFutureRegion);
      transform.appendTranslation(translationToFutureRegion);

      applyTransform(transform);
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

   public void updateArea()
   {
      area = PlanarRegionTools.computePlanarRegionArea(this);
   }

   public ConvexPolygonTools getConvexPolygonTools()
   {
      return convexPolygonTools;
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

   public String getDebugString()
   {
      return String.format("Regions ID: %d\nArea: %.2f\nConcave Hull Size: %d\nMatched: %d\nOrigin: %s\nNormal: %s\nTime: %d",
                    regionId,
                    getArea(),
                    getConcaveHullSize(),
                    getNumberOfTimesMatched(),
                    String.format("%.2f, %.2f, %.2f", origin.getX(), origin.getY(), origin.getZ()),
                    String.format("%.2f, %.2f, %.2f", normal.getX(), normal.getY(), normal.getZ()),
                    getTickOfLastMeasurement());
   }

   public Point3D getConcaveHullPoint3DInWorld(int index)
   {
      Point2D pointInPlane = concaveHullsVertices.get(index);
      Point3D pointInWorld = new Point3D(pointInPlane.getX(), pointInPlane.getY(), 0.0);
      pointInWorld.applyTransform(fromLocalToWorldTransform);
      return pointInWorld;
   }

   public int getNumberOfTimesMatched()
   {
      return numberOfTimesMatched;
   }

   public void incrementNumberOfTimesMatched()
   {
      numberOfTimesMatched++;
   }

   public double getArea()
   {
      return area;
   }

   public void setArea(double area)
   {
      this.area = area;
   }

   public int getTickOfLastMeasurement()
   {
      return tickOfLastMeasurement;
   }

   public void setTickOfLastMeasurement(int tickOfLastMeasurement)
   {
      this.tickOfLastMeasurement = tickOfLastMeasurement;
   }
}