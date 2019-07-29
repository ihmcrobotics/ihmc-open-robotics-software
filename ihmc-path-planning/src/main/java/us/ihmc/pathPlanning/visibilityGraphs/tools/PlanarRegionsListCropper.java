package us.ihmc.pathPlanning.visibilityGraphs.tools;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionsListCropper
{
   public static class Result
   {
      boolean fullyDeleteRegion = true;
      PlanarRegion croppedOrKeptRegion;
   }

   public static PlanarRegionsList cropPlanarRegions(Plane3D p1, Plane3D p2, Plane3D p3, Plane3D p4, PlanarRegionsList map)
   {
      PlanarRegionsList croppedRegions = new PlanarRegionsList();

      for (PlanarRegion mapRegion : map.getPlanarRegionsAsList())
      {
         boolean atLeastPartiallyAboveOrOnEachPlane = true;
         atLeastPartiallyAboveOrOnEachPlane &= !PlanarRegionTools.allVerticesAreBelowPlane3D(p1, mapRegion);
         atLeastPartiallyAboveOrOnEachPlane &= !PlanarRegionTools.allVerticesAreBelowPlane3D(p2, mapRegion);
         atLeastPartiallyAboveOrOnEachPlane &= !PlanarRegionTools.allVerticesAreBelowPlane3D(p3, mapRegion);
         atLeastPartiallyAboveOrOnEachPlane &= !PlanarRegionTools.allVerticesAreBelowPlane3D(p4, mapRegion);

         if (atLeastPartiallyAboveOrOnEachPlane) // at least part of the region is inside/above/on all planes
         {

         }
      }

      return croppedRegions;
   }

   public static PlanarRegionsList cropByPlane(Plane3D plane, PlanarRegionsList map)
   {
      PlanarRegionsList croppedRegions = new PlanarRegionsList();

      for (PlanarRegion mapRegion : map.getPlanarRegionsAsList())
      {

      }

      return croppedRegions;
   }


   public static Result cropRegionByPlane(Plane3D plane, PlanarRegion region)
   {
      Result result = new Result();

      Point3D vertexInWorld = new Point3D();
      Point3D centroidInWorld = new Point3D();
      RigidBodyTransformReadOnly regionToWorld = region.getTransformToWorld();
      Point2D intersectionPoint1 = new Point2D();
      Point2D intersectionPoint2 = new Point2D();
      Line3D line3D = new Line3D();
      Line2D line2D = new Line2D();

      PlanarRegion croppedOrKeptRegion = new PlanarRegion();
      croppedOrKeptRegion.setTransformOnly(region);

      for (ConvexPolygon2D convexPolygon : region.getConvexPolygons())
      {
         boolean convexPolygonHasPointsAbove = false;
         boolean convexPolygonHasPointsBelow = false;
         for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
         {
            Point2DReadOnly vertex = convexPolygon.getVertex(i);
            vertexInWorld.set(vertex.getX(), vertex.getY(), 0.0);
            vertexInWorld.applyTransform(regionToWorld);

            boolean vertexIsAbovePlane = EuclidGeometryTools.isPoint3DAbovePlane3D(vertexInWorld, plane.getPoint(), plane.getNormal());
            boolean vertexIsBelowPlane = EuclidGeometryTools.isPoint3DBelowPlane3D(vertexInWorld, plane.getPoint(), plane.getNormal());
            convexPolygonHasPointsAbove |= vertexIsAbovePlane;
            convexPolygonHasPointsBelow |= vertexIsBelowPlane;

            result.fullyDeleteRegion &= !convexPolygonHasPointsAbove;
         }

         if (convexPolygonHasPointsAbove && convexPolygonHasPointsBelow)
         {
            GeometryTools.getIntersectionBetweenTwoPlanes(plane, region.getPlane(), line3D); // parallel is impossible after above; ignore return value
            line3D.applyTransform(region.getTransformToLocal());
            line2D.set(line3D);
            int numberOfIntersectionPoints = convexPolygon.intersectionWith(line2D, intersectionPoint1, intersectionPoint2);

            if (numberOfIntersectionPoints == 0)
            {
               throw new RuntimeException("Should be impossible. Check logic.");
            }
            else if (numberOfIntersectionPoints == 1) // single point; decide to keep or delete
            {
               Point2DReadOnly centroid = convexPolygon.getCentroid();  // if centroid above plane, keep
               centroidInWorld.set(centroid.getX(), centroid.getY(), 0.0);
               centroidInWorld.applyTransform(regionToWorld);
               if (EuclidGeometryTools.isPoint3DAbovePlane3D(centroidInWorld, plane.getPoint(), plane.getNormal()))
               {
                  croppedOrKeptRegion.getConvexPolygons().add(new ConvexPolygon2D(convexPolygon));
               }
            }
            else if (numberOfIntersectionPoints == 2)
            {

            }


            //            ConvexPolygonTools.cutPolygonWithLine(, , )
         }
         else if (convexPolygonHasPointsAbove)
         {
            croppedOrKeptRegion.getConvexPolygons().add(new ConvexPolygon2D(convexPolygon));
         }
      }

      result.croppedOrKeptRegion = croppedOrKeptRegion;
      return result;
   }


   public static Result processPlane(PlanarRegion region, Plane3D p1, Plane3D p2, Plane3D p3, Plane3D p4)
   {
      Result result = new Result();

      Point3D vertexInWorld = new Point3D();
      RigidBodyTransformReadOnly regionToWorld = region.getTransformToWorld();
      Point2D intersectionPoint1 = new Point2D();
      Point2D intersectionPoint2 = new Point2D();
      Line3D line3D = new Line3D();
      Line2D line2D = new Line2D();

      for (ConvexPolygon2D convexPolygon : region.getConvexPolygons())
      {
         boolean convexPolygonPartiallyInside = true;
         for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
         {
            Point2DReadOnly vertex = convexPolygon.getVertex(i);
            vertexInWorld.set(vertex.getX(), vertex.getY(), 0.0);
            vertexInWorld.applyTransform(regionToWorld);

            boolean vertexIsInside = !EuclidGeometryTools.isPoint3DBelowPlane3D(vertexInWorld, p1.getPoint(), p1.getNormal());
            vertexIsInside &= !EuclidGeometryTools.isPoint3DBelowPlane3D(vertexInWorld, p2.getPoint(), p2.getNormal());
            vertexIsInside &= !EuclidGeometryTools.isPoint3DBelowPlane3D(vertexInWorld, p3.getPoint(), p3.getNormal());
            vertexIsInside &= !EuclidGeometryTools.isPoint3DBelowPlane3D(vertexInWorld, p4.getPoint(), p4.getNormal());

            convexPolygonPartiallyInside &= vertexIsInside;
//            result.fullyKeepRegion &= convexPolygonPartiallyInside;
         }

         if (convexPolygonPartiallyInside)
         {
            boolean parallel = !GeometryTools.getIntersectionBetweenTwoPlanes(p1, region.getPlane(), line3D);
            if (!parallel)
            {
               line3D.applyTransform(region.getTransformToLocal());
               line2D.set(line3D);
               int intersectionPoints = convexPolygon.intersectionWith(line2D, intersectionPoint1, intersectionPoint2);
            }


//            ConvexPolygonTools.cutPolygonWithLine(, , )
         }
      }

      return result;
   }
}
