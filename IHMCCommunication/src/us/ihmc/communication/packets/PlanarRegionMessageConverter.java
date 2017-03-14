package us.ihmc.communication.packets;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionMessageConverter
{
   public static PlanarRegionMessage convertToPlanarRegionMessage(PlanarRegion planarRegion)
   {
      int regionId = planarRegion.getRegionId();
      Point3D32 regionOrigin = new Point3D32();
      Vector3D32 regionNormal = new Vector3D32();

      planarRegion.getPointInRegion(regionOrigin);
      planarRegion.getNormal(regionNormal);

      List<Point2D32[]> concaveHullsVertices = new ArrayList<>();

      for (int hullIndex = 0; hullIndex < planarRegion.getNumberOfConcaveHulls(); hullIndex++)
      {
         Point2D[] hullVertices = planarRegion.getConcaveHull(hullIndex);
         Point2D32[] messageHullVertices = new Point2D32[hullVertices.length];

         for (int vertexIndex = 0; vertexIndex < hullVertices.length; vertexIndex++)
         {
            messageHullVertices[vertexIndex] = new Point2D32(hullVertices[vertexIndex]);
         }
         concaveHullsVertices.add(messageHullVertices);
      }

      List<Point2D32[]> convexPolygonsVertices = new ArrayList<>();

      for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
      {
         ConvexPolygon2d convexPolygon = planarRegion.getConvexPolygon(polygonIndex);
         Point2D32[] vertices = new Point2D32[convexPolygon.getNumberOfVertices()];
         for (int vertexIndex = 0; vertexIndex < convexPolygon.getNumberOfVertices(); vertexIndex++)
         {
            vertices[vertexIndex] = new Point2D32(convexPolygon.getVertex(vertexIndex));
         }
         convexPolygonsVertices.add(vertices);
      }

      PlanarRegionMessage planarRegionMessage = new PlanarRegionMessage(regionOrigin, regionNormal, concaveHullsVertices, convexPolygonsVertices);
      planarRegionMessage.setRegionId(regionId);
      return planarRegionMessage;
   }

   public static PlanarRegion convertToPlanarRegion(PlanarRegionMessage planarRegionMessage)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();

      Vector3D regionOrigin = new Vector3D(planarRegionMessage.getRegionOrigin());
      Vector3D regionNormal = new Vector3D(planarRegionMessage.getRegionNormal());
      AxisAngle regionOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(regionNormal);
      transformToWorld.set(regionOrientation, regionOrigin);

      List<Point2D32[]> messageHullsVertices = planarRegionMessage.getConcaveHullsVertices();
      List<Point2D[]> concaveHullsVertices = new ArrayList<>();
      for (int hullIndex = 0; hullIndex < messageHullsVertices.size(); hullIndex++)
      {
         Point2D32[] messageHullVertices = messageHullsVertices.get(hullIndex);
         Point2D[] hullVertices = new Point2D[messageHullVertices.length];
         for (int vertexIndex = 0; vertexIndex < messageHullVertices.length; vertexIndex++)
         {
            hullVertices[vertexIndex] = new Point2D(messageHullVertices[vertexIndex]);
         }
         concaveHullsVertices.add(hullVertices);
      }

      List<ConvexPolygon2d> planarRegionConvexPolygons = new ArrayList<>();
      List<Point2D32[]> convexPolygonsVertices = planarRegionMessage.getConvexPolygonsVertices();
      for (int polygonIndex = 0; polygonIndex < convexPolygonsVertices.size(); polygonIndex++)
      {
         ConvexPolygon2d convexPolygon = new ConvexPolygon2d(convexPolygonsVertices.get(polygonIndex));
         planarRegionConvexPolygons.add(convexPolygon);
      }

      PlanarRegion planarRegion = new PlanarRegion(transformToWorld, concaveHullsVertices, planarRegionConvexPolygons);
      planarRegion.setRegionId(planarRegionMessage.getRegionId());
      return planarRegion;
   }

   public static PlanarRegionsListMessage convertToPlanarRegionsListMessage(PlanarRegionsList planarRegionsList)
   {
      List<PlanarRegionMessage> planarRegionMessages = new ArrayList<>();

      for (int regionIndex = 0; regionIndex < planarRegionsList.getNumberOfPlanarRegions(); regionIndex++)
         planarRegionMessages.add(convertToPlanarRegionMessage(planarRegionsList.getPlanarRegion(regionIndex)));

      return new PlanarRegionsListMessage(planarRegionMessages);
   }

   public static PlanarRegionsList convertToPlanarRegionsList(PlanarRegionsListMessage planarRegionsListMessage)
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();
      List<PlanarRegionMessage> planarRegionMessages = planarRegionsListMessage.getPlanarRegions();

      for (int regionIndex = 0; regionIndex < planarRegionMessages.size(); regionIndex++)
         planarRegions.add(convertToPlanarRegion(planarRegionMessages.get(regionIndex)));

      return new PlanarRegionsList(planarRegions);
   }
}
