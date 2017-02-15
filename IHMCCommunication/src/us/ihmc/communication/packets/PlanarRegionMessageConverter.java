package us.ihmc.communication.packets;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point2d;
import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class PlanarRegionMessageConverter
{
   public static PlanarRegionMessage convertToPlanarRegionMessage(PlanarRegion planarRegion)
   {
      int regionId = planarRegion.getRegionId();
      Point3f regionOrigin = new Point3f();
      Vector3f regionNormal = new Vector3f();

      planarRegion.getPointInRegion(regionOrigin);
      planarRegion.getNormal(regionNormal);

      List<Point2f[]> concaveHullsVertices = new ArrayList<>();

      for (int hullIndex = 0; hullIndex < planarRegion.getNumberOfConcaveHulls(); hullIndex++)
      {
         Point2d[] hullVertices = planarRegion.getConcaveHull(hullIndex);
         Point2f[] messageHullVertices = new Point2f[hullVertices.length];

         for (int vertexIndex = 0; vertexIndex < hullVertices.length; vertexIndex++)
         {
            messageHullVertices[vertexIndex] = new Point2f(hullVertices[vertexIndex]);
         }
         concaveHullsVertices.add(messageHullVertices);
      }

      List<Point2f[]> convexPolygonsVertices = new ArrayList<>();

      for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
      {
         ConvexPolygon2d convexPolygon = planarRegion.getConvexPolygon(polygonIndex);
         Point2f[] vertices = new Point2f[convexPolygon.getNumberOfVertices()];
         for (int vertexIndex = 0; vertexIndex < convexPolygon.getNumberOfVertices(); vertexIndex++)
         {
            vertices[vertexIndex] = new Point2f(convexPolygon.getVertex(vertexIndex));
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

      Vector3d regionOrigin = new Vector3d(planarRegionMessage.getRegionOrigin());
      Vector3d regionNormal = new Vector3d(planarRegionMessage.getRegionNormal());
      AxisAngle4d regionOrientation = GeometryTools.getAxisAngleFromZUpToVector(regionNormal);
      transformToWorld.set(regionOrientation, regionOrigin);

      List<Point2f[]> messageHullsVertices = planarRegionMessage.getConcaveHullsVertices();
      List<Point2d[]> concaveHullsVertices = new ArrayList<>();
      for (int hullIndex = 0; hullIndex < messageHullsVertices.size(); hullIndex++)
      {
         Point2f[] messageHullVertices = messageHullsVertices.get(hullIndex);
         Point2d[] hullVertices = new Point2d[messageHullVertices.length];
         for (int vertexIndex = 0; vertexIndex < messageHullVertices.length; vertexIndex++)
         {
            hullVertices[vertexIndex] = new Point2d(messageHullVertices[vertexIndex]);
         }
         concaveHullsVertices.add(hullVertices);
      }

      List<ConvexPolygon2d> planarRegionConvexPolygons = new ArrayList<>();
      List<Point2f[]> convexPolygonsVertices = planarRegionMessage.getConvexPolygonsVertices();
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
