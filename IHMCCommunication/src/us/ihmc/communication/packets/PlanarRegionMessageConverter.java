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

      Point2f[] concaveHullVertices = new Point2f[planarRegion.getConcaveHull().size()];
      for (int vertexIndex = 0; vertexIndex < planarRegion.getConcaveHull().size(); vertexIndex++)
         concaveHullVertices[vertexIndex] = new Point2f(planarRegion.getConcaveHull().get(vertexIndex));

      List<Point2f[]> convexPolygonsVertices = new ArrayList<>();

      for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
      {
         ConvexPolygon2d convexPolygon = planarRegion.getConvexPolygon(polygonIndex);
         Point2f[] vertices = new Point2f[convexPolygon.getNumberOfVertices()];
         for (int vertexIndex = 0; vertexIndex < convexPolygon.getNumberOfVertices(); vertexIndex++)
         {
            vertices[vertexIndex] = new Point2f(convexPolygon.getVertex(vertexIndex));
         }
      }

      PlanarRegionMessage planarRegionMessage = new PlanarRegionMessage(regionOrigin, regionNormal, concaveHullVertices, convexPolygonsVertices);
      planarRegionMessage.setRegionId(regionId);
      return planarRegionMessage;
   }

   public static PlanarRegion convertToPlanarRegion(PlanarRegionMessage planarRegionMessage)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      List<ConvexPolygon2d> planarRegionConvexPolygons = new ArrayList<>();

      Vector3d regionOrigin = new Vector3d(planarRegionMessage.getRegionOrigin());
      Vector3d regionNormal = new Vector3d(planarRegionMessage.getRegionNormal());
      AxisAngle4d regionOrientation = GeometryTools.getRotationBasedOnNormal(regionNormal);
      transformToWorld.set(regionOrientation, regionOrigin);

      List<Point2d> concaveHullVertices = new ArrayList<>();
      for (Point2f vertex : planarRegionMessage.concaveHullVertices)
         concaveHullVertices.add(new Point2d(vertex));

      List<Point2f[]> convexPolygonsVertices = planarRegionMessage.getConvexPolygonsVertices();
      for (int polygonIndex = 0; polygonIndex < convexPolygonsVertices.size(); polygonIndex++)
      {
         ConvexPolygon2d convexPolygon = new ConvexPolygon2d(convexPolygonsVertices.get(polygonIndex));
         planarRegionConvexPolygons.add(convexPolygon);
      }

      PlanarRegion planarRegion = new PlanarRegion(transformToWorld, concaveHullVertices, planarRegionConvexPolygons);
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
