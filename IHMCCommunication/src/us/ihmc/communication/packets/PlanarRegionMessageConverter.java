package us.ihmc.communication.packets;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.AxisAngle4d;
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
      Point3f regionOrigin = new Point3f();
      Vector3f regionNormal = new Vector3f();

      planarRegion.getPointInRegion(regionOrigin);
      planarRegion.getNormal(regionNormal);

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

      return new PlanarRegionMessage(regionOrigin, regionNormal, convexPolygonsVertices);
   }

   public static PlanarRegion convertToPlanarRegion(PlanarRegionMessage planarRegionMessage)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      List<ConvexPolygon2d> planarRegionConvexPolygons = new ArrayList<>();

      Vector3d regionOrigin = new Vector3d(planarRegionMessage.getRegionOrigin());
      Vector3d regionNormal = new Vector3d(planarRegionMessage.getRegionNormal());
      AxisAngle4d regionOrientation = GeometryTools.getRotationBasedOnNormal(regionNormal);
      transformToWorld.set(regionOrientation, regionOrigin);

      List<Point2f[]> convexPolygonsVertices = planarRegionMessage.getConvexPolygonsVertices();
      for (int polygonIndex = 0; polygonIndex < convexPolygonsVertices.size(); polygonIndex++)
      {
         ConvexPolygon2d convexPolygon = new ConvexPolygon2d(convexPolygonsVertices.get(polygonIndex));
         planarRegionConvexPolygons.add(convexPolygon);
      }

      return new PlanarRegion(transformToWorld, planarRegionConvexPolygons);
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
