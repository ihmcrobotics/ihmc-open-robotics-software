package us.ihmc.communication.packets;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.idl.PreallocatedList;
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

      PlanarRegionMessage planarRegionMessage = new PlanarRegionMessage();
      planarRegionMessage.setRegionId(regionId);
      planarRegionMessage.regionOrigin = regionOrigin;
      planarRegionMessage.regionNormal = regionNormal;

      Polygon2DMessage concaveHullMessage = planarRegionMessage.concaveHull;

      for (int vertexIndex = 0; vertexIndex < planarRegion.getConcaveHullSize(); vertexIndex++)
      {
         concaveHullMessage.getVertices().add().set(planarRegion.getConcaveHullVertex(vertexIndex));
      }

      for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
      {
         ConvexPolygon2D convexPolygon = planarRegion.getConvexPolygon(polygonIndex);
         Polygon2DMessage convexPolygonMessage = planarRegionMessage.convexPolygons.add();

         for (int vertexIndex = 0; vertexIndex < convexPolygon.getNumberOfVertices(); vertexIndex++)
         {
            convexPolygonMessage.getVertices().add().set(convexPolygon.getVertex(vertexIndex));
         }
      }

      return planarRegionMessage;
   }

   public static PlanarRegion convertToPlanarRegion(PlanarRegionMessage planarRegionMessage)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();

      Vector3D regionOrigin = new Vector3D(planarRegionMessage.getRegionOrigin());
      Vector3D regionNormal = new Vector3D(planarRegionMessage.getRegionNormal());
      AxisAngle regionOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(regionNormal);
      transformToWorld.set(regionOrientation, regionOrigin);

      Polygon2DMessage concaveHullMessage = planarRegionMessage.getConcaveHull();
      Point2D[] concaveHullVertices = new Point2D[concaveHullMessage.getVertices().size()];

      for (int vertexIndex = 0; vertexIndex < concaveHullMessage.getVertices().size(); vertexIndex++)
      {
         concaveHullVertices[vertexIndex] = new Point2D(concaveHullMessage.getVertices().get(vertexIndex));
      }

      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();
      PreallocatedList<Polygon2DMessage> convexPolygonsMessage = planarRegionMessage.getConvexPolygons();
      for (int polygonIndex = 0; polygonIndex < convexPolygonsMessage.size(); polygonIndex++)
      {
         ConvexPolygon2D convexPolygon = new ConvexPolygon2D(convexPolygonsMessage.get(polygonIndex).getVertices().toArray());
         convexPolygons.add(convexPolygon);
      }

      PlanarRegion planarRegion = new PlanarRegion(transformToWorld, concaveHullVertices, convexPolygons);
      planarRegion.setRegionId(planarRegionMessage.getRegionId());
      return planarRegion;
   }

   public static PlanarRegionsListMessage convertToPlanarRegionsListMessage(PlanarRegionsList planarRegionsList)
   {
      List<PlanarRegionMessage> planarRegionMessages = new ArrayList<>();

      for (int regionIndex = 0; regionIndex < planarRegionsList.getNumberOfPlanarRegions(); regionIndex++)
         planarRegionMessages.add(convertToPlanarRegionMessage(planarRegionsList.getPlanarRegion(regionIndex)));

      return MessageTools.createPlanarRegionsListMessage(planarRegionMessages);
   }

   public static PlanarRegionsList convertToPlanarRegionsList(PlanarRegionsListMessage planarRegionsListMessage)
   {
      List<PlanarRegion> planarRegions = new ArrayList<>();
      PreallocatedList<PlanarRegionMessage> planarRegionMessages = planarRegionsListMessage.getPlanarRegions();

      for (int regionIndex = 0; regionIndex < planarRegionMessages.size(); regionIndex++)
         planarRegions.add(convertToPlanarRegion(planarRegionMessages.get(regionIndex)));

      return new PlanarRegionsList(planarRegions);
   }
}
