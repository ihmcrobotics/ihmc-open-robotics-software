package us.ihmc.communication.packets;

import java.util.ArrayList;
import java.util.List;

import perception_msgs.msg.dds.PlanarRegionMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.FramePlanarRegionsListMessage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionMessageConverter
{
   public static PlanarRegionMessage convertToPlanarRegionMessage(PlanarRegion planarRegion)
   {
      PlanarRegionMessage message = new PlanarRegionMessage();
      message.setRegionId(planarRegion.getRegionId());
      planarRegion.getPointInRegion(message.getRegionOrigin());
      planarRegion.getNormal(message.getRegionNormal());
      RigidBodyTransform transform = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transform);
      message.getRegionOrientation().set(transform.getRotation());

      message.setConcaveHullSize(planarRegion.getConcaveHullSize());
      message.setNumberOfConvexPolygons(planarRegion.getNumberOfConvexPolygons());

      Object<Point3D> vertexBuffer = message.getVertexBuffer();
      vertexBuffer.clear();

      for (int vertexIndex = 0; vertexIndex < planarRegion.getConcaveHullSize(); vertexIndex++)
      {
         vertexBuffer.add().set(planarRegion.getConcaveHullVertex(vertexIndex), 0.0);
      }

      for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
      {
         ConvexPolygon2D convexPolygon = planarRegion.getConvexPolygon(polygonIndex);
         message.getConvexPolygonsSize().add(convexPolygon.getNumberOfVertices());

         for (int vertexIndex = 0; vertexIndex < convexPolygon.getNumberOfVertices(); vertexIndex++)
         {
            vertexBuffer.add().set(convexPolygon.getVertex(vertexIndex), 0.0);
         }
      }

      return message;
   }

   public static PlanarRegion convertToPlanarRegion(PlanarRegionMessage message)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();

      if (Math.abs(AngleTools.trimAngleMinusPiToPi(message.getRegionOrientation().getAngle())) < 1.0e-3)
      {
         Vector3D regionNormal = new Vector3D(message.getRegionNormal());
         AxisAngle regionOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(regionNormal);
         transformToWorld.set(regionOrientation, message.getRegionOrigin());
      }
      else
      {
         transformToWorld.set(message.getRegionOrientation(), message.getRegionOrigin());
      }

      Object<Point3D> vertexBuffer = message.getVertexBuffer();

      List<Point2D> concaveHullVertices = new ArrayList<>();
      int vertexIndex = 0;
      int upperBound = message.getConcaveHullSize();

      for (; vertexIndex < upperBound; vertexIndex++)
      {
         concaveHullVertices.add(new Point2D(vertexBuffer.get(vertexIndex)));
      }

      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();

      for (int polygonIndex = 0; polygonIndex < message.getNumberOfConvexPolygons(); polygonIndex++)
      {
         upperBound += message.getConvexPolygonsSize().get(polygonIndex);
         ConvexPolygon2D convexPolygon = new ConvexPolygon2D();

         for (; vertexIndex < upperBound; vertexIndex++)
            convexPolygon.addVertex(vertexBuffer.get(vertexIndex));
         convexPolygon.update();
         convexPolygons.add(convexPolygon);
      }

      PlanarRegion planarRegion = new PlanarRegion(transformToWorld, concaveHullVertices, convexPolygons);
      planarRegion.setRegionId(message.getRegionId());
      return planarRegion;
   }

   public static PlanarRegionsListMessage convertToPlanarRegionsListMessage(PlanarRegion planarRegion)
   {
      PlanarRegionsList planarRegionsList = new PlanarRegionsList();
      planarRegionsList.addPlanarRegion(planarRegion);
      return convertToPlanarRegionsListMessage(planarRegionsList);
   }

   public static PlanarRegionsListMessage convertToPlanarRegionsListMessage(PlanarRegionsList planarRegionsList)
   {
      PlanarRegionsListMessage message = new PlanarRegionsListMessage();

      Object<Point3D> vertexBuffer = message.getVertexBuffer();

      vertexBuffer.clear();

      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         planarRegion.getTransformToWorld(transform);
         transform.get(message.getRegionOrientation().add(), message.getRegionOrigin().add());
         planarRegion.getNormal(message.getRegionNormal().add());
         message.getRegionId().add(planarRegion.getRegionId());

         message.getConcaveHullsSize().add(planarRegion.getConcaveHullSize());
         message.getNumberOfConvexPolygons().add(planarRegion.getNumberOfConvexPolygons());

         for (int vertexIndex = 0; vertexIndex < planarRegion.getConcaveHullSize(); vertexIndex++)
         {
            vertexBuffer.add().set(planarRegion.getConcaveHullVertex(vertexIndex), 0.0);
         }

         for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
         {
            ConvexPolygon2DReadOnly convexPolygon = planarRegion.getConvexPolygon(polygonIndex);
            message.getConvexPolygonsSize().add(convexPolygon.getNumberOfVertices());

            for (int vertexIndex = 0; vertexIndex < convexPolygon.getNumberOfVertices(); vertexIndex++)
            {
               vertexBuffer.add().set(convexPolygon.getVertex(vertexIndex), 0.0);
            }
         }
      }

      return message;
   }

   public static PlanarRegionsList convertToPlanarRegionsList(PlanarRegionsListMessage message)
   {
      if (message == null)
         return null;

      int vertexIndex = 0;
      Object<Vector3D> normals = message.getRegionNormal();
      Object<Point3D> origins = message.getRegionOrigin();

      Object<Point3D> vertexBuffer = message.getVertexBuffer();

      List<PlanarRegion> planarRegions = new ArrayList<>();

      int upperBound = 0;
      int convexPolygonIndexStart = 0;

      for (int regionIndex = 0; regionIndex < message.getConcaveHullsSize().size(); regionIndex++)
      {
         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         if (message.getRegionOrientation().isEmpty()
             || Math.abs(AngleTools.trimAngleMinusPiToPi(message.getRegionOrientation().get(regionIndex).getAngle())) < 1.0e-3)
         {
            AxisAngle regionOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(normals.get(regionIndex));
            transformToWorld.set(regionOrientation, origins.get(regionIndex));
         }
         else
         {
            transformToWorld.set(message.getRegionOrientation().get(regionIndex), message.getRegionOrigin().get(regionIndex));
         }

         upperBound += message.getConcaveHullsSize().get(regionIndex);
         List<Point2D> concaveHullVertices = new ArrayList<>();

         for (; vertexIndex < upperBound; vertexIndex++)
         {
            concaveHullVertices.add(new Point2D(vertexBuffer.get(vertexIndex)));
         }

         List<ConvexPolygon2D> convexPolygons = new ArrayList<>();
         int polygonIndex = 0;
         for (; polygonIndex < message.getNumberOfConvexPolygons().get(regionIndex); polygonIndex++)
         {
            upperBound += message.getConvexPolygonsSize().get(convexPolygonIndexStart + polygonIndex);
            ConvexPolygon2D convexPolygon = new ConvexPolygon2D();

            for (; vertexIndex < upperBound; vertexIndex++)
               convexPolygon.addVertex(vertexBuffer.get(vertexIndex));
            convexPolygon.update();
            convexPolygons.add(convexPolygon);
         }
         convexPolygonIndexStart += polygonIndex;

         PlanarRegion planarRegion = new PlanarRegion(transformToWorld, concaveHullVertices, convexPolygons);
         planarRegion.setRegionId(message.getRegionId().get(regionIndex));
         planarRegions.add(planarRegion);
      }

      return new PlanarRegionsList(planarRegions);
   }

   public static PlanarRegionsListMessage createPlanarRegionsListMessage(List<PlanarRegionMessage> planarRegions)
   {
      PlanarRegionsListMessage message = new PlanarRegionsListMessage();
      for (PlanarRegionMessage planarRegionMessage : planarRegions)
      {
         message.getRegionId().add(planarRegionMessage.getRegionId());
         message.getRegionOrigin().add().set(planarRegionMessage.getRegionOrigin());
         message.getRegionOrientation().add().set(planarRegionMessage.getRegionOrientation());
         message.getRegionNormal().add().set(planarRegionMessage.getRegionNormal());
         message.getConcaveHullsSize().add(planarRegionMessage.getConcaveHullSize());
         message.getNumberOfConvexPolygons().add(planarRegionMessage.getNumberOfConvexPolygons());
         message.getConvexPolygonsSize().addAll(planarRegionMessage.getConvexPolygonsSize());
         for (int i = 0; i < planarRegionMessage.getVertexBuffer().size(); i++)
            message.getVertexBuffer().add().set(planarRegionMessage.getVertexBuffer().get(i));
      }
      return message;
   }

   public static FramePlanarRegionsList convertToFramePlanarRegionsList(FramePlanarRegionsListMessage message)
   {
      FramePlanarRegionsList framePlanarRegionsListToReturn = new FramePlanarRegionsList();

      PlanarRegionsList planarRegionsList = convertToPlanarRegionsList(message.getPlanarRegions());
      framePlanarRegionsListToReturn.setPlanarRegionsList(planarRegionsList);
      framePlanarRegionsListToReturn.getSensorToWorldFrameTransform().set(message.getSensorOrientation(), message.getSensorPosition());

      return framePlanarRegionsListToReturn;
   }

   public static PlanarRegionsList convertToPlanarRegionsListInWorld(FramePlanarRegionsListMessage message)
   {
      PlanarRegionsList planarRegionsList = convertToPlanarRegionsList(message.getPlanarRegions());
      planarRegionsList.applyTransform(new RigidBodyTransform(message.getSensorOrientation(), message.getSensorPosition()));
      return planarRegionsList;
   }

   public static FramePlanarRegionsListMessage convertToFramePlanarRegionsListMessage(FramePlanarRegionsList frameRegions)
   {
      FramePlanarRegionsListMessage messageToReturn = new FramePlanarRegionsListMessage();

      PlanarRegionsListMessage planarRegionsListMessage = convertToPlanarRegionsListMessage(frameRegions.getPlanarRegionsList());
      messageToReturn.getPlanarRegions().set(planarRegionsListMessage);
      messageToReturn.getSensorPosition().set(frameRegions.getSensorToWorldFrameTransform().getTranslation());
      messageToReturn.getSensorOrientation().set(frameRegions.getSensorToWorldFrameTransform().getRotation());

      return messageToReturn;
   }
}
