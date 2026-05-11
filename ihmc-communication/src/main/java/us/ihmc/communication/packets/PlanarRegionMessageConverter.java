package us.ihmc.communication.packets;

import java.util.ArrayList;
import java.util.List;

import perception_msgs.PlanarRegionMessage;
import perception_msgs.PlanarRegionsListMessage;
import perception_msgs.FramePlanarRegionsListMessage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
// IDLSequence is now in fastddsjava
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

      Point3D pointInRegion = new Point3D();
      planarRegion.getPointInRegion(pointInRegion);
      MessageTools.toMessage(pointInRegion, message.getRegionOrigin());

      Vector3D normal = new Vector3D();
      planarRegion.getNormal(normal);
      MessageTools.toMessage(normal, message.getRegionNormal());

      RigidBodyTransform transform = new RigidBodyTransform();
      planarRegion.getTransformToWorld(transform);
      MessageTools.toMessage(transform.getRotation(), message.getRegionOrientation());

      message.setConcaveHullSize(planarRegion.getConcaveHullSize());
      message.setNumberOfConvexPolygons(planarRegion.getNumberOfConvexPolygons());

      us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence<us.ihmc.euclid.jros2.messages.EuclidPoint3DMessage> vertexBuffer = message.getVertexBuffer();
      vertexBuffer.clear();

      for (int vertexIndex = 0; vertexIndex < planarRegion.getConcaveHullSize(); vertexIndex++)
      {
         Point3D vertex = new Point3D();
         vertex.set(planarRegion.getConcaveHullVertex(vertexIndex), 0.0);
         MessageTools.toMessage(vertex, vertexBuffer.add());
      }

      for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
      {
         ConvexPolygon2D convexPolygon = planarRegion.getConvexPolygon(polygonIndex);
         message.getConvexPolygonsSize().add(convexPolygon.getNumberOfVertices());

         for (int vertexIndex = 0; vertexIndex < convexPolygon.getNumberOfVertices(); vertexIndex++)
         {
            Point3D vertex = new Point3D();
            vertex.set(convexPolygon.getVertex(vertexIndex), 0.0);
            MessageTools.toMessage(vertex, vertexBuffer.add());
         }
      }

      return message;
   }

   public static PlanarRegion convertToPlanarRegion(PlanarRegionMessage message)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();

      us.ihmc.euclid.tuple4D.Quaternion quaternionTemp = new us.ihmc.euclid.tuple4D.Quaternion();
      MessageTools.fromMessage(message.getRegionOrientation(), quaternionTemp);
      if (Math.abs(AngleTools.trimAngleMinusPiToPi(quaternionTemp.getAngle())) < 1.0e-3)
      {
         Vector3D regionNormal = new Vector3D();
         MessageTools.fromMessage(message.getRegionNormal(), regionNormal);
         AxisAngle regionOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(regionNormal);
         Point3D regionOrigin = new Point3D();
         MessageTools.fromMessage(message.getRegionOrigin(), regionOrigin);
         transformToWorld.set(regionOrientation, regionOrigin);
      }
      else
      {
         us.ihmc.euclid.tuple4D.Quaternion quat = new us.ihmc.euclid.tuple4D.Quaternion();
         MessageTools.fromMessage(message.getRegionOrientation(), quat);
         Point3D regionOrigin = new Point3D();
         MessageTools.fromMessage(message.getRegionOrigin(), regionOrigin);
         transformToWorld.set(quat, regionOrigin);
      }

      us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence<us.ihmc.euclid.jros2.messages.EuclidPoint3DMessage> vertexBuffer = message.getVertexBuffer();

      List<Point2D> concaveHullVertices = new ArrayList<>();
      int vertexIndex = 0;
      int upperBound = message.getConcaveHullSize();

      for (; vertexIndex < upperBound; vertexIndex++)
      {
         Point3D vertex = new Point3D();
         MessageTools.fromMessage(vertexBuffer.get(vertexIndex), vertex);
         concaveHullVertices.add(new Point2D(vertex));
      }

      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();

      for (int polygonIndex = 0; polygonIndex < message.getNumberOfConvexPolygons(); polygonIndex++)
      {
         upperBound += message.getConvexPolygonsSize().get(polygonIndex);
         ConvexPolygon2D convexPolygon = new ConvexPolygon2D();

         for (; vertexIndex < upperBound; vertexIndex++)
         {
            Point3D vertex = new Point3D();
            MessageTools.fromMessage(vertexBuffer.get(vertexIndex), vertex);
            convexPolygon.addVertex(vertex);
         }
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

      us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence<us.ihmc.euclid.jros2.messages.EuclidPoint3DMessage> vertexBuffer = message.getVertexBuffer();
      us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence<us.ihmc.euclid.jros2.messages.EuclidQuaternionMessage> orientationBuffer = message.getRegionOrientation();
      us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence<us.ihmc.euclid.jros2.messages.EuclidPoint3DMessage> originBuffer = message.getRegionOrigin();
      us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence<us.ihmc.euclid.jros2.messages.EuclidVector3DMessage> normalBuffer = message.getRegionNormal();

      vertexBuffer.clear();

      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         planarRegion.getTransformToWorld(transform);

         MessageTools.toMessage(transform.getRotation(), orientationBuffer.add());
         MessageTools.toMessage(transform.getTranslation(), originBuffer.add());

         Vector3D normal = new Vector3D();
         planarRegion.getNormal(normal);
         MessageTools.toMessage(normal, normalBuffer.add());

         message.getRegionId().add(planarRegion.getRegionId());

         message.getConcaveHullsSize().add(planarRegion.getConcaveHullSize());
         message.getNumberOfConvexPolygons().add(planarRegion.getNumberOfConvexPolygons());

         for (int vertexIndex = 0; vertexIndex < planarRegion.getConcaveHullSize(); vertexIndex++)
         {
            Point3D vertex = new Point3D();
            vertex.set(planarRegion.getConcaveHullVertex(vertexIndex), 0.0);
            MessageTools.toMessage(vertex, vertexBuffer.add());
         }

         for (int polygonIndex = 0; polygonIndex < planarRegion.getNumberOfConvexPolygons(); polygonIndex++)
         {
            ConvexPolygon2DReadOnly convexPolygon = planarRegion.getConvexPolygon(polygonIndex);
            message.getConvexPolygonsSize().add(convexPolygon.getNumberOfVertices());

            for (int vertexIndex = 0; vertexIndex < convexPolygon.getNumberOfVertices(); vertexIndex++)
            {
               Point3D vertex = new Point3D();
               vertex.set(convexPolygon.getVertex(vertexIndex), 0.0);
               MessageTools.toMessage(vertex, vertexBuffer.add());
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
      us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence<us.ihmc.euclid.jros2.messages.EuclidVector3DMessage> normals = message.getRegionNormal();
      us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence<us.ihmc.euclid.jros2.messages.EuclidPoint3DMessage> origins = message.getRegionOrigin();

      us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence<us.ihmc.euclid.jros2.messages.EuclidPoint3DMessage> vertexBuffer = message.getVertexBuffer();

      List<PlanarRegion> planarRegions = new ArrayList<>();

      int upperBound = 0;
      int convexPolygonIndexStart = 0;

      for (int regionIndex = 0; regionIndex < message.getConcaveHullsSize().size(); regionIndex++)
      {
         RigidBodyTransform transformToWorld = new RigidBodyTransform();
         boolean useNormalBasedOrientation = message.getRegionOrientation().isEmpty();
         if (!useNormalBasedOrientation)
         {
            us.ihmc.euclid.tuple4D.Quaternion quatTemp = new us.ihmc.euclid.tuple4D.Quaternion();
            MessageTools.fromMessage(message.getRegionOrientation().get(regionIndex), quatTemp);
            useNormalBasedOrientation = Math.abs(AngleTools.trimAngleMinusPiToPi(quatTemp.getAngle())) < 1.0e-3;
         }
         if (useNormalBasedOrientation)
         {
            Vector3D normal = new Vector3D();
            MessageTools.fromMessage(normals.get(regionIndex), normal);
            AxisAngle regionOrientation = EuclidGeometryTools.axisAngleFromZUpToVector3D(normal);
            Point3D origin = new Point3D();
            MessageTools.fromMessage(origins.get(regionIndex), origin);
            transformToWorld.set(regionOrientation, origin);
         }
         else
         {
            us.ihmc.euclid.tuple4D.Quaternion quat = new us.ihmc.euclid.tuple4D.Quaternion();
            MessageTools.fromMessage(message.getRegionOrientation().get(regionIndex), quat);
            Point3D origin = new Point3D();
            MessageTools.fromMessage(message.getRegionOrigin().get(regionIndex), origin);
            transformToWorld.set(quat, origin);
         }

         upperBound += message.getConcaveHullsSize().get(regionIndex);
         List<Point2D> concaveHullVertices = new ArrayList<>();

         for (; vertexIndex < upperBound; vertexIndex++)
         {
            Point3D vertex = new Point3D();
            MessageTools.fromMessage(vertexBuffer.get(vertexIndex), vertex);
            concaveHullVertices.add(new Point2D(vertex));
         }

         List<ConvexPolygon2D> convexPolygons = new ArrayList<>();
         int polygonIndex = 0;
         for (; polygonIndex < message.getNumberOfConvexPolygons().get(regionIndex); polygonIndex++)
         {
            upperBound += message.getConvexPolygonsSize().get(convexPolygonIndexStart + polygonIndex);
            ConvexPolygon2D convexPolygon = new ConvexPolygon2D();

            for (; vertexIndex < upperBound; vertexIndex++)
            {
               Point3D vertex = new Point3D();
               MessageTools.fromMessage(vertexBuffer.get(vertexIndex), vertex);
               convexPolygon.addVertex(vertex);
            }
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
         for (int i = 0; i < planarRegionMessage.getConvexPolygonsSize().size(); i++)
           message.getConvexPolygonsSize().add(planarRegionMessage.getConvexPolygonsSize().get(i));
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
      us.ihmc.euclid.tuple4D.Quaternion sensorQuat = new us.ihmc.euclid.tuple4D.Quaternion();
      Point3D sensorPos = new Point3D();
      MessageTools.fromMessage(message.getSensorOrientation(), sensorQuat);
      MessageTools.fromMessage(message.getSensorPosition(), sensorPos);
      framePlanarRegionsListToReturn.getSensorToWorldFrameTransform().set(sensorQuat, sensorPos);

      return framePlanarRegionsListToReturn;
   }

   public static PlanarRegionsList convertToPlanarRegionsListInWorld(FramePlanarRegionsListMessage message)
   {
      PlanarRegionsList planarRegionsList = convertToPlanarRegionsList(message.getPlanarRegions());
      us.ihmc.euclid.tuple4D.Quaternion sensorQuat = new us.ihmc.euclid.tuple4D.Quaternion();
      Point3D sensorPos = new Point3D();
      MessageTools.fromMessage(message.getSensorOrientation(), sensorQuat);
      MessageTools.fromMessage(message.getSensorPosition(), sensorPos);
      planarRegionsList.applyTransform(new RigidBodyTransform(sensorQuat, sensorPos));
      return planarRegionsList;
   }

   public static FramePlanarRegionsListMessage convertToFramePlanarRegionsListMessage(FramePlanarRegionsList frameRegions)
   {
      FramePlanarRegionsListMessage messageToReturn = new FramePlanarRegionsListMessage();

      PlanarRegionsListMessage planarRegionsListMessage = convertToPlanarRegionsListMessage(frameRegions.getPlanarRegionsList());
      messageToReturn.getPlanarRegions().set(planarRegionsListMessage);
      MessageTools.toMessage(frameRegions.getSensorToWorldFrameTransform().getTranslation(), messageToReturn.getSensorPosition());
      MessageTools.toMessage(frameRegions.getSensorToWorldFrameTransform().getRotation(), messageToReturn.getSensorOrientation());

      return messageToReturn;
   }
}
