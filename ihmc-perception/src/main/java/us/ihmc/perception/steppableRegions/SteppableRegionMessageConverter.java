package us.ihmc.perception.steppableRegions;

import perception_msgs.msg.dds.*;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

import java.util.ArrayList;
import java.util.List;

public class SteppableRegionMessageConverter
{
   public static SteppableRegionMessage convertToPlanarRegionMessage(SteppableRegion steppableRegion)
   {
      SteppableRegionMessage message = new SteppableRegionMessage();

      message.setFootYaw(steppableRegion.getFootYaw());
      message.setRegionId(steppableRegion.getRegionId());
      message.getRegionOrigin().set(steppableRegion.getRegionOrigin());
      message.getRegionNormal().set(steppableRegion.getRegionNormal());
      message.getRegionOrientation().set(steppableRegion.getRegionOrientation());

      message.setConcaveHullSize(steppableRegion.getConcaveHullInRegionFrame().getNumberOfVertices());

      IDLSequence.Object<Point3D> vertexBuffer = message.getVertexBuffer();
      vertexBuffer.clear();

      for (int vertexIndex = 0; vertexIndex < steppableRegion.getConcaveHullInRegionFrame().getNumberOfVertices(); vertexIndex++)
      {
         vertexBuffer.add().set(steppableRegion.getConcaveHullInRegionFrame().getVertex(vertexIndex), 0.0);
      }

      HeightMapMessageTools.toMessage(steppableRegion.getLocalHeightMap(), message.getLocalHeightMap());

      return message;
   }

   public static SteppableRegion convertToSteppableRegion(SteppableRegionMessage message)
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

      IDLSequence.Object<Point3D> vertexBuffer = message.getVertexBuffer();

      List<Point2D> concaveHullVertices = new ArrayList<>();
      int vertexIndex = 0;
      int upperBound = message.getConcaveHullSize();

      for (; vertexIndex < upperBound; vertexIndex++)
      {
         concaveHullVertices.add(new Point2D(vertexBuffer.get(vertexIndex)));
      }

      double footYaw = message.getFootYaw();
      SteppableRegion steppableRegion = new SteppableRegion(transformToWorld,
                                                            concaveHullVertices,
                                                            footYaw);
      steppableRegion.setRegionId(message.getRegionId());

      if (message.getLocalHeightMap().getHeights().size() > 0)
      {
         HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(message.getLocalHeightMap());
         steppableRegion.setLocalHeightMap(heightMapData);
      }

      return steppableRegion;
   }

   public static SteppableRegionsListMessage convertToSteppableRegionsListMessage(SteppableRegionsList steppableRegionsList)
   {
      SteppableRegionsListMessage message = new SteppableRegionsListMessage();

      IDLSequence.Object<Point3D> vertexBuffer = message.getVertexBuffer();
      IDLSequence.Object<Point3D> originBuffer = message.getRegionOrigin();
      IDLSequence.Object<Vector3D> normalBuffer = message.getRegionNormal();
      IDLSequence.Object<Quaternion> orientationBuffer = message.getRegionOrientation();
      vertexBuffer.clear();
      originBuffer.clear();
      normalBuffer.clear();
      orientationBuffer.clear();

      IDLSequence.Integer regionIdBuffer = message.getRegionId();
      IDLSequence.Integer hullSizeBuffer = message.getConcaveHullsSize();
      regionIdBuffer.clear();
      hullSizeBuffer.clear();

      IDLSequence.Object<HeightMapMessage> heightMapBuffer = message.getLocalHeightMap();
      heightMapBuffer.clear();

      double footYaw = steppableRegionsList.getFootYaw();

      for (SteppableRegion steppableRegion : steppableRegionsList.getSteppableRegionsAsList())
      {
         regionIdBuffer.add(steppableRegion.getRegionId());
         originBuffer.add().set(steppableRegion.getRegionOrigin());
         normalBuffer.add().set(steppableRegion.getRegionNormal());
         orientationBuffer.add().set(steppableRegion.getRegionOrientation());

         hullSizeBuffer.add(steppableRegion.getConcaveHullInRegionFrame().getNumberOfVertices());

         for (int vertexIndex = 0; vertexIndex < steppableRegion.getConcaveHullInRegionFrame().getNumberOfVertices(); vertexIndex++)
         {
            vertexBuffer.add().set(steppableRegion.getConcaveHullInRegionFrame().getVertex(vertexIndex), 0.0);
         }

         if (!MathTools.epsilonEquals(steppableRegion.getFootYaw(), footYaw, 1e-5))
            throw new RuntimeException("Yaws are not equal.");

         HeightMapMessage heightMapMessage = heightMapBuffer.add();
         if (steppableRegion.getLocalHeightMap() != null)
            HeightMapMessageTools.toMessage(steppableRegion.getLocalHeightMap(), heightMapMessage);
         else
            HeightMapMessageTools.clear(heightMapMessage);
      }
      message.setFootYaw(footYaw);

      return message;
   }

   public static SteppableRegionsList convertToSteppableRegionsList(SteppableRegionsListMessage message)
   {
      if (message == null)
         return null;

      int vertexIndex = 0;
      IDLSequence.Object<Vector3D> normals = message.getRegionNormal();
      IDLSequence.Object<Point3D> origins = message.getRegionOrigin();

      IDLSequence.Object<Point3D> vertexBuffer = message.getVertexBuffer();

      List<SteppableRegion> steppableRegions = new ArrayList<>();

      int upperBound = 0;
      double footYaw = message.getFootYaw();

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

         SteppableRegion steppableRegion = new SteppableRegion(transformToWorld,
                                                               concaveHullVertices,
                                                               footYaw);
         steppableRegion.setRegionId(message.getRegionId().get(regionIndex));
         steppableRegions.add(steppableRegion);

         if (message.getLocalHeightMap().get(regionIndex).getHeights().size() > 0)
         {
            HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(message.getLocalHeightMap().get(regionIndex));
            steppableRegion.setLocalHeightMap(heightMapData);
         }
      }

      return new SteppableRegionsList(footYaw, steppableRegions);
   }

   public static SteppableRegionsListCollectionMessage convertToSteppableRegionsListCollectionMessage(SteppableRegionsListCollection steppableRegionListCollection)
   {
      SteppableRegionsListCollectionMessage message = new SteppableRegionsListCollectionMessage();
      convertToSteppableRegionsListCollectionMessage(steppableRegionListCollection, message);

      return message;
   }

   public static void convertToSteppableRegionsListCollectionMessage(SteppableRegionsListCollection steppableRegionListCollection,
                                                                     SteppableRegionsListCollectionMessage messageToPack)
   {
      IDLSequence.Object<Point3D> vertexBuffer = messageToPack.getVertexBuffer();
      IDLSequence.Object<Point3D> originBuffer = messageToPack.getRegionOrigin();
      IDLSequence.Object<Vector3D> normalBuffer = messageToPack.getRegionNormal();
      IDLSequence.Object<Quaternion> orientationBuffer = messageToPack.getRegionOrientation();
      vertexBuffer.clear();
      originBuffer.clear();
      normalBuffer.clear();
      orientationBuffer.clear();

      IDLSequence.Integer regionIdBuffer = messageToPack.getRegionId();
      IDLSequence.Integer numberOfRegionsBuffer = messageToPack.getRegionsPerYaw();
      IDLSequence.Integer concaveHullSizeBuffer = messageToPack.getConcaveHullsSize();
      IDLSequence.Double footYawBuffer = messageToPack.getFootYaw();
      regionIdBuffer.clear();
      numberOfRegionsBuffer.clear();
      concaveHullSizeBuffer.clear();
      footYawBuffer.clear();

      IDLSequence.Object<HeightMapMessage> heightMapBuffer = messageToPack.getLocalHeightMap();
      heightMapBuffer.clear();

      for (int yawIndex = 0; yawIndex < steppableRegionListCollection.getDiscretizations(); yawIndex++)
      {
         SteppableRegionsList steppableRegionsList = steppableRegionListCollection.getSteppableRegions(yawIndex);
         double footYaw = steppableRegionsList.getFootYaw();

         numberOfRegionsBuffer.add(steppableRegionsList.getNumberOfSteppableRegions());
         footYawBuffer.add(footYaw);

         for (SteppableRegion steppableRegion : steppableRegionsList.getSteppableRegionsAsList())
         {
            regionIdBuffer.add(steppableRegion.getRegionId());
            originBuffer.add().set(steppableRegion.getRegionOrigin());
            normalBuffer.add().set(steppableRegion.getRegionNormal());
            orientationBuffer.add().set(steppableRegion.getRegionOrientation());

            concaveHullSizeBuffer.add(steppableRegion.getConcaveHullInRegionFrame().getNumberOfVertices());

            for (int vertexIndex = 0; vertexIndex < steppableRegion.getConcaveHullInRegionFrame().getNumberOfVertices(); vertexIndex++)
            {
               vertexBuffer.add().set(steppableRegion.getConcaveHullInRegionFrame().getVertex(vertexIndex), 0.0);
            }

            if (Double.isFinite(footYaw) && !MathTools.epsilonEquals(steppableRegion.getFootYaw(), footYaw, 1e-5))
               throw new RuntimeException("Yaws are not equal.");

            HeightMapMessage heightMapMessage = heightMapBuffer.add();
            if (steppableRegion.getLocalHeightMap() != null)
               HeightMapMessageTools.toMessage(steppableRegion.getLocalHeightMap(), heightMapMessage);
            else
               HeightMapMessageTools.clear(heightMapMessage);
         }
      }
   }

   public static SteppableRegionsListCollection convertToSteppableRegionsListCollection(SteppableRegionsListCollectionMessage message)
   {
      int yaws = message.getFootYaw().size();

      SteppableRegionsListCollection collection = new SteppableRegionsListCollection(yaws);

      IDLSequence.Object<Point3D> vertexBuffer = message.getVertexBuffer();
      IDLSequence.Object<Vector3D> normals = message.getRegionNormal();
      IDLSequence.Object<Point3D> origins = message.getRegionOrigin();

      int vertexIndex = 0;
      int vertexUpperBound = 0;

      int regionIndex = 0;
      int regionUpperBound = 0;

      for (int yawIndex = 0; yawIndex < yaws; yawIndex++)
      {
         regionUpperBound += message.getRegionsPerYaw().get(yawIndex);
         double footYaw = message.getFootYaw().get(yawIndex);

         List<SteppableRegion> steppableRegions = new ArrayList<>();

         for (; regionIndex < regionUpperBound; regionIndex++)
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

            vertexUpperBound += message.getConcaveHullsSize().get(regionIndex);
            List<Point2D> concaveHullVertices = new ArrayList<>();

            for (; vertexIndex < vertexUpperBound; vertexIndex++)
            {
               concaveHullVertices.add(new Point2D(vertexBuffer.get(vertexIndex)));
            }

            SteppableRegion steppableRegion = new SteppableRegion(transformToWorld,
                                                                  concaveHullVertices,
                                                                  footYaw);
            steppableRegion.setRegionId(message.getRegionId().get(regionIndex));
            steppableRegions.add(steppableRegion);

            if (message.getLocalHeightMap().get(regionIndex).getHeights().size() > 0)
            {
               HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(message.getLocalHeightMap().get(regionIndex));
               steppableRegion.setLocalHeightMap(heightMapData);
            }
         }

         SteppableRegionsList steppableRegionsList = new SteppableRegionsList(footYaw, steppableRegions);
         collection.setSteppableRegions(yawIndex, steppableRegionsList);
      }

      return collection;
   }
}
