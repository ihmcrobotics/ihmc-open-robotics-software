package us.ihmc.ihmcPerception.steppableRegions;

import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.SteppableRegionMessage;
import perception_msgs.msg.dds.SteppableRegionsListMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
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
      message.setFootLength(steppableRegion.getFootLength());
      message.setFootWidth(steppableRegion.getFootWidth());
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

      HeightMapMessage heightMapMessage = HeightMapMessageTools.toMessage(steppableRegion.getLocalHeightMap());
      message.getLocalHeightMap().set(heightMapMessage);

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
      double footLength = message.getFootLength();
      double footWidth = message.getFootWidth();
      SteppableRegion steppableRegion = new SteppableRegion(transformToWorld.getTranslation(),
                                                            transformToWorld.getRotation(),
                                                            concaveHullVertices,
                                                            footYaw,
                                                            footLength,
                                                            footWidth);
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
      vertexBuffer.clear();

      double footYaw = steppableRegionsList.getSteppableRegionsAsList().get(0).getFootYaw();
      double footLength = steppableRegionsList.getSteppableRegionsAsList().get(0).getFootLength();
      double footWidth = steppableRegionsList.getSteppableRegionsAsList().get(0).getFootWidth();

      for (SteppableRegion steppableRegion : steppableRegionsList.getSteppableRegionsAsList())
      {
         message.getRegionId().add(steppableRegion.getRegionId());
         message.getRegionOrigin().add().set(steppableRegion.getRegionOrigin());
         message.getRegionNormal().add().set(steppableRegion.getRegionNormal());
         message.getRegionOrientation().add().set(steppableRegion.getRegionOrientation());

         message.getConcaveHullsSize().add(steppableRegion.getConcaveHullInRegionFrame().getNumberOfVertices());

         for (int vertexIndex = 0; vertexIndex < steppableRegion.getConcaveHullInRegionFrame().getNumberOfVertices(); vertexIndex++)
         {
            vertexBuffer.add().set(steppableRegion.getConcaveHullInRegionFrame().getVertex(vertexIndex), 0.0);
         }

         if (MathTools.epsilonEquals(steppableRegion.getFootYaw(), footYaw, 1e-5))
            throw new RuntimeException("Yaws are not equal.");
         if (MathTools.epsilonEquals(steppableRegion.getFootLength(), footLength, 1e-5))
            throw new RuntimeException("Lengths are not equal.");
         if (MathTools.epsilonEquals(steppableRegion.getFootWidth(), footWidth, 1e-5))
            throw new RuntimeException("Widths are not equal.");

         HeightMapMessage heightMapMessage = HeightMapMessageTools.toMessage(steppableRegion.getLocalHeightMap());
         message.getLocalHeightMap().add().set(heightMapMessage);
      }
      message.setFootLength(footLength);
      message.setFootLength(footWidth);
      message.setFootYaw(footYaw);

      return message;
   }

   public static SteppableRegionsList convertToPlanarRegionsList(SteppableRegionsListMessage message)
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
      double footLength = message.getFootLength();
      double footWidth = message.getFootWidth();

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

         SteppableRegion steppableRegion = new SteppableRegion(transformToWorld.getTranslation(),
                                                                    transformToWorld.getRotation(),
                                                                    concaveHullVertices,
                                                                    footYaw,
                                                                    footLength,
                                                                    footWidth);
         steppableRegion.setRegionId(message.getRegionId().get(regionIndex));
         steppableRegions.add(steppableRegion);
      }

      return new SteppableRegionsList(footYaw, footLength, footWidth, steppableRegions);
   }
}
