package us.ihmc.ihmcPerception.steppableRegions;

import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.SteppableRegionMessage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
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

      SteppableRegion steppableRegion = new SteppableRegion(transformToWorld.getTranslation(), transformToWorld.getRotation(), concaveHullVertices);
      steppableRegion.setRegionId(message.getRegionId());

      if (message.getLocalHeightMap().getHeights().size() > 0)
      {
         HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(message.getLocalHeightMap());
         steppableRegion.setLocalHeightMap(heightMapData);
      }

      return steppableRegion;
   }
}
