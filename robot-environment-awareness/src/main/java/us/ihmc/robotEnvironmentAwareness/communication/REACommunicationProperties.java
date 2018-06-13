package us.ihmc.robotEnvironmentAwareness.communication;

import java.util.ArrayList;

import controller_msgs.msg.dds.BoundingBox3DMessage;
import controller_msgs.msg.dds.BoundingBox3DMessagePubSubType;
import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.LidarScanMessagePubSubType;
import controller_msgs.msg.dds.PlanarRegionMessage;
import controller_msgs.msg.dds.PlanarRegionMessagePubSubType;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessagePubSubType;
import controller_msgs.msg.dds.Polygon2DMessage;
import controller_msgs.msg.dds.Polygon2DMessagePubSubType;
import controller_msgs.msg.dds.RequestLidarScanMessage;
import controller_msgs.msg.dds.RequestLidarScanMessagePubSubType;
import controller_msgs.msg.dds.RequestPlanarRegionsListMessage;
import controller_msgs.msg.dds.RequestPlanarRegionsListMessagePubSubType;
import geometry_msgs.msg.dds.PointPubSubType;
import geometry_msgs.msg.dds.QuaternionPubSubType;
import geometry_msgs.msg.dds.Vector3PubSubType;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionsRequestType;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.idl.IDLSequence;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.javaFXToolkit.messager.Message;
import us.ihmc.javaFXToolkit.messager.MessagerAPIFactory.TopicID;
import us.ihmc.pubsub.TopicDataType;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoxMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.LineSegment3DMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeNodeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OcTreeKeyMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;

/**
 * Created by adrien on 11/18/16.
 */
public class REACommunicationProperties
{
   public static final MessageTopicNameGenerator publisherTopicNameGenerator = ROS2Tools.getTopicNameGenerator(null, ROS2Tools.REA_MODULE, ROS2TopicQualifier.OUTPUT);
   public static final MessageTopicNameGenerator subscriberTopicNameGenerator = ROS2Tools.getTopicNameGenerator(null, ROS2Tools.REA_MODULE, ROS2TopicQualifier.INPUT);

   private static final NetClassList privateNetClassList = new NetClassList();
   static
   {
      privateNetClassList.registerPacketClass(Message.class);
      privateNetClassList.registerPacketField(PacketDestination.class);
      privateNetClassList.registerPacketField(Boolean.class);
      privateNetClassList.registerPacketField(Double.class);
      privateNetClassList.registerPacketField(Integer.class);
      privateNetClassList.registerPacketField(float[].class);
      privateNetClassList.registerPacketField(int[].class);
      privateNetClassList.registerPacketField(ArrayList.class);
      privateNetClassList.registerPacketField(Point3D.class);
      privateNetClassList.registerPacketField(Point3D32.class);
      privateNetClassList.registerPacketField(Point2D.class);
      privateNetClassList.registerPacketField(Vector3D.class);
      privateNetClassList.registerPacketField(Vector3D32.class);
      privateNetClassList.registerPacketField(Quaternion.class);
      privateNetClassList.registerPacketField(Point3D[].class);
      privateNetClassList.registerPacketField(Point3D32[].class);
      privateNetClassList.registerPacketField(Point2D[].class);
      privateNetClassList.registerPacketField(LineSegment3DMessage.class);
      privateNetClassList.registerPacketField(LineSegment3DMessage[].class);
      privateNetClassList.registerPacketField(TopicID.class);
      privateNetClassList.registerPacketField(LidarScanMessage.class);
      privateNetClassList.registerPacketField(BoxMessage.class);
      privateNetClassList.registerPacketField(BoundingBoxParametersMessage.class);
      privateNetClassList.registerPacketField(NormalEstimationParameters.class);
      privateNetClassList.registerPacketField(PlanarRegionSegmentationParameters.class);
      privateNetClassList.registerPacketField(IntersectionEstimationParameters.class);
      privateNetClassList.registerPacketField(PolygonizerParameters.class);
      privateNetClassList.registerPacketField(NormalOcTreeMessage.class);
      privateNetClassList.registerPacketField(NormalOcTreeNodeMessage.class);
      privateNetClassList.registerPacketField(NormalOcTreeNodeMessage[].class);
      privateNetClassList.registerPacketField(OcTreeKeyMessage.class);
      privateNetClassList.registerPacketField(OcTreeKeyMessage[].class);
      privateNetClassList.registerPacketField(PlanarRegionSegmentationMessage.class);
      privateNetClassList.registerPacketField(PlanarRegionSegmentationMessage[].class);
      privateNetClassList.registerPacketField(PlanarRegionsListMessage.class);
      privateNetClassList.registerPacketField(Polygon2DMessage.class);
      privateNetClassList.registerPacketField(PlanarRegionMessage.class);
      privateNetClassList.registerPacketField(ConcaveHullFactoryParameters.class);

      privateNetClassList.registerPacketField(Vector3PubSubType.class);
      privateNetClassList.registerPacketField(PointPubSubType.class);
      privateNetClassList.registerPacketField(QuaternionPubSubType.class);
      privateNetClassList.registerPacketField(Polygon2DMessagePubSubType.class);
      privateNetClassList.registerPacketField(BoundingBox3DMessagePubSubType.class);
      privateNetClassList.registerPacketField(RequestPlanarRegionsListMessagePubSubType.class);
      privateNetClassList.registerPacketField(RequestLidarScanMessagePubSubType.class);
      privateNetClassList.registerPacketField(PlanarRegionsListMessagePubSubType.class);
      privateNetClassList.registerPacketField(LidarScanMessagePubSubType.class);
      privateNetClassList.registerPacketField(PlanarRegionMessagePubSubType.class);

      privateNetClassList.registerPacketField(IDLSequence.Object.class);
      privateNetClassList.registerPacketField(IDLSequence.Float.class);
      privateNetClassList.registerPacketField(IDLSequence.Boolean.class);
      privateNetClassList.registerPacketField(IDLSequence.Double.class);
      privateNetClassList.registerPacketField(IDLSequence.Integer.class);
      privateNetClassList.registerPacketField(IDLSequence.Byte.class);
      privateNetClassList.registerPacketField(IDLSequence.Long.class);
      privateNetClassList.registerPacketField(IDLSequence.StringBuilderHolder.class);
      privateNetClassList.registerPacketField(TopicDataType.class);
      privateNetClassList.registerPacketField(RecyclingArrayList.class);
      privateNetClassList.registerPacketField(us.ihmc.idl.CDR.class);
   }

   private static final NetClassList publicNetClassList = new NetClassList();

   static
   {
      publicNetClassList.registerPacketClass(Packet.class);
      publicNetClassList.registerPacketClass(LidarScanMessage.class);
      publicNetClassList.registerPacketClass(PlanarRegionsListMessage.class);
      publicNetClassList.registerPacketClass(RequestPlanarRegionsListMessage.class);
      publicNetClassList.registerPacketClass(RequestLidarScanMessage.class);

      publicNetClassList.registerPacketField(PacketDestination.class);

      publicNetClassList.registerPacketField(float[].class);
      publicNetClassList.registerPacketField(ArrayList.class);
      publicNetClassList.registerPacketField(Vector3D.class);
      publicNetClassList.registerPacketField(Point3D.class);
      publicNetClassList.registerPacketField(Point3D32.class);
      publicNetClassList.registerPacketField(Point2D32.class);
      publicNetClassList.registerPacketField(Vector3D32.class);
      publicNetClassList.registerPacketField(Quaternion32.class);
      publicNetClassList.registerPacketField(Quaternion.class);
      publicNetClassList.registerPacketField(Point2D32[].class);
      publicNetClassList.registerPacketField(PlanarRegionsRequestType.class);

      publicNetClassList.registerPacketField(Polygon2DMessage.class);
      publicNetClassList.registerPacketField(PlanarRegionMessage.class);
      publicNetClassList.registerPacketField(BoundingBox3DMessage.class);

      publicNetClassList.registerPacketField(Vector3PubSubType.class);
      publicNetClassList.registerPacketField(PointPubSubType.class);
      publicNetClassList.registerPacketField(QuaternionPubSubType.class);
      publicNetClassList.registerPacketField(Polygon2DMessagePubSubType.class);
      publicNetClassList.registerPacketField(BoundingBox3DMessagePubSubType.class);
      publicNetClassList.registerPacketField(RequestPlanarRegionsListMessagePubSubType.class);
      publicNetClassList.registerPacketField(RequestLidarScanMessagePubSubType.class);
      publicNetClassList.registerPacketField(PlanarRegionsListMessagePubSubType.class);
      publicNetClassList.registerPacketField(LidarScanMessagePubSubType.class);
      publicNetClassList.registerPacketField(PlanarRegionMessagePubSubType.class);

      publicNetClassList.registerPacketField(IDLSequence.Object.class);
      publicNetClassList.registerPacketField(IDLSequence.Float.class);
      publicNetClassList.registerPacketField(IDLSequence.Boolean.class);
      publicNetClassList.registerPacketField(IDLSequence.Double.class);
      publicNetClassList.registerPacketField(IDLSequence.Integer.class);
      publicNetClassList.registerPacketField(IDLSequence.Byte.class);
      publicNetClassList.registerPacketField(IDLSequence.Long.class);
      publicNetClassList.registerPacketField(IDLSequence.StringBuilderHolder.class);
      publicNetClassList.registerPacketField(TopicDataType.class);
      publicNetClassList.registerPacketField(RecyclingArrayList.class);
      publicNetClassList.registerPacketField(us.ihmc.idl.CDR.class);
   }

   public static NetClassList getPublicNetClassList()
   {
      return publicNetClassList;
   }
   
   public static NetClassList getPrivateNetClassList()
   {
      return privateNetClassList;
   }
}
