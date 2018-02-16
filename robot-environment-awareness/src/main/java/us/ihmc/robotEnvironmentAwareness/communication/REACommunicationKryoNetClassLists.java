package us.ihmc.robotEnvironmentAwareness.communication;

import java.util.ArrayList;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionMessage;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestLidarScanMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionsRequestType;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.APIElementId;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoxMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.LineSegment3DMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeNodeMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OcTreeKeyMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;

/**
 * Created by adrien on 11/18/16.
 */
public class REACommunicationKryoNetClassLists
{
   private static final NetClassList privateNetClassList = new NetClassList();
   static
   {
      privateNetClassList.registerPacketClass(Packet.class);
      privateNetClassList.registerPacketClass(REAMessage.class);
      privateNetClassList.registerPacketField(PacketDestination.class);
      privateNetClassList.registerPacketField(Boolean.class);
      privateNetClassList.registerPacketField(Double.class);
      privateNetClassList.registerPacketField(Integer.class);
      privateNetClassList.registerPacketField(float[].class);
      privateNetClassList.registerPacketField(int[].class);
      privateNetClassList.registerPacketField(ArrayList.class);
      privateNetClassList.registerPacketField(Point3D.class);
      privateNetClassList.registerPacketField(Point3D32.class);
      privateNetClassList.registerPacketField(Point2D32.class);
      privateNetClassList.registerPacketField(Vector3D32.class);
      privateNetClassList.registerPacketField(Quaternion.class);
      privateNetClassList.registerPacketField(Quaternion32.class);
      privateNetClassList.registerPacketField(Point3D32[].class);
      privateNetClassList.registerPacketField(Point2D32[].class);
      privateNetClassList.registerPacketField(LineSegment3DMessage.class);
      privateNetClassList.registerPacketField(LineSegment3DMessage[].class);
      privateNetClassList.registerPacketField(APIElementId.class);
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
      privateNetClassList.registerPacketField(PlanarRegionMessage.class);
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
      publicNetClassList.registerPacketField(Point3D32.class);
      publicNetClassList.registerPacketField(Point2D32.class);
      publicNetClassList.registerPacketField(Vector3D32.class);
      publicNetClassList.registerPacketField(Quaternion32.class);
      publicNetClassList.registerPacketField(Point2D32[].class);
      publicNetClassList.registerPacketField(PlanarRegionsRequestType.class);

      publicNetClassList.registerPacketField(PlanarRegionMessage.class);
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
