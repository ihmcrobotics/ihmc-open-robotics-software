package us.ihmc.atlas.multisenseBlobExperiments;

import controller_msgs.msg.dds.PointCloudWorldPacket;
import sensor_msgs.PointCloud2;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.utilities.ros.subscriber.RosPointCloudSubscriber;

public class MultisenseBlobDetectionPointCloudReceiver extends RosPointCloudSubscriber
{
   private static final boolean DEBUG = false;

   private final PacketCommunicator packetCommunicator;

   public MultisenseBlobDetectionPointCloudReceiver(PacketCommunicator packetCommunicator)
   {
      this.packetCommunicator = packetCommunicator;
   }

   /**
    * received a point cloud from the multisense
    * transform the points using the mocap data
    */
   @Override
   public void onNewMessage(PointCloud2 pointCloud)
   {
      UnpackedPointCloud pointCloudData = unpackPointsAndIntensities(pointCloud);
      Point3D[] points = pointCloudData.getPoints();

      PointCloudWorldPacket pointCloudWorldPacket = new PointCloudWorldPacket();
      HumanoidMessageTools.setDecayingWorldScan(points, pointCloudWorldPacket);
      pointCloudWorldPacket.setDestination(PacketDestination.BROADCAST.ordinal());

      if(DEBUG)
         System.out.println("received point cloud. cloud size = " + points.length);

      packetCommunicator.send(pointCloudWorldPacket);
   }
}
