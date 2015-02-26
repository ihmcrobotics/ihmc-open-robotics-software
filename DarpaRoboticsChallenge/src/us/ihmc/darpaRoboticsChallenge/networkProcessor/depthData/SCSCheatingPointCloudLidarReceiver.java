package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.util.ArrayList;
import java.util.Arrays;

import javax.vecmath.Point3d;

import us.ihmc.communication.blackoutGenerators.CommunicationBlackoutSimulator;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.BlackoutPacketConsumer;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.sensing.SimulatedLidarScanPacket;
import us.ihmc.utilities.lidar.polarLidar.LidarScan;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class SCSCheatingPointCloudLidarReceiver implements PacketConsumer<SimulatedLidarScanPacket>
{

   private final PointCloudDataReceiver pointCloudDataReceiver;
   private final ReferenceFrame lidarFrame;
   
   public SCSCheatingPointCloudLidarReceiver(String lidarName, PacketCommunicator packetCommunicator, PointCloudDataReceiver pointCloudDataReceiver)
   {
      this.pointCloudDataReceiver = pointCloudDataReceiver;
      this.lidarFrame = pointCloudDataReceiver.getLidarFrame(lidarName);
      packetCommunicator.attachListener(SimulatedLidarScanPacket.class, this);
   }

   @Override
   public void receivedPacket(SimulatedLidarScanPacket packet)
   {
      LidarScan scan = packet.createFullLidarScan();
      ArrayList<Point3d> points = scan.getAllPoints();

      long[] timestamps = new long[points.size()];
      Arrays.fill(timestamps, packet.getScanStartTime());

      pointCloudDataReceiver.receivedPointCloudData(ReferenceFrame.getWorldFrame(), lidarFrame, timestamps, points);
   }

   public void connect()
   {

   }

}
