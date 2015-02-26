package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.util.ArrayList;
import java.util.Arrays;

import javax.vecmath.Point3d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.sensing.SimulatedLidarScanPacket;
import us.ihmc.ihmcPerception.depthData.RobotBoundingBoxes;
import us.ihmc.utilities.lidar.polarLidar.LidarScan;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class SCSCheatingPointCloudLidarReceiver implements PacketConsumer<SimulatedLidarScanPacket>
{

   private final PointCloudDataReceiver pointCloudDataReceiver;
   public SCSCheatingPointCloudLidarReceiver(RobotBoundingBoxes robotBoundingBoxes, PacketCommunicator packetCommunicator, PointCloudDataReceiver pointCloudDataReceiver)
   {
      this.pointCloudDataReceiver = pointCloudDataReceiver;
      pointCloudDataReceiver.addPointFilter(robotBoundingBoxes);
      
      packetCommunicator.attachListener(SimulatedLidarScanPacket.class, this);
   }

   @Override
   public void receivedPacket(SimulatedLidarScanPacket packet)
   {
      LidarScan scan = packet.createFullLidarScan();
      ArrayList<Point3d> points = scan.getAllPoints();

      long[] timestamps = new long[points.size()];
      Arrays.fill(timestamps, packet.getScanStartTime());
      Point3d origin = new Point3d();
      scan.getAverageTransform().transform(origin);

      pointCloudDataReceiver.receivedPointCloudData(ReferenceFrame.getWorldFrame(), origin, timestamps, points);
   }

   public void connect()
   {

   }

}
