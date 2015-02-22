package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javax.vecmath.Point3d;

import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.userInterface.util.TimestampedPoint;

public class PointCloudWorldPacketGenerator implements Runnable
{
   DepthDataFilter depthDataFilter;
   PacketCommunicator packetCommunicator;
   public PointCloudWorldPacketGenerator(DepthDataFilter depthDataFilter)
   {
      this(depthDataFilter, null);
   }

   public PointCloudWorldPacketGenerator(DepthDataFilter depthDataFilter, PacketCommunicator packetCommunicator)
   {
      this.depthDataFilter = depthDataFilter;
      this.packetCommunicator= packetCommunicator;
      ScheduledExecutorService service = Executors.newSingleThreadScheduledExecutor();
      int publishRateHz = 1;
      service.scheduleAtFixedRate(this, 0, 1000/publishRateHz, TimeUnit.MILLISECONDS);
   }
   
   public PointCloudWorldPacket getPointCloudWorldPacket()
   {
      PointCloudWorldPacket packet = new PointCloudWorldPacket();
      ArrayList<Point3d> groundPoints = new ArrayList<>();
      depthDataFilter.getQuadTree().getStoredPoints(groundPoints);
      packet.setGroundQuadTreeSupport(groundPoints.toArray(new Point3d[groundPoints.size()]));
      
      ArrayList<TimestampedPoint> nearScanTimestampedPoints = depthDataFilter.getNearScan().getPointsCopy();
      ArrayList<Point3d> nearScanPoints = new ArrayList<>();
      for(TimestampedPoint point:nearScanTimestampedPoints)
      {
         nearScanPoints.add(new Point3d(point.x, point.y,point.z));
      }
      packet.setDecayingWorldScan(nearScanPoints.toArray(new Point3d[nearScanPoints.size()]));
      packet.timestamp = System.nanoTime();
      return packet;
   }

   @Override
   public void run()
   {
      if(packetCommunicator!=null)
         packetCommunicator.send(getPointCloudWorldPacket());
   }
}

