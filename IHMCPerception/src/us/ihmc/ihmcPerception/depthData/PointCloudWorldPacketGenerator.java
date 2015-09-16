package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock.ReadLock;

import javax.vecmath.Point3d;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.userInterface.util.TimestampedPoint;

public class PointCloudWorldPacketGenerator implements Runnable
{
   private final ScheduledExecutorService executorService = Executors.newSingleThreadScheduledExecutor(ThreadTools
         .getNamedThreadFactory("PointCloudWorldPacketGenerator"));

   private final ReadLock readLock;
   private final DepthDataStore depthDataFilter;
   private final PacketCommunicator packetCommunicator;
   private ScheduledFuture<?> scheduled = null;

   public PointCloudWorldPacketGenerator(PacketCommunicator sensorSuitePacketCommunicator, ReadLock readLock, DepthDataStore depthDataFilter)
   {
      this.packetCommunicator = sensorSuitePacketCommunicator;
      this.readLock = readLock;
      this.depthDataFilter = depthDataFilter;
   }

   PointCloudWorldPacketGenerator(DepthDataStore depthDataFilter)
   {
      readLock = new ReentrantReadWriteLock().readLock();
      this.depthDataFilter = depthDataFilter;
      this.packetCommunicator = null;
   }

   public PointCloudWorldPacket getPointCloudWorldPacket()
   {
      PointCloudWorldPacket packet = new PointCloudWorldPacket();

      readLock.lock();
      ArrayList<Point3d> groundPoints = new ArrayList<>();
      ArrayList<TimestampedPoint> nearScanTimestampedPoints = depthDataFilter.getNearScan().getPointsCopy();
      packet.defaultGroundHeight = (float) depthDataFilter.getQuadTree().getDefaultHeightWhenNoPoints();
      depthDataFilter.getQuadTree().getCellAverageStoredPoints(groundPoints);
      readLock.unlock();

      packet.setGroundQuadTreeSupport(groundPoints.toArray(new Point3d[groundPoints.size()]));

      ArrayList<Point3d> nearScanPoints = new ArrayList<>();
      for (TimestampedPoint point : nearScanTimestampedPoints)
      {
         nearScanPoints.add(new Point3d(point.x, point.y, point.z));
      }
      packet.setDecayingWorldScan(nearScanPoints.toArray(new Point3d[nearScanPoints.size()]));
      packet.timestamp = System.nanoTime();
      return packet;
   }

   public void start()
   {
      if (scheduled == null)
      {
         scheduled = executorService.scheduleAtFixedRate(this, 0, 1, TimeUnit.SECONDS);
      }

   }

   public void stop()
   {
      scheduled.cancel(true);
      executorService.shutdown();
   }

   @Override
   public void run()
   {
      try
      {
         PointCloudWorldPacket pointCloudWorldPacket = getPointCloudWorldPacket();
         packetCommunicator.send(pointCloudWorldPacket);
      }
      catch(Exception e)
      {
         e.printStackTrace();
         throw new RuntimeException(e);
      }
   }
}
