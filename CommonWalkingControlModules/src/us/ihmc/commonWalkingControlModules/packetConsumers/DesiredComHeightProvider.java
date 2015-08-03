package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.math.trajectories.WaypointPositionTrajectoryData;

import com.google.common.util.concurrent.AtomicDouble;

public class DesiredComHeightProvider
{
   private final PacketConsumer<ComHeightPacket>           comHeightPacketConsumer;
   private final PacketConsumer<WholeBodyTrajectoryPacket> wholeBodyPacketConsumer;

   // Do not do it like this, preferably use one atomic
   private final AtomicBoolean newDataAvailable = new AtomicBoolean(false);
   private final AtomicDouble comHeightOffset = new AtomicDouble(0.0);
   private final AtomicDouble trajectoryTime = new AtomicDouble(0.0);
   private final AtomicReference<WaypointPositionTrajectoryData> multipointTrajectory = new  AtomicReference<WaypointPositionTrajectoryData>(null);

   private final double defaultTrajectoryTime = 0.5; //Hackish default time for height trajectory. We need to just ensure that this is always set in the packet instead and then get rid of this.
   private final GlobalDataProducer globalDataProducer;
   
   public DesiredComHeightProvider(GlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;
      
      comHeightPacketConsumer = new PacketConsumer<ComHeightPacket>()
      {
         @Override public void receivedPacket(ComHeightPacket packet) {  receivedPacketImplementation(packet);  }
      };
      
      wholeBodyPacketConsumer = new PacketConsumer<WholeBodyTrajectoryPacket>()
      {
         @Override public void receivedPacket(WholeBodyTrajectoryPacket packet) {  receivedPacketImplementation(packet);  }
      };
   }

   public PacketConsumer<ComHeightPacket> getComHeightPacketConsumer()
   {
      return comHeightPacketConsumer;
   }
   
   public PacketConsumer<WholeBodyTrajectoryPacket> getWholeBodyPacketConsumer()
   {
      return wholeBodyPacketConsumer;
   }

   public void receivedPacketImplementation(ComHeightPacket packet)
   {
      if (globalDataProducer != null)
      {
         String errorMessage = PacketValidityChecker.validateCoMHeightPacket(packet);
         if (errorMessage != null)
         {
            globalDataProducer.notifyInvalidPacketReceived(ComHeightPacket.class, errorMessage);
            return;
         }
      }
      
      newDataAvailable.set(true);
      
      double heightOffset = packet.getHeightOffset();
      heightOffset = MathTools.clipToMinMax(heightOffset, ComHeightPacket.MIN_COM_HEIGHT, ComHeightPacket.MAX_COM_HEIGHT);
      comHeightOffset.set(heightOffset);
      double packetTime = packet.getTrajectoryTime();
      
      if ((packetTime < 1e-7) || (Double.isNaN(packetTime) || packetTime > 1000))
      {
         packetTime = defaultTrajectoryTime;
      }
      trajectoryTime.set(packetTime);
   }
   
   public void receivedPacketImplementation(WholeBodyTrajectoryPacket packet)
   {
      if (packet != null && packet.pelvisWorldPosition != null )
      {
         double[] timeAtWaypoints = packet.timeAtWaypoint;
         Point3d[] positions = packet.pelvisWorldPosition;
         Vector3d[] velocities = packet.pelvisLinearVelocity;
         WaypointPositionTrajectoryData positionTrajectoryData = new WaypointPositionTrajectoryData(ReferenceFrame.getWorldFrame(), timeAtWaypoints, positions, velocities);
         multipointTrajectory.set(positionTrajectoryData);
        // System.out.println("DesiredComHeightProvider: PACKET received\n" +  packet);
      }
   }


   public boolean isNewComHeightInformationAvailable()
   {
      return newDataAvailable.getAndSet(false);
   }

   public boolean isNewComHeightMultipointAvailable()
   {
      return multipointTrajectory.get() != null ;
   }

   public double getComHeightTrajectoryTime()
   {
      return trajectoryTime.get();
   }

   public double getComHeightOffset()
   {
      return comHeightOffset.get();
   }
   
   public WaypointPositionTrajectoryData getComHeightMultipointWorldPosition()
   {
      return  multipointTrajectory.getAndSet(null);
   }

}
