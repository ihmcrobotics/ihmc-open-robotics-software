package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.lang.mutable.MutableDouble;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.ChestOrientationPacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryDevelopmentPacket;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameOrientationWaypoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class DesiredChestOrientationProvider extends ChestOrientationProvider
{
   double defaultTrajectoryTime;
   private AtomicReference<FrameOrientationWaypoint[]> chestOrientationWaypoints = new AtomicReference<FrameOrientationWaypoint[]>(null);
   
   private final AtomicBoolean goToHomeOrientation = new AtomicBoolean(false);
   private final ReferenceFrame chestOrientationFrame;
   private final PacketConsumer<ChestOrientationPacket> packetConsumer;
   private final PacketConsumer<WholeBodyTrajectoryDevelopmentPacket> wholeBodyTrajectoryPacketConsumer;
   private double homeTrajectoryTime = 0;
   
   public DesiredChestOrientationProvider(ReferenceFrame chestOrientationFrame, double defaultTrajectoryTime)
   {
      this.chestOrientationFrame = chestOrientationFrame;
      this.defaultTrajectoryTime = defaultTrajectoryTime;
      //trajectoryTime.set(defaultTrajectoryTime);

      packetConsumer = new PacketConsumer<ChestOrientationPacket>()
            {
         @Override public void receivedPacket(ChestOrientationPacket packet)
         {
            if (packet != null){ receivedPacketImplementation(packet);  }
         }
            };  
            
      wholeBodyTrajectoryPacketConsumer = new PacketConsumer<WholeBodyTrajectoryDevelopmentPacket>()
            {
         @Override public void receivedPacket(WholeBodyTrajectoryDevelopmentPacket packet)
         {
            if (packet != null){ receivedPacketImplementation(packet);  }
         }
            };  
   }

   @Override
   public boolean checkForNewChestOrientation()
   {
      return chestOrientationWaypoints.get() != null;
   }

   @Override
   public boolean checkForHomeOrientation()
   {
      return goToHomeOrientation.getAndSet(false);
   }

   @Override
   public FrameOrientationWaypoint[] getDesiredChestOrientations()
   {
      FrameOrientationWaypoint[] waypoints = chestOrientationWaypoints.getAndSet(null);
      if( waypoints == null )
      {
         return null;
      }
      
      for (int i=0; i<waypoints.length; i++ ) {
         waypoints[i].orientation.changeFrame(chestOrientationFrame);
      }

      return waypoints;
   }
   
   public void receivedPacketImplementation(WholeBodyTrajectoryDevelopmentPacket packet)
   {
      if (packet == null) return;
      int N = packet.getNumberOfWaypoints();
      
      FrameOrientationWaypoint[]  chestWaypoints = new FrameOrientationWaypoint[N]; 
            
      for (int i=0; i<N; i++)
      {
         chestWaypoints[i] = new FrameOrientationWaypoint( 
               packet.timeSincePrevious[i], 
               new FrameOrientation( ReferenceFrame.getWorldFrame(), packet.chestOrientation[i] ) );
      }
      chestOrientationWaypoints.set( chestWaypoints );
   }
   
   public double getTrajectoryTimeForHomeOrientation()
   {
      return homeTrajectoryTime;
   }
   
   public void receivedPacketImplementation(ChestOrientationPacket packet)
   {
      if (packet == null) return;
  
      // If go to home orientation requested, ignore the other commands.
      if (packet.isToHomeOrientation())
      {
         goToHomeOrientation.set(true);
         homeTrajectoryTime = packet.getTrajectoryTime();
         return;
      }

      FrameOrientationWaypoint singleWaypoint = new FrameOrientationWaypoint(
            packet.getTrajectoryTime(),  
            new FrameOrientation(ReferenceFrame.getWorldFrame(), packet.getOrientation() ) );
      
      chestOrientationWaypoints.set( new FrameOrientationWaypoint[] { singleWaypoint } );
   }

   @Override
   public ReferenceFrame getChestOrientationExpressedInFrame()
   {
      return chestOrientationFrame;
   }

   public PacketConsumer<ChestOrientationPacket> getConsumer()
   {
      return packetConsumer;
   }
   
   public PacketConsumer<WholeBodyTrajectoryDevelopmentPacket> getTrajectoryConsumer()
   {
      return wholeBodyTrajectoryPacketConsumer;
   }
}
