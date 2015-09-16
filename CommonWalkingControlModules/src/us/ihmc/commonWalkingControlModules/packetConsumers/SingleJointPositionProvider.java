package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.SingleJointAnglePacket;

public class SingleJointPositionProvider
{
   private static final double timeout = 0.5;
   private static final double resetTrajectoryTime = 0.25;
   private static final String defaultJointName = "l_leg_aky";
   
   private final PacketConsumer<SingleJointAnglePacket> packetConsumer;
   private final AtomicReference<SingleJointAnglePacket> lastPacket = new AtomicReference<SingleJointAnglePacket>();
   
   private final AtomicBoolean receivedData = new AtomicBoolean();

   private double lastReceivedTime = Double.MIN_VALUE;
   private double resetAngle = Double.NaN;
   
   public SingleJointPositionProvider()
   {
      packetConsumer = new PacketConsumer<SingleJointAnglePacket>()
      {
         @Override
         public void receivedPacket(SingleJointAnglePacket packet)
         {     
            if (packet != null)
            {
               lastPacket.set(packet);
            }
         }
      };
   }
   
   public void notifyWatchdogEventReceived()
   {
      receivedData.set(true);
   }
   
   public SingleJointAnglePacket getNewPacket(double time)
   {
      SingleJointAnglePacket packet = lastPacket.getAndSet(null);

      if(receivedData.getAndSet(false))
      {
         lastReceivedTime = time;         
      }
      
      if(packet != null)
      {
         resetAngle = packet.resetAngle;
         if(packet.jointName == null)
         {
            packet.jointName = defaultJointName;
         }
         
      }
      else if (!Double.isNaN(resetAngle))
      {
         if((time - lastReceivedTime) > timeout)
         {
//            PrintTools.debug(this, "JOINT ANGLE WATCHDOG TIMEOUT: LIFTING TOES");
            packet = new SingleJointAnglePacket(defaultJointName, resetAngle, resetTrajectoryTime, Double.NaN);
            resetAngle = Double.NaN;
         }
      }
      return packet;
   }
   
   public PacketConsumer<SingleJointAnglePacket> getPacketConsumer()
   {
      return packetConsumer;
   }

   public void receivedPacket(SingleJointAnglePacket singleJointAnglePacket)
   {
      packetConsumer.receivedPacket(singleJointAnglePacket);
   }
}
