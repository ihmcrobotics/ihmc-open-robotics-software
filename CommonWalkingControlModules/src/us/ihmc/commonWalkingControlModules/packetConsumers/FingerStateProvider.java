package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.commonWalkingControlModules.packets.FingerStatePacket;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.net.ObjectConsumer;

/**
 * @author twan
 *         Date: 6/7/13
 */
public class FingerStateProvider implements ObjectConsumer<FingerStatePacket>
{
   private final ConcurrentLinkedQueue<FingerStatePacket> packetQueue = new ConcurrentLinkedQueue<FingerStatePacket>();

   public void consumeObject(FingerStatePacket packet)
   {
      packetQueue.add(packet);
   }

   public FingerStatePacket pullPacket()
   {
      return packetQueue.poll();
   }

   public boolean isNewFingerStateAvailable()
   {
      return !packetQueue.isEmpty();
   }
   
   public RobotSide getSide()
   {
      if(isNewFingerStateAvailable())
      {
         return packetQueue.peek().getRobotSide();
      }
      return null;
   }
}
