package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.FootStatePacket;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class DesiredFootStateProvider implements PacketConsumer<FootStatePacket>
{
   private SideDependentList<AtomicBoolean> hasLoadBearingBeenRequested = new SideDependentList<AtomicBoolean>();
   
   public DesiredFootStateProvider()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         hasLoadBearingBeenRequested.put(robotSide, new AtomicBoolean(false));
      }
   }
   
   public boolean checkForNewLoadBearingRequest(RobotSide robotSide)
   {
      return hasLoadBearingBeenRequested.get(robotSide).getAndSet(false);
   }

   public void receivedPacket(FootStatePacket object)
   {
      hasLoadBearingBeenRequested.get(object.getRobotSide()).set(true);;
   }
}
