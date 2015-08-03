package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicInteger;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.walking.ThighStatePacket;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class DesiredThighLoadBearingProvider implements PacketConsumer<ThighStatePacket>
{
   private final SideDependentList<AtomicInteger> loadBearingState = new SideDependentList<AtomicInteger>();
   
   public DesiredThighLoadBearingProvider()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         loadBearingState.put(robotSide, new AtomicInteger(-1));
      }
   }
   
   public boolean checkForNewLoadBearingState(RobotSide robotSide)
   {
      return loadBearingState.get(robotSide).get() != -1;
   }
   
   public boolean getDesiredThighLoadBearingState(RobotSide robotSide)
   {
      return loadBearingState.get(robotSide).getAndSet(-1) == 1;
   }

   public void receivedPacket(ThighStatePacket object)
   {
      RobotSide robotSide = object.getRobotSide();
      loadBearingState.get(robotSide).set(object.isLoadBearing() ? 1 : 0);
   }
}
