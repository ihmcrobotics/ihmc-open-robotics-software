package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicInteger;

import us.ihmc.commonWalkingControlModules.packets.HandLoadBearingPacket;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredHandLoadBearingProvider implements ObjectConsumer<HandLoadBearingPacket>
{
   private SideDependentList<AtomicInteger> hasLoadBearingBeenRequested = new SideDependentList<AtomicInteger>();

   public DesiredHandLoadBearingProvider()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         hasLoadBearingBeenRequested.put(robotSide, new AtomicInteger(-1));
      }
   }

   public boolean checkForNewInformation(RobotSide robotSide)
   {
      return hasLoadBearingBeenRequested.get(robotSide).get() != -1;
   }

   public boolean hasLoadBearingBeenRequested(RobotSide robotSide)
   {
      return hasLoadBearingBeenRequested.get(robotSide).getAndSet(-1) == 1;
   }

   public void consumeObject(HandLoadBearingPacket object)
   {
      RobotSide robotSide = object.getRobotSide();
      hasLoadBearingBeenRequested.get(robotSide).set(object.isLoadBearing() ? 1 : 0);
   }
}
