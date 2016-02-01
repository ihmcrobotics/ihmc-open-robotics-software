package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicInteger;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandLoadBearingPacket;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class DesiredHandLoadBearingProvider implements PacketConsumer<HandLoadBearingPacket>, HandLoadBearingProvider
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

   public void receivedPacket(HandLoadBearingPacket object)
   {
      RobotSide robotSide = object.getRobotSide();
      hasLoadBearingBeenRequested.get(robotSide).set(object.isLoadBearing() ? 1 : 0);
   }
}
