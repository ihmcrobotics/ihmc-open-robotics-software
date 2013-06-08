package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.HandLoadBearingPacket;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredHandLoadBearingProvider implements ObjectConsumer<HandLoadBearingPacket>
{
   private SideDependentList<Boolean> hasLoadBearingBeenRequested = new SideDependentList<Boolean>();
   private SideDependentList<Boolean> hasNewInformation = new SideDependentList<Boolean>();


   public DesiredHandLoadBearingProvider()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         hasLoadBearingBeenRequested.put(robotSide, false);
         hasNewInformation.put(robotSide, false);
      }
   }

   public synchronized boolean checkForNewInformation(RobotSide robotSide)
   {
      return hasNewInformation.get(robotSide);
   }

   public synchronized boolean hasLoadBearingBeenRequested(RobotSide robotSide)
   {
      hasNewInformation.put(robotSide, false);

      return hasLoadBearingBeenRequested.get(robotSide);
   }

   public synchronized void consumeObject(HandLoadBearingPacket object)
   {
      RobotSide robotSide = object.getRobotSide();
      hasLoadBearingBeenRequested.put(robotSide, object.isLoadBearing());
      hasNewInformation.put(robotSide, true);
   }
}
