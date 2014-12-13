package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.walking.FootStatePacket;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class DesiredFootStateProvider implements ObjectConsumer<FootStatePacket>
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

   public void consumeObject(FootStatePacket object)
   {
      hasLoadBearingBeenRequested.get(object.getRobotSide()).set(true);;
   }
}
