package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.FootStatePacket;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredFootStateProvider implements ObjectConsumer<FootStatePacket>
{
   private SideDependentList<Boolean> hasLoadBearingBeenRequested = new SideDependentList<Boolean>();
   
   public DesiredFootStateProvider()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         hasLoadBearingBeenRequested.put(robotSide, false);
      }
   }
   
   public synchronized boolean checkForNewLoadBearingRequest(RobotSide robotSide)
   {
      boolean ret = hasLoadBearingBeenRequested.get(robotSide);
      hasLoadBearingBeenRequested.put(robotSide, false);
      
      return ret;
   }

   public void consumeObject(FootStatePacket object)
   {
      RobotSide robotSide = object.getRobotSide();
      hasLoadBearingBeenRequested.put(robotSide, true);
   }
}
