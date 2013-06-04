package us.ihmc.commonWalkingControlModules.packetConsumers;

import us.ihmc.commonWalkingControlModules.packets.ThighStatePacket;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredThighLoadBearingProvider implements ObjectConsumer<ThighStatePacket>
{
   private SideDependentList<Boolean> hasNewLoadBearingState = new SideDependentList<Boolean>();
   private SideDependentList<Boolean> loadBearingState = new SideDependentList<Boolean>();
   
   public DesiredThighLoadBearingProvider()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         hasNewLoadBearingState.put(robotSide, false);
         loadBearingState.put(robotSide, false);
      }
   }
   
   public synchronized boolean checkForNewLoadBearingState(RobotSide robotSide)
   {
      return hasNewLoadBearingState.get(robotSide);
   }
   
   public synchronized boolean getDesiredThighLoadBearingState(RobotSide robotSide)
   {
      hasNewLoadBearingState.put(robotSide, false);
      
      return loadBearingState.get(robotSide);
   }

   public void consumeObject(ThighStatePacket object)
   {
      RobotSide robotSide = object.getRobotSide();
      hasNewLoadBearingState.put(robotSide, true);
      loadBearingState.put(robotSide, object.isLoadBearing());
   }
}
