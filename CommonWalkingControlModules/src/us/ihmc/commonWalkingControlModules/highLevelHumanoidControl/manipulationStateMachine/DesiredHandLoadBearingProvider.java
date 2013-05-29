package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredHandLoadBearingProvider implements ObjectConsumer<HandLoadBearingPacket>
{
   private SideDependentList<Boolean> hasLoadBearingBeenRequested = new SideDependentList<Boolean>();
   
   public DesiredHandLoadBearingProvider()
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

   public void consumeObject(HandLoadBearingPacket object)
   {
      RobotSide robotSide = object.getRobotSide();
      hasLoadBearingBeenRequested.put(robotSide, true);
   }
}
