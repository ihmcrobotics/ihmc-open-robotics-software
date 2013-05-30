package us.ihmc.commonWalkingControlModules.packets;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.net.ObjectConsumer;

public class DesiredHighLevelStateProvider implements ObjectConsumer<HighLevelStatePacket>
{

   private boolean hasNewState = false;
   private HighLevelState highLevelState = HighLevelState.WALKING;

   public DesiredHighLevelStateProvider()
   {
   }
   
   public synchronized boolean checkForNewState()
   {
      return hasNewState;
   }
   
   public synchronized HighLevelState getDesiredHighLevelState()
   {
      hasNewState = false;
      
      return highLevelState;
   }
   
   public void consumeObject(HighLevelStatePacket object)
   {
      hasNewState = true;

      highLevelState = object.getHighLevelState();
   }

}
