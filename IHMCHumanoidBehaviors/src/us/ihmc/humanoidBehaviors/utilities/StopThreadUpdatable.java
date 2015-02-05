package us.ihmc.humanoidBehaviors.utilities;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;

public abstract class StopThreadUpdatable implements Updatable
{
   protected boolean stopThread = false;

   public boolean stopThread()
   {
      return stopThread;
   }
}
