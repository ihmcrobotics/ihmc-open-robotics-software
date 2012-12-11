package us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity;

import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.Updatable;

public class DesiredHeadingUpdater implements Updatable
{
   private DesiredHeadingControlModule desiredHeadingControlModule;

   public DesiredHeadingUpdater(DesiredHeadingControlModule desiredHeadingControlModule)
   {
      this.desiredHeadingControlModule = desiredHeadingControlModule;
   }

   public void update(double time)
   {
      desiredHeadingControlModule.updateDesiredHeadingFrame();
   }

}
