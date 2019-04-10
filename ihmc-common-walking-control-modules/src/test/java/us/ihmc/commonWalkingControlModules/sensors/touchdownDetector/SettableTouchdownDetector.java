package us.ihmc.commonWalkingControlModules.sensors.touchdownDetector;

import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;

class SettableTouchdownDetector implements TouchdownDetector
{
   private boolean hasTouchedDown = false;
   private boolean hasForSureTouchedDown = false;

   public void setHasTouchedDown(boolean hasTouchedDown)
   {
      this.hasTouchedDown = hasTouchedDown;
   }

   public void setHasForSureTouchedDown(boolean hasForSureTouchedDown)
   {
      this.hasForSureTouchedDown = hasForSureTouchedDown;
   }

   @Override
   public boolean hasTouchedDown()
   {
      return hasTouchedDown;
   }

   @Override
   public boolean hasForSureTouchedDown()
   {
      return hasForSureTouchedDown;
   }

   @Override
   public void update()
   {

   }

   @Override
   public void reset()
   {

   }

   @Override
   public String getName()
   {
      return null;
   }
}
