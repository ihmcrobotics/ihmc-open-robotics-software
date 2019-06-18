package us.ihmc.commonWalkingControlModules.sensors.touchdownDetector;

import us.ihmc.commonWalkingControlModules.touchdownDetector.TouchdownDetector;

class SettableTouchdownDetector implements TouchdownDetector
{
   private boolean hasTouchedDown = false;

   public void setHasTouchedDown(boolean hasTouchedDown)
   {
      this.hasTouchedDown = hasTouchedDown;
   }

   @Override
   public boolean hasTouchedDown()
   {
      return hasTouchedDown;
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
