package us.ihmc.commonWalkingControlModules.touchdownDetector;

public interface TouchdownDetector
{
   boolean hasTouchedDown();

   /**
    * This should only be called once per tick!!
    */
   void update();

   void reset();

   String getName();
}
