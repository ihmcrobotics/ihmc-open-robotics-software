package us.ihmc.commonWalkingControlModules.touchdownDetector;

public interface TouchdownDetector
{
   /** Has touched down according to some criteria. */
   boolean hasTouchedDown();

   /** Has definitely touched down, we don't need any other confirmation */
   boolean hasForSureTouchedDown();

   /**
    * This should only be called once per tick!!
    */
   void update();

   void reset();

   String getName();
}
