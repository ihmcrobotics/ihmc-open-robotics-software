package us.ihmc.commonWalkingControlModules.touchdownDetector;

import java.util.ArrayList;

/**
 * This class is a combination of touchdown sensors. For it to detect touchdown, any of the component touchdown sensors is "sufficient", meaning that only one
 * must detect touchdown.
 */
public class SufficientTouchdownDetectors implements TouchdownDetector
{
   private final ArrayList<TouchdownDetector> touchdownDetectors = new ArrayList<>();

   public void addTouchdownDetector(TouchdownDetector touchdownDetector)
   {
      touchdownDetectors.add(touchdownDetector);
   }

   @Override
   public boolean hasTouchedDown()
   {
      for (int i = 0; i < touchdownDetectors.size(); i++)
      {
         if (touchdownDetectors.get(i).hasTouchedDown())
            return true;
      }

      return false;
   }

   @Override
   public boolean hasForSureTouchedDown()
   {
      for (int i = 0; i < touchdownDetectors.size(); i++)
      {
         if (touchdownDetectors.get(i).hasForSureTouchedDown())
            return true;
      }

      return false;
   }

   @Override
   public void update()
   {
      for (int i = 0; i < touchdownDetectors.size(); i++)
         touchdownDetectors.get(i).update();
   }

   @Override
   public void reset()
   {
      for (int i = 0; i < touchdownDetectors.size(); i++)
         touchdownDetectors.get(i).reset();
   }

   @Override
   public String getName()
   {
      return "SufficientTouchdownDetectors";
   }
}
