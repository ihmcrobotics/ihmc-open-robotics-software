package us.ihmc.commonWalkingControlModules.touchdownDetector;

import us.ihmc.robotics.math.filters.WeightedAverageYoBoolean;
import us.ihmc.tools.lists.PairList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.ArrayList;

/**
 * This class is a combination of touchdown sensors. For it to detect touchdown, the weighted average of all the component touchdown sensors must be true.
 */
public class WeightedAverageTouchdownDetectors implements TouchdownDetector
{
   private final ArrayList<TouchdownDetector> touchdownDetectors = new ArrayList<>();
   private final WeightedAverageYoBoolean hasTouchedDown;

   private final String prefix;

   public WeightedAverageTouchdownDetectors(PairList<DoubleProvider, TouchdownDetector> touchdownDetectors, String prefix, YoVariableRegistry registry)
   {
      this.prefix = prefix;

      hasTouchedDown = new WeightedAverageYoBoolean(prefix + "_HasTouchedDown", registry);
      for (int i = 0; i < touchdownDetectors.size(); i++)
      {
         TouchdownDetector touchdownDetector = touchdownDetectors.get(i).getRight();
         YoBoolean oneHasTouchedDown = new YoBoolean(prefix + touchdownDetector.getName() + "_HasTouchedDown", registry);
         hasTouchedDown.addBooleanToAverage(touchdownDetectors.get(i).getLeft(), oneHasTouchedDown);

         this.touchdownDetectors.add(touchdownDetector);
      }
   }


   @Override
   public boolean hasTouchedDown()
   {
      return hasTouchedDown.getBooleanValue();
   }

   @Override
   public void update()
   {
      for (int i = 0; i < touchdownDetectors.size(); i++)
         touchdownDetectors.get(i).update();

      hasTouchedDown.update();
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
      return prefix + "WeightedAverageTouchdownDetector";
   }
}
