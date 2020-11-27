package us.ihmc.humanoidBehaviors.tools.behaviorTree.utility;

import us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeAction;

import java.util.ArrayList;

/**
 * Based on Dave Mark Architecture Tricks: Managing Behaviors in Time, Space, and Depth
 * https://gdcvault.com/play/1018040
 */
public abstract class UtilityBasedAction implements BehaviorTreeAction
{
   private final ArrayList<UtilityAxis> axes = new ArrayList<>();

   @Override
   public double evaluateUtility()
   {
      if (axes.isEmpty())
      {
         return 0.0;
      }

      double utility = 1.0;
      for (UtilityAxis axis : axes)
      {
         utility *= axis.calculate();
      }

      return utility;
   }

   public void addUtilityAxis(UtilityAxis axis)
   {
      axes.add(axis);
   }
}
