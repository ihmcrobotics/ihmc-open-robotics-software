package us.ihmc.behaviors.behaviorTree.utility;

import us.ihmc.behaviors.behaviorTree.LocalOnlyBehaviorTreeNodeExecutor;

import java.util.ArrayList;
import java.util.List;

/**
 * Based on Dave Mark Architecture Tricks: Managing Behaviors in Time, Space, and Depth
 * https://gdcvault.com/play/1018040
 */
public abstract class UtilityBasedAction extends LocalOnlyBehaviorTreeNodeExecutor
{
   private final ArrayList<UtilityAxis> axes = new ArrayList<>();

   private final List<UtilityBasedAction> children = new ArrayList<>();

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

   // TODO: Fix
   public List<UtilityBasedAction> getUtilityChildren()
   {
      return children;
   }
}
