package us.ihmc.manipulation.planning.rrt.constrainedplanning;

import java.util.Random;

import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;

public class CTTreeTools
{
   public static void setRandomNormalizedNodeData(CTTaskNode node, boolean isUniform)
   {
      for (int i = 0; i < node.getDimensionOfNodeData(); i++)
         setRandomNormalizedNodeData(node, i, isUniform);
   }

   public static void setRandomNormalizedNodeData(CTTaskNode node, int index, boolean isUniform)
   {
      Random randomManager = new Random();
      setRandomNormalizedNodeData(node, randomManager, index, isUniform);
   }

   public static void setRandomNormalizedNodeData(CTTaskNode node, Random randomManager, int index, boolean isUniform)
   {
      double exceedIntentionalTimeRatio = 1.0;
      double exceedIntentionalRatio = 0.5;

      if (isUniform)
         exceedIntentionalRatio = 0.0;
      else
         exceedIntentionalRatio = 1.0;

      double value;

      if (index == 0)
         value = randomManager.nextDouble() * (1.0 + exceedIntentionalTimeRatio);
      else
      {
         value = randomManager.nextDouble() * (1.0 + exceedIntentionalRatio);
         value = value - 0.5 * exceedIntentionalRatio;

         if (value >= 1)
            value = 1;
         if (value <= 0)
            value = 0;
      }

      node.setNormalizedNodeData(index, value);
   }

}
