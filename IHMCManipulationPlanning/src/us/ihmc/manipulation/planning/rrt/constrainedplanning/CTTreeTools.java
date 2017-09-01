package us.ihmc.manipulation.planning.rrt.constrainedplanning;

import java.util.Random;

import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;

public class CTTreeTools
{
   /**
    * note.
    * packet.setNumberOfFindInitialGuess(100);
      packet.setNumberOfExpanding(600);
      seed = 11.
      timeintentionalratio = 5.0.
      initial stuck.
    */
   static Random randomManager = new Random();
   
   public static void setRandomNormalizedNodeData(CTTaskNode node, boolean isUniform, double treeReachingTime)
   {
      for (int i = 0; i < node.getDimensionOfNodeData(); i++)
         setRandomNormalizedNodeData(node, i, isUniform, treeReachingTime);
   }

   public static void setRandomNormalizedNodeData(CTTaskNode node, int index, boolean isUniform, double treeReachingTime)
   {  
      setRandomNormalizedNodeData(node, randomManager, index, isUniform, treeReachingTime);
   }

   public static void setRandomNormalizedNodeData(CTTaskNode node, Random randomManager, int index, boolean isUniform, double treeReachingTime)
   {
      double exceedIntentionalTimeRatio = 3.0 * treeReachingTime;
      
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
