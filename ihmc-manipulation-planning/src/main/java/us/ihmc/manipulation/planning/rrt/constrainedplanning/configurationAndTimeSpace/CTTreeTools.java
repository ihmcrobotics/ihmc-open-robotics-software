package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.Random;

import us.ihmc.robotics.robotSide.RobotSide;

public class CTTreeTools
{
   /*
    * 50 okay . why 1 has problem? angular velocity problem on chest.
    */
   static Random randomManager = new Random(3);
   static double timeCoefficient = 3.0;

   public static void setRandomNormalizedNodeData(CTTaskNode node, boolean isUniform, double treeReachingTime)
   {
      for (int i = 0; i < node.getDimensionOfNodeData(); i++)
         setRandomNormalizedNodeData(node, i, isUniform, treeReachingTime);
   }

   public static void setRandomNormalizedNodeData(CTTaskNode node, int index, boolean isUniform, double treeReachingTime)
   {
      //double exceedIntentionalTimeRatio = 3.0;
      double exceedIntentionalTimeRatio = timeCoefficient * treeReachingTime;
      // double exceedIntentionalTimeRatio = 3.0 * treeReachingTime;

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

   public static void setRandomConfigurationData(CTTaskNode node, int index, double lowerLimit, double upperLimit, boolean isUniform)
   {
      double exceedIntentionalRatio = 0.5;

      if (isUniform)
         exceedIntentionalRatio = 0.0;
      else
         exceedIntentionalRatio = 1.0;

      double value;

      value = randomManager.nextDouble() * (1.0 + exceedIntentionalRatio);
      value = value - 0.5 * exceedIntentionalRatio;

      if (value >= 1)
         value = 1;
      if (value <= 0)
         value = 0;

      double randomConfigurationData = lowerLimit + value * (upperLimit - lowerLimit);
      node.setNodeData(index, randomConfigurationData);
   }

   public static void setRandomTimeData(CTTaskNode node, double trajectoryTime, double treeReachingTime)
   {
      double exceedIntentionalTimeRatio = timeCoefficient * treeReachingTime;

      double value;

      value = randomManager.nextDouble() * (1.0 + exceedIntentionalTimeRatio);

      if (value > 1)
         value = 1;

      double randomTimeData = value * trajectoryTime;
      node.setTimeData(randomTimeData);
   }
}