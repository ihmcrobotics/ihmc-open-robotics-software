package us.ihmc.manipulation.planning.rrt.constrainedplanning;

import java.util.Random;

import us.ihmc.humanoidRobotics.communication.packets.manipulation.constrainedWholeBodyPlanning.ConfigurationSpace;
import us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace.CTTaskNode;
import us.ihmc.robotics.robotSide.RobotSide;

public class CTTreeTools
{
   /*
    * 50 okay . why 1 has problem?
    * angular velocity problem on chest.
    */
   static Random randomManager = new Random(2);
   
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

   public static ConfigurationSpace getConfigurationSpace(CTTaskNode node, RobotSide robotSide)
   {
      ConfigurationSpace configurationSpace = new ConfigurationSpace();
      if(robotSide == RobotSide.LEFT)
      {
         configurationSpace.setTranslation(node.getNodeData(5), node.getNodeData(6), node.getNodeData(7));
         configurationSpace.setRotation(node.getNodeData(8), node.getNodeData(9), node.getNodeData(10));
      }
      else
      {
         configurationSpace.setTranslation(node.getNodeData(11), node.getNodeData(12), node.getNodeData(13));
         configurationSpace.setRotation(node.getNodeData(14), node.getNodeData(15), node.getNodeData(16));
      }
      
      return configurationSpace;
   }
}
