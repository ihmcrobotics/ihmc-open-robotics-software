package us.ihmc.humanoidBehaviors.behaviors.rrtPlanner;

import java.util.Random;

import us.ihmc.manipulation.planning.rrt.RRTNode;
import us.ihmc.manipulation.planning.rrt.RRTTree;

public class TimeDomainTree extends RRTTree
{
   protected double motionMaximumTime;
   protected double timeScaleForMatric = 0.5;

   protected double maximumDisplacementOfStep = 0.5;   
   protected double maximumTimeGapOfStep = 0.8;

   public TimeDomainTree(RRTNode rootNode)
   {
      super(rootNode);
   }

   public void setMaximumMotionTime(double motionTime)
   {
      this.motionMaximumTime = motionTime;
   }

   public void setMaximumDisplacementOfStep(double maximumDisplacementOfStep)
   {
      this.maximumDisplacementOfStep = maximumDisplacementOfStep;
   }
   
   public void setMaximumTimeGapOfStep(double maximumTimeGapOfStep)
   {
      this.maximumTimeGapOfStep = maximumTimeGapOfStep;
   }
   
   public void setTimeScaleForMatric(double timeScaleForMatric)
   {
      this.timeScaleForMatric = timeScaleForMatric;
   }
   
   public double getMotionTime()
   {
      return motionMaximumTime;
   }
      
   public double getTime(RRTNode node)
   {
      return node.getNodeData(0);
   }
      
   public double getDisplacement(RRTNode predecessorNode, RRTNode successorNode)
   {
      double squaredDisplacement = 0;
      
      for(int i=1;i<successorNode.getDimensionOfNodeData();i++)
      {
         squaredDisplacement = squaredDisplacement + (successorNode.getNodeData(i)-predecessorNode.getNodeData(i))*(successorNode.getNodeData(i)-predecessorNode.getNodeData(i));
      }
      
      double displacement = Math.sqrt(squaredDisplacement);
      
      return displacement;
   }
   
   public double getSpeed(RRTNode predecessorNode, RRTNode successorNode)
   {
      if(successorNode.getNodeData(0) - predecessorNode.getNodeData(0) > 0)
      {
         return getDisplacement(predecessorNode, successorNode)/(successorNode.getNodeData(0) - predecessorNode.getNodeData(0));         
      }
      else
      {  
         return 0;
      }
   }
   
   @Override
   public RRTNode getRandomNode()
   {
      RRTNode randomNode = nodeCreator.createNode();

      Random randomManager = new Random();

      for (int i = 0; i < rootNode.getDimensionOfNodeData(); i++)
      {
         double randonValue = randomManager.nextDouble() * (this.upperBoundNode.getNodeData(i) - this.lowerBoundNode.getNodeData(i))
               + this.lowerBoundNode.getNodeData(i);
         randomNode.setNodeData(i, randonValue);
      }

      if (randomNode.getNodeData(0) > motionMaximumTime)
      {
         randomNode.setNodeData(0, motionMaximumTime);
      }

      return randomNode;
   }

   @Override
   public double getMatric(RRTNode predecessorNode, RRTNode successorNode)
   {
      double matric;
      
      if(successorNode.getNodeData(0) - predecessorNode.getNodeData(0) > 0)
      {
         double squaredMatric = timeScaleForMatric*(successorNode.getNodeData(0)-predecessorNode.getNodeData(0))*(successorNode.getNodeData(0)-predecessorNode.getNodeData(0));

         squaredMatric = squaredMatric + getDisplacement(predecessorNode, successorNode)*getDisplacement(predecessorNode, successorNode);
         
         matric = Math.sqrt(squaredMatric);
      }
      else
      {
         matric = Double.MAX_VALUE;    
      }      
      
      return matric;
   }
}
