package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.ArrayList;
import java.util.List;

public class SpatialNodeTree
{
   private SpatialNode rootNode;
   private double mostAdvancedTime = 0.0;
   private List<SpatialNode> validNodes = new ArrayList<>();

   private List<SpatialNode> invalidNodes = new ArrayList<>();

   private double timeWeight = 0.5;
   private double positionWeight = 1.0;
   private double orientationWeight = 1.0;

   private double maxTimeInterval = 1 / 2.0;
   private double maxPositionDistance = 0.05;
   private double maxOrientationDistance = Math.toRadians(10.0);
   
//   private double maxTimeInterval = 1 / 3.0;
//   private double maxPositionDistance = 0.08;
//   private double maxOrientationDistance = Math.toRadians(25);

   private SpatialNode currentCandidate = null;
   private SpatialNode currentCandidateParent = null;

   private SpatialNode randomNode = null;

   public SpatialNodeTree()
   {

   }

   public SpatialNodeTree(SpatialNode rootNode)
   {
      this.rootNode = rootNode;
      validNodes.add(rootNode);
   }

   public void addInitialNode(SpatialNode node)
   {
      validNodes.add(node);
   }

   public void setRandomNode(SpatialNode node)
   {
      randomNode = node;
   }

   public SpatialNode getCandidate()
   {
      return currentCandidate;
   }

   public boolean findNearestValidNodeToCandidate(boolean includeTimeComparison)
   {
      double distanceToNearestNode = Double.MAX_VALUE;
      SpatialNode nearestNode = null;

      for (SpatialNode candidateForParent : validNodes)
      {
         if (randomNode.getTime() < candidateForParent.getTime())
            continue;

         double distance;

         if (includeTimeComparison)
            //distance = candidateForParent.computeDistance(timeWeight, positionWeight, orientationWeight, randomNode);
            distance = candidateForParent.computeDistanceWithinMaxDistance(timeWeight, positionWeight, orientationWeight, randomNode, maxTimeInterval,
                                                                           maxPositionDistance, maxOrientationDistance);
         else
            //distance = candidateForParent.computeDistance(0.0, positionWeight, orientationWeight, randomNode);
            distance = candidateForParent.computeDistanceWithinMaxDistance(0.0, positionWeight, orientationWeight, randomNode, maxTimeInterval,
                                                                           maxPositionDistance, maxOrientationDistance);

         if (distance < distanceToNearestNode)
         {
            distanceToNearestNode = distance;
            nearestNode = candidateForParent;
         }
      }

      if (nearestNode == null)
      {
         return false;
      }
      else
      {
         currentCandidateParent = nearestNode;
         return true;
      }
   }

   public void limitCandidateDistanceFromParent(double trajectoryTime)
   {
      //currentCandidate = currentCandidateParent.createNodeWithinMaxDistance(maxTimeInterval, maxPositionDistance, maxOrientationDistance, randomNode);
      currentCandidate = currentCandidateParent.createNodeWithinTimeStep(maxTimeInterval, randomNode);
      if (currentCandidate.getTime() > trajectoryTime)
         currentCandidate.setTime(trajectoryTime);
      currentCandidate.setParent(currentCandidateParent);
   }

   public void attachCandidate()
   {
      if (!currentCandidate.isValid())
         throw new RuntimeException("Should only attach valid nodes to this tree.");

      mostAdvancedTime = Math.max(currentCandidate.getTime(), mostAdvancedTime);
      validNodes.add(new SpatialNode(currentCandidate));
   }

   public void attachCandidate(SpatialNode node)
   {
      mostAdvancedTime = Math.max(node.getTime(), mostAdvancedTime);
      validNodes.add(new SpatialNode(node));
   }

   public void dismissCandidate()
   {
      if (currentCandidate.isValid())
         throw new RuntimeException("Should attach valid nodes to this tree.");

      currentCandidate.clearParent();
      invalidNodes.add(currentCandidate);
   }

   public SpatialNode getLastNodeAdded()
   {
      return validNodes.get(validNodes.size() - 1);
   }

   public double getTimeWeight()
   {
      return timeWeight;
   }

   public double getPositionWeight()
   {
      return positionWeight;
   }

   public double getOrientationWeight()
   {
      return orientationWeight;
   }

   public double getMaxTimeInterval()
   {
      return maxTimeInterval;
   }

   public double getMaxPositionDistance()
   {
      return maxPositionDistance;
   }

   public double getMaxOrientationDistance()
   {
      return maxOrientationDistance;
   }

   public void setTimeWeight(double timeWeight)
   {
      this.timeWeight = timeWeight;
   }

   public void setPositionWeight(double positionWeight)
   {
      this.positionWeight = positionWeight;
   }

   public void setOrientationWeight(double orientationWeight)
   {
      this.orientationWeight = orientationWeight;
   }

   public void setMaxTimeInterval(double maxTimeInterval)
   {
      this.maxTimeInterval = maxTimeInterval;
   }

   public void setMaxPositionDistance(double maxPositionDistance)
   {
      this.maxPositionDistance = maxPositionDistance;
   }

   public void setMaxOrientationDistance(double maxOrientationDistance)
   {
      this.maxOrientationDistance = maxOrientationDistance;
   }

   public List<SpatialNode> getValidNodes()
   {
      return validNodes;
   }

   public double getMostAdvancedTime()
   {
      return mostAdvancedTime;
   }
}
