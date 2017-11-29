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
   private double orientationWeight = 0.5;

   public double dismissableTimeStep = 0.05;
   private double maxTimeInterval = 0.2;
   private double maxPositionDistance = 0.01;
   private double maxOrientationDistance = Math.toRadians(5);

   private SpatialNode currentCandidate = null;
   private SpatialNode currentCandidateParent = null;

   private SpatialNode randomNode = null;

   public SpatialNodeTree(SpatialNode rootNode)
   {
      this.rootNode = rootNode;
      validNodes.add(rootNode);
   }

   public void setRandomNode(SpatialNode node)
   {
      randomNode = node;
   }

   public SpatialNode getCandidate()
   {
      return currentCandidate;
   }

   public double getDistance(SpatialNode nodeOne, SpatialNode nodeTwo)
   {
      return nodeOne.computeDistance(timeWeight, positionWeight, orientationWeight, nodeTwo);
   }

   public void findNearestValidNodeToCandidate(boolean includeTimeComparison)
   {
      double distanceToNearestNode = Double.POSITIVE_INFINITY;
      SpatialNode nearestNode = null;

      for (SpatialNode candidateForParent : validNodes)
      {
         if (randomNode.getTime() < candidateForParent.getTime())
            continue;

         double distance;

         if (includeTimeComparison)
            distance = candidateForParent.computeDistance(timeWeight, positionWeight, orientationWeight, randomNode);
         else
            distance = candidateForParent.computeDistance(0.0, positionWeight, orientationWeight, randomNode);

         if (distance < distanceToNearestNode)
         {
            distanceToNearestNode = distance;
            nearestNode = candidateForParent;
         }
      }

      currentCandidateParent = nearestNode;
   }

   public void limitCandidateDistanceFromParent(double trajectoryTime)
   {
      currentCandidate = currentCandidateParent.createNodeWithinMaxDistance(maxTimeInterval, maxPositionDistance, maxOrientationDistance, randomNode);
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
