package us.ihmc.manipulation.planning.rrt.constrainedplanning.configurationAndTimeSpace;

import java.util.ArrayList;
import java.util.List;

public class SpatialNodeTree
{
   private SpatialNode rootNode;
   private double mostAdvancedTime = 0.0;
   private List<SpatialNode> validNodes = new ArrayList<>();

   private List<SpatialNode> invalidNodes = new ArrayList<>();

   private double timeWeight = 2.0;
   private double positionWeight = 0.5;
   private double orientationWeight = 1.5;

   private double maxTimeInterval = 0.25;
   private double maxPositionDistance = 0.30;
   private double maxOrientationDistance = Math.toRadians(45.0);

   private SpatialNode currentCandidate = null;
   private SpatialNode currentCandidateParent = null;
   private SpatialNode lastNodeAdded = null;
   private SpatialNode mostAdvancedNode = null;

   public SpatialNodeTree(SpatialNode rootNode)
   {
      this.rootNode = rootNode;
      lastNodeAdded = rootNode;
      mostAdvancedNode = rootNode;
      validNodes.add(rootNode);
   }

   public void setCandidate(SpatialNode candidate)
   {
      currentCandidate = candidate;
   }

   public SpatialNode getCandidate()
   {
      return currentCandidate;
   }

   public SpatialNode getMostAdvancedNode()
   {
      return mostAdvancedNode;
   }

   public void findNearestValidNodeToCandidate(boolean includeTimeComparison)
   {
      double distanceToNearestNode = Double.POSITIVE_INFINITY;
      SpatialNode nearestNode = null;

      for (SpatialNode candidate : validNodes)
      {
         if (currentCandidate.getTime() < candidate.getTime())
            continue;

         double distance;
         if (includeTimeComparison)
            distance = candidate.computeDistance(timeWeight, positionWeight, orientationWeight, currentCandidate);
         else
            distance = candidate.computeDistance(0.0, positionWeight, orientationWeight, currentCandidate);

         if (distance < distanceToNearestNode)
         {
            distanceToNearestNode = distance;
            nearestNode = candidate;
         }
      }

      currentCandidateParent = nearestNode;
   }

   public SpatialNode limitCandidateDistanceFromParent()
   {
      currentCandidate = currentCandidateParent.createNodeWithinMaxDistance(maxTimeInterval, maxPositionDistance, maxOrientationDistance, currentCandidate);
      return currentCandidate;
   }

   public void attachCandidate()
   {
      if (!currentCandidate.isValid())
         throw new RuntimeException("Should only attach valid nodes to this tree.");

      lastNodeAdded = currentCandidate;
      if (mostAdvancedNode == null || currentCandidate.getTime() > mostAdvancedNode.getTime())
         mostAdvancedNode = currentCandidate;
      validNodes.add(currentCandidate);
      currentCandidateParent.addChild(currentCandidate);
      currentCandidate = null;
      currentCandidateParent = null;
   }

   public void dismissCandidate()
   {
      if (currentCandidate.isValid())
         throw new RuntimeException("Should attach valid nodes to this tree.");

      invalidNodes.add(currentCandidate);
      currentCandidate = null;
      currentCandidateParent = null;
   }

   public SpatialNode getLastNodeAdded()
   {
      return lastNodeAdded;
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
