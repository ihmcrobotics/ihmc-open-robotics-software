package us.ihmc.footstepPlanning.log;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.mutable.MutableDouble;
import org.apache.commons.lang3.mutable.MutableObject;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;

public class FootstepPlannerEdgeData
{
   private FootstepNode stanceNode = null;
   private FootstepNode candidateNode = null;
   private final FootstepNodeSnapData candidateNodeSnapData = FootstepNodeSnapData.identityData();

   private BipedalFootstepPlannerNodeRejectionReason rejectionReason = null;
   private double footAreaPercentage = Double.NaN;
   private double stepWidth = Double.NaN;
   private double stepLength = Double.NaN;
   private double stepHeight = Double.NaN;
   private double stepReach = Double.NaN;
   private double costFromStart = Double.NaN;
   private double edgeCost = Double.POSITIVE_INFINITY;
   private double heuristicCost = Double.NaN;
   private boolean solutionEdge = false;
   private int stepIndex = -1;

   public void clear()
   {
      stanceNode = null;
      candidateNode = null;
      candidateNodeSnapData.clear();
      rejectionReason = null;
      footAreaPercentage = Double.NaN;
      stepWidth = Double.NaN;
      stepLength = Double.NaN;
      stepHeight = Double.NaN;
      stepReach = Double.NaN;
      costFromStart = Double.NaN;
      edgeCost = Double.POSITIVE_INFINITY;
      heuristicCost = Double.NaN;
      solutionEdge = false;
      stepIndex = -1;
   }

   public FootstepPlannerEdgeData getCopyAndClear()
   {
      FootstepPlannerEdgeData copy = new FootstepPlannerEdgeData();
      copy.stanceNode = stanceNode;
      copy.candidateNode = candidateNode;
      copy.candidateNodeSnapData.set(candidateNodeSnapData);
      copy.rejectionReason = rejectionReason;
      copy.footAreaPercentage = footAreaPercentage;
      copy.stepWidth = stepWidth;
      copy.stepLength = stepLength;
      copy.stepHeight = stepHeight;
      copy.stepReach = stepReach;
      copy.costFromStart = costFromStart;
      copy.edgeCost = edgeCost;
      copy.heuristicCost = heuristicCost;
      copy.solutionEdge = solutionEdge;
      copy.stepIndex = stepIndex;
      clear();
      return copy;
   }

   //////////////// GETTERS ////////////////

   public FootstepNode getStanceNode()
   {
      return stanceNode;
   }

   public FootstepNode getCandidateNode()
   {
      return candidateNode;
   }

   public FootstepNodeSnapData getCandidateNodeSnapData()
   {
      return candidateNodeSnapData;
   }

   public BipedalFootstepPlannerNodeRejectionReason getRejectionReason()
   {
      return rejectionReason;
   }

   public double getFootAreaPercentage()
   {
      return footAreaPercentage;
   }

   public double getStepWidth()
   {
      return stepWidth;
   }

   public double getStepLength()
   {
      return stepLength;
   }

   public double getStepHeight()
   {
      return stepHeight;
   }

   public double getStepReach()
   {
      return stepReach;
   }

   public double getCostFromStart()
   {
      return costFromStart;
   }

   public double getEdgeCost()
   {
      return edgeCost;
   }

   public double getHeuristicCost()
   {
      return heuristicCost;
   }

   public boolean getSolutionEdge()
   {
      return solutionEdge;
   }

   public int getStepIndex()
   {
      return stepIndex;
   }

   //////////////// SETTERS ////////////////

   public void setStanceNode(FootstepNode stanceNode)
   {
      this.stanceNode = stanceNode;
   }

   public void setCandidateNode(FootstepNode candidateNode)
   {
      this.candidateNode = candidateNode;
   }

   public void setCandidateNodeSnapData(FootstepNodeSnapData candidateNodeSnapData)
   {
      this.candidateNodeSnapData.set(candidateNodeSnapData);
   }

   public void setRejectionReason(BipedalFootstepPlannerNodeRejectionReason rejectionReason)
   {
      this.rejectionReason = rejectionReason;
   }

   public void setFootAreaPercentage(double footAreaPercentage)
   {
      this.footAreaPercentage = footAreaPercentage;
   }

   public void setStepWidth(double stepWidth)
   {
      this.stepWidth = stepWidth;
   }

   public void setStepLength(double stepLength)
   {
      this.stepLength = stepLength;
   }

   public void setStepHeight(double stepHeight)
   {
      this.stepHeight = stepHeight;
   }

   public void setStepReach(double stepReach)
   {
      this.stepReach = stepReach;
   }

   public void setCostFromStart(double costFromStart)
   {
      this.costFromStart = costFromStart;
   }

   public void setEdgeCost(double edgeCost)
   {
      this.edgeCost = edgeCost;
   }

   public void setHeuristicCost(double heuristicCost)
   {
      this.heuristicCost = heuristicCost;
   }

   public void setSolutionEdge(boolean solutionEdge)
   {
      this.solutionEdge = solutionEdge;
   }

   public void setStepIndex(int stepIndex)
   {
      this.stepIndex = stepIndex;
   }
}
