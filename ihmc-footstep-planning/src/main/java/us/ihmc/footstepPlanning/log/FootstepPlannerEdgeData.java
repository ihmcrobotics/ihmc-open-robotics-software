package us.ihmc.footstepPlanning.log;

import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;

public class FootstepPlannerEdgeData
{
   private DiscreteFootstep stanceNode = null;
   private DiscreteFootstep candidateNode = null;
   private final FootstepSnapData candidateNodeSnapData = FootstepSnapData.identityData();
   private boolean solutionEdge = false;

   private final int capacity;
   private final long[] dataBuffer;

   public FootstepPlannerEdgeData(int bufferCapacity)
   {
      capacity = bufferCapacity;
      dataBuffer = new long[bufferCapacity];
   }

   public void clear()
   {
      stanceNode = null;
      candidateNode = null;
      candidateNodeSnapData.clear();
      solutionEdge = false;
   }

   public FootstepPlannerEdgeData getCopyAndClear()
   {
      FootstepPlannerEdgeData copy = new FootstepPlannerEdgeData(capacity);
      copy.stanceNode = stanceNode;
      copy.candidateNode = candidateNode;
      copy.candidateNodeSnapData.set(candidateNodeSnapData);
      copy.solutionEdge = solutionEdge;
      for (int i = 0; i < dataBuffer.length; i++)
      {
         copy.dataBuffer[i] = dataBuffer[i];
      }

      return copy;
   }

   //////////////// GETTERS ////////////////

   public DiscreteFootstep getStanceNode()
   {
      return stanceNode;
   }

   public DiscreteFootstep getCandidateNode()
   {
      return candidateNode;
   }

   public FootstepSnapData getCandidateNodeSnapData()
   {
      return candidateNodeSnapData;
   }

   public long[] getDataBuffer()
   {
      return dataBuffer;
   }

   public boolean isSolutionEdge()
   {
      return solutionEdge;
   }

   //////////////// SETTERS ////////////////

   public void setStanceNode(DiscreteFootstep stanceNode)
   {
      this.stanceNode = stanceNode;
   }

   public void setCandidateNode(DiscreteFootstep candidateNode)
   {
      this.candidateNode = candidateNode;
   }

   public void setCandidateNodeSnapData(FootstepSnapData candidateNodeSnapData)
   {
      this.candidateNodeSnapData.set(candidateNodeSnapData);
   }

   public void setSolutionEdge(boolean solutionEdge)
   {
      this.solutionEdge = solutionEdge;
   }

   public void setData(int index, long data)
   {
      dataBuffer[index] = data;
   }
}
