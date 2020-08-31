package us.ihmc.footstepPlanning.log;

import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;

public class FootstepPlannerEdgeData
{
   private FootstepNode stanceNode = null;
   private FootstepNode candidateNode = null;
   private final FootstepNodeSnapData candidateNodeSnapData = FootstepNodeSnapData.identityData();
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

   public long[] getDataBuffer()
   {
      return dataBuffer;
   }

   public boolean isSolutionEdge()
   {
      return solutionEdge;
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

   public void setSolutionEdge(boolean solutionEdge)
   {
      this.solutionEdge = solutionEdge;
   }

   public void setData(int index, long data)
   {
      dataBuffer[index] = data;
   }
}
