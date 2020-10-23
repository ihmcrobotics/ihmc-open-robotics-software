package us.ihmc.footstepPlanning.log;

import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;

public class FootstepPlannerEdgeData
{
   private FootstepGraphNode parentNode = null;
   private FootstepGraphNode childNode = null;
   private final FootstepSnapData endStepSnapData = FootstepSnapData.identityData();
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
      parentNode = null;
      childNode = null;
      endStepSnapData.clear();
      solutionEdge = false;
   }

   public FootstepPlannerEdgeData getCopyAndClear()
   {
      FootstepPlannerEdgeData copy = new FootstepPlannerEdgeData(capacity);
      copy.parentNode = parentNode;
      copy.childNode = childNode;
      copy.endStepSnapData.set(endStepSnapData);
      copy.solutionEdge = solutionEdge;
      for (int i = 0; i < dataBuffer.length; i++)
      {
         copy.dataBuffer[i] = dataBuffer[i];
      }

      return copy;
   }

   //////////////// GETTERS ////////////////

   public FootstepGraphNode getParentNode()
   {
      return parentNode;
   }

   public FootstepGraphNode getChildNode()
   {
      return childNode;
   }

   public FootstepSnapData getEndStepSnapData()
   {
      return endStepSnapData;
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

   public void setParentNode(FootstepGraphNode parentNode)
   {
      this.parentNode = parentNode;
   }

   public void setChildNode(FootstepGraphNode childNode)
   {
      this.childNode = childNode;
   }

   public void setEndStepSnapData(FootstepSnapData endStepSnapData)
   {
      this.endStepSnapData.set(endStepSnapData);
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
