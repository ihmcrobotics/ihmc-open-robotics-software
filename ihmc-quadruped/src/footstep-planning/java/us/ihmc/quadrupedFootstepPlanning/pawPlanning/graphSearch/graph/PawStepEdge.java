package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public class PawStepEdge
{
   private final PawNode startNode;
   private final PawNode endNode;

   private final int hashCode;

   public PawStepEdge(PawNode startNode, PawNode endNode)
   {
      this.startNode = startNode;
      this.endNode = endNode;

      hashCode = computeHashCode(this);
   }

   public PawNode getStartNode()
   {
      return startNode;
   }

   public PawNode getEndNode()
   {
      return endNode;
   }

   public boolean isValidEdge()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (robotQuadrant == endNode.getMovingQuadrant())
            continue;

         if (!startNode.quadrantGeometricallyEquals(robotQuadrant, endNode))
            return false;
      }

      return true;
   }

   @Override
   public int hashCode()
   {
      return hashCode;
   }

   private static int computeHashCode(PawStepEdge edge)
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((edge.endNode == null) ? 0 : edge.endNode.hashCode());
      result = prime * result + ((edge.startNode == null) ? 0 : edge.startNode.hashCode());
      return result;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      PawStepEdge other = (PawStepEdge) obj;
      if (endNode == null)
      {
         if (other.endNode != null)
            return false;
      }
      else if (!endNode.equals(other.endNode))
         return false;
      if (startNode == null)
      {
         if (other.startNode != null)
            return false;
      }
      else if (!startNode.equals(other.startNode))
         return false;
      return true;
   }

}
