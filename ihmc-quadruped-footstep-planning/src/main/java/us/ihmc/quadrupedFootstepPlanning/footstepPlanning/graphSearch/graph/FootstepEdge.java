package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public class FootstepEdge
{
   private final FootstepNode startNode;
   private final FootstepNode endNode;

   private final int hashCode;

   public FootstepEdge(FootstepNode startNode, FootstepNode endNode)
   {
      this.startNode = startNode;
      this.endNode = endNode;

      hashCode = computeHashCode(this);
   }

   public FootstepNode getStartNode()
   {
      return startNode;
   }

   public FootstepNode getEndNode()
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

   private static int computeHashCode(FootstepEdge edge)
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((edge.endNode == null) ? 0 : edge.endNode.fullHashCode());
      result = prime * result + ((edge.startNode == null) ? 0 : edge.startNode.fullHashCode());
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
      FootstepEdge other = (FootstepEdge) obj;
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
