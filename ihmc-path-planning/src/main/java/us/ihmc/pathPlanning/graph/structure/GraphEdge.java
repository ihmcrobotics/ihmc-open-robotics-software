package us.ihmc.pathPlanning.graph.structure;

public class GraphEdge<N>
{
   private final N startNode;
   private final N endNode;

   private final int hashCode;

   public GraphEdge(N startNode, N endNode)
   {
      this.startNode = startNode;
      this.endNode = endNode;

      hashCode = computeHashCode(this);
   }

   public N getStartNode()
   {
      return startNode;
   }

   public N getEndNode()
   {
      return endNode;
   }

   @Override
   public int hashCode()
   {
      return hashCode;
   }

   private static int computeHashCode(GraphEdge edge)
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
      GraphEdge other = (GraphEdge) obj;
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
