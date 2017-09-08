package us.ihmc.geometry.polytope;

import us.ihmc.robotics.lists.RecyclingArrayList;

public class ExpandingPolytopeEdgeList
{
   private final RecyclingArrayList<ExpandingPolytopeEdge> edges = new RecyclingArrayList<>(ExpandingPolytopeEdge.class);

   public ExpandingPolytopeEdgeList()
   {
   }

   public void addEdge(ExpandingPolytopeEntry entry, int edgeIndex)
   {
      ExpandingPolytopeEdge edge = edges.add();
      edge.setEntryAndIndex(entry, edgeIndex);
   }

   public void clear()
   {
      edges.clear();
   }

   public int getNumberOfEdges()
   {
      return edges.size();
   }

   public ExpandingPolytopeEdge getEdge(int index)
   {
      return edges.get(index);
   }
}
