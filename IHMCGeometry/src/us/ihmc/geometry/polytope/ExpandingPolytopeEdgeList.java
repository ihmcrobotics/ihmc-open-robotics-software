package us.ihmc.geometry.polytope;

import java.util.ArrayList;

public class ExpandingPolytopeEdgeList
{
   private final ArrayList<ExpandingPolytopeEdge> edges = new ArrayList<>();

   public ExpandingPolytopeEdgeList()
   {

   }

   public void addEdge(ExpandingPolytopeEntry entry, int edgeIndex)
   {
      ExpandingPolytopeEdge edge = new ExpandingPolytopeEdge(entry, edgeIndex);
      edges.add(edge);
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
