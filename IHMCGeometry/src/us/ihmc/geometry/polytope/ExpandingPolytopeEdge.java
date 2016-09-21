package us.ihmc.geometry.polytope;

public class ExpandingPolytopeEdge
{
   private final ExpandingPolytopeEntry entry;
   private final int edgeIndex;

   public ExpandingPolytopeEdge(ExpandingPolytopeEntry entry, int edgeIndex)
   {
      this.entry = entry;
      this.edgeIndex = edgeIndex;
   }

   public ExpandingPolytopeEntry getEntry()
   {
      return entry;
   }

   public int getEdgeIndex()
   {
      return edgeIndex;
   }
}
