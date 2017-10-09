package us.ihmc.geometry.polytope;

import us.ihmc.euclid.tuple3D.Point3D;

public class ExpandingPolytopeEdge
{
   private ExpandingPolytopeEntry entry;
   private int edgeIndex;

   public ExpandingPolytopeEdge()
   {
   }

   public ExpandingPolytopeEdge(ExpandingPolytopeEntry entry, int edgeIndex)
   {
      setEntryAndIndex(entry, edgeIndex);
   }
   
   public void setEntryAndIndex(ExpandingPolytopeEntry entry, int edgeIndex)
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
   
   public Point3D getStartPoint()
   {
      return entry.getVertex(edgeIndex);
   }
   
   public Point3D getEndPoint()
   {
      return entry.getVertex((edgeIndex + 1) %3);
   }
}
