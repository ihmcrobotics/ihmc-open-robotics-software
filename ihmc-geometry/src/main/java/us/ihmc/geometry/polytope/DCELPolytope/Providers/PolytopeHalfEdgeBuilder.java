package us.ihmc.geometry.polytope.DCELPolytope.Providers;

import us.ihmc.geometry.polytope.DCELPolytope.ConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedPolytopeVertex;
import us.ihmc.geometry.polytope.DCELPolytope.PolytopeHalfEdge;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeVertexBasics;

public class PolytopeHalfEdgeBuilder implements PolytopeHalfEdgeProvider<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace>
{

   @Override
   public PolytopeHalfEdge getHalfEdge(ExtendedPolytopeVertex origin, ExtendedPolytopeVertex destination)
   {
      return new PolytopeHalfEdge((ExtendedPolytopeVertex) origin, (ExtendedPolytopeVertex) destination);
   }

   @Override
   public PolytopeHalfEdge getHalfEdge()
   {
      return new PolytopeHalfEdge();
   }

   @Override
   public PolytopeHalfEdge getHalfEdge(PolytopeHalfEdgeReadOnly polytopeHalfEdge)
   {
      return new PolytopeHalfEdge(polytopeHalfEdge);
   }
}
