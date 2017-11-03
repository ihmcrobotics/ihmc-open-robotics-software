package us.ihmc.geometry.polytope.DCELPolytope.Providers;

import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeFaceBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeVertexBasics;

public interface PolytopeHalfEdgeProvider<A extends PolytopeVertexBasics<A, B, C>, B extends PolytopeHalfEdgeBasics<A, B, C>, C extends ConvexPolytopeFaceBasics<A, B, C>>
{
   B getHalfEdge(A origin, A destination);
   B getHalfEdge();
   B getHalfEdge(PolytopeHalfEdgeReadOnly polytopeHalfEdge);
}
