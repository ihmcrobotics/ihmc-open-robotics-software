package us.ihmc.geometry.polytope.DCELPolytope.Providers;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeFaceBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeVertexBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.SimplexBasics;

public interface PolytopeVertexProvider<A extends PolytopeVertexBasics<A, B, C>, B extends PolytopeHalfEdgeBasics<A, B, C>, C extends ConvexPolytopeFaceBasics<A, B, C>> 
{
   A getVertex();
   A getVertex(double x, double y, double z);
   A getVertex(double coords[]);
   A getVertex(Point3DReadOnly vertexToAdd);
}
