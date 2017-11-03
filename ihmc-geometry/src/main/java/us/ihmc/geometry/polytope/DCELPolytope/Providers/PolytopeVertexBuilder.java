package us.ihmc.geometry.polytope.DCELPolytope.Providers;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.ConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedPolytopeVertex;
import us.ihmc.geometry.polytope.DCELPolytope.PolytopeHalfEdge;
import us.ihmc.geometry.polytope.DCELPolytope.Simplex;

public class PolytopeVertexBuilder implements PolytopeVertexProvider
{

   @Override
   public ExtendedPolytopeVertex getVertex()
   {
      return new ExtendedPolytopeVertex();
   }

   @Override
   public ExtendedPolytopeVertex getVertex(double x, double y, double z)
   {
      return new ExtendedPolytopeVertex(x, y, z);
   }

   @Override
   public ExtendedPolytopeVertex getVertex(double[] coords)
   {
      return new ExtendedPolytopeVertex(coords[0], coords[1], coords[2]);
   }

   @Override
   public ExtendedPolytopeVertex getVertex(Point3DReadOnly vertexToAdd)
   {
      return new ExtendedPolytopeVertex(vertexToAdd);
   }
   
}
