package us.ihmc.geometry.polytope.DCELPolytope.Basics;

import java.util.List;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.geometry.polytope.SupportingVertexHolder;

public interface ConvexPolytopeReadOnly extends EpsilonComparable<ConvexPolytopeReadOnly>, SupportingVertexHolder 
{
   List<? extends ConvexPolytopeFaceReadOnly> getFaces();
   List<? extends PolytopeHalfEdgeReadOnly> getEdges();
   List<? extends PolytopeVertexReadOnly> getVertices();
   
   PolytopeVertexReadOnly getSupportingVertexHack(Vector3DReadOnly supportingVertexDirection);
   default boolean isEmpty()
   {
      List<? extends ConvexPolytopeFaceReadOnly> faces = getFaces();
      for (int i = 0; i < faces.size(); i++)
         if(faces.get(i).getNumberOfEdges() != 0)
            return false;
      return true;
   }
}
