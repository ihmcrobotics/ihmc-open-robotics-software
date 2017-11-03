package us.ihmc.geometry.polytope.DCELPolytope;

import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeVertexReadOnly;

public class SimplexVertex extends ExtendedPolytopeVertex
{
   PolytopeVertexReadOnly polytopeAVertexReference;
   PolytopeVertexReadOnly polytopeBVertexReference;
   
   public SimplexVertex()
   {
      super();
   }
   
   public SimplexVertex(ExtendedPolytopeVertex vertexOnPolytopeA, ExtendedPolytopeVertex vertexOnPolytopeB)
   {
      set(vertexOnPolytopeA, vertexOnPolytopeB);
   }
   
   public void set(PolytopeVertexReadOnly vertexOnPolytopeA, PolytopeVertexReadOnly vertexOnPolytopeB)
   {
      this.polytopeAVertexReference = vertexOnPolytopeA;
      this.polytopeBVertexReference = vertexOnPolytopeB;
      sub(vertexOnPolytopeA, vertexOnPolytopeB);
   }
   
   public PolytopeVertexReadOnly getVertexOnPolytopeA()
   {
      return polytopeAVertexReference;
   }
   
   public PolytopeVertexReadOnly getVertexOnPolytopeB()
   {
      return polytopeBVertexReference;
   }
}
