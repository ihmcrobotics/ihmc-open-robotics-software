package us.ihmc.geometry.polytope.DCELPolytope;

import java.util.List;

import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.ConvexPolytopeFaceBuilder;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.ConvexPolytopeFaceProvider;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeVertexBuilder;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeVertexProvider;

/**
 * A convex polytope is a collection of faces that describe it 
 * 
 * This class is a data structure for storing a polytope in the DCEL notation (ref: https://en.wikipedia.org/wiki/Doubly_connected_edge_list).
 * Based on the original implementation by Jerry Pratt
 * @author Apoorv S
 */

public class ExtendedConvexPolytope extends ConvexPolytopeBasics<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace>
{
   private final ConvexPolytopeFaceBuilder faceBuilder = new ConvexPolytopeFaceBuilder(); 
   private final PolytopeVertexBuilder vertexBuilder =  new PolytopeVertexBuilder();
   public ExtendedConvexPolytope()
   {
      super();
   }
   
   public ExtendedConvexPolytope(ExtendedConvexPolytope polytope)
   {
      super(polytope);
   }

   @Override
   protected PolytopeVertexProvider<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace> getVertexProvider()
   {
      return vertexBuilder;
   }

   @Override
   protected ConvexPolytopeFaceProvider<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace> getConvexFaceProvider()
   {
      return faceBuilder;
   }
   
   @Override
   public ConvexPolytopeFace getFace(int index)
   {
      return (ConvexPolytopeFace) super.getFace(index);
   }
   
   @Override
   public List<ExtendedPolytopeVertex> getVertices()
   {
      return (List<ExtendedPolytopeVertex>) super.getVertices();
   }
   
   @Override
   public List<PolytopeHalfEdge> getEdges()
   {
      return (List<PolytopeHalfEdge>) super.getEdges();
   }
}
