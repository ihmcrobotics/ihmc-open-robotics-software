package us.ihmc.geometry.polytope.DCELPolytope;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeFaceBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeHalfEdgeBuilder;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeHalfEdgeProvider;

/**
 * This class defines a polytope face. A face is defined by the set of edges that bound it.
 * 
 * @author Apoorv S
 *
 */
public class ConvexPolytopeFace extends ConvexPolytopeFaceBasics<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace> implements Simplex
{
   private final PolytopeHalfEdgeBuilder halfEdgeBuilder = new PolytopeHalfEdgeBuilder();
   
   public ConvexPolytopeFace()
   {
      super();
   }
   
   public ConvexPolytopeFace(PolytopeHalfEdge[] edgeList)
   {
      this();
      copyEdgeList(edgeList);
   }

   @Override
   protected PolytopeHalfEdgeProvider<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace> getHalfEdgeProvider()
   {
      return halfEdgeBuilder;
   }
   
   @Override
   public PolytopeHalfEdge getEdge(int index)
   {
      return (PolytopeHalfEdge) super.getEdge(index);
   }
   
   @Override
   public PolytopeHalfEdge getFirstVisibleEdge(Point3DReadOnly vertex)
   {
      return (PolytopeHalfEdge) super.getFirstVisibleEdge(vertex);
   }
}
