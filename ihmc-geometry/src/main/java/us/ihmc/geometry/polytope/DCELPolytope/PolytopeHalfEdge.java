package us.ihmc.geometry.polytope.DCELPolytope;

import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeHalfEdgeBuilder;
import us.ihmc.geometry.polytope.DCELPolytope.Providers.PolytopeHalfEdgeProvider;

/**
 * This class implements a doubly connected edge list (https://en.wikipedia.org/wiki/Doubly_connected_edge_list)
 * for storing polytope information
 * A half edge is completely described by its origin, destination and twin edge
 * The face, previous half edge and next half edge are stored for readability of code and should not be used for any geometrical operations
 * An attempt is made to update the twin in case the edge is modified to ensure that the relation remains consistent
 * @author Apoorv S
 */
public class PolytopeHalfEdge extends PolytopeHalfEdgeBasics<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace> implements Simplex
{
   private final PolytopeHalfEdgeBuilder halfEdgeBuilder = new PolytopeHalfEdgeBuilder();
   
   public PolytopeHalfEdge()
   {
      super();
   }
   
   /**
    * Creates a new edge at the same location. References to origin / destionation vertices, twin / next / previous edges and associated is not preserved
    * @param edge
    */
   public PolytopeHalfEdge(PolytopeHalfEdgeReadOnly edge)
   {
      super(new ExtendedPolytopeVertex(edge.getOriginVertex()), new ExtendedPolytopeVertex(edge.getOriginVertex()));
   }

   public PolytopeHalfEdge(ExtendedPolytopeVertex origin, ExtendedPolytopeVertex destination)
   {
      super(origin, destination);
   }
   
   public PolytopeHalfEdge(ExtendedPolytopeVertex originVertex, ExtendedPolytopeVertex destinationVertex, PolytopeHalfEdge twinEdge, PolytopeHalfEdge nextHalfEdge, PolytopeHalfEdge previousHalfEdge, ConvexPolytopeFace face)
   {
      super(originVertex, destinationVertex, twinEdge, nextHalfEdge, previousHalfEdge, face);
   }

   public PolytopeHalfEdge(PolytopeHalfEdge twinEdge, ConvexPolytopeFace face)
   {
      super(twinEdge, face);
   }
   
   @Override
   public PolytopeHalfEdge getNextHalfEdge()
   {
      return (PolytopeHalfEdge) super.getNextHalfEdge();
   }

   @Override
   protected PolytopeHalfEdgeProvider<ExtendedPolytopeVertex, PolytopeHalfEdge, ConvexPolytopeFace> getHalfEdgeProvider()
   {
      return halfEdgeBuilder;
   }
}
