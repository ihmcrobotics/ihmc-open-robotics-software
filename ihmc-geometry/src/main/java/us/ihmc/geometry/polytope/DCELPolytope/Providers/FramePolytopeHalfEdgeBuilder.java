package us.ihmc.geometry.polytope.DCELPolytope.Providers;

import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeHalfEdgeReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeVertexBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FramePolytopeHalfEdge;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FramePolytopeVertex;

public class FramePolytopeHalfEdgeBuilder implements PolytopeHalfEdgeProvider<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace>
{
   private final ReferenceFrameHolder referenceFrameHolder;
   
   public FramePolytopeHalfEdgeBuilder(ReferenceFrameHolder referenceFrameHolder)
   {
      this.referenceFrameHolder = referenceFrameHolder;
   }
   
   @Override
   public FramePolytopeHalfEdge getHalfEdge(FramePolytopeVertex origin, FramePolytopeVertex destination)
   {
      return new FramePolytopeHalfEdge(referenceFrameHolder.getReferenceFrame(), (FramePolytopeVertex) origin, (FramePolytopeVertex) destination);
   }

   @Override
   public FramePolytopeHalfEdge getHalfEdge()
   {
      return new FramePolytopeHalfEdge(referenceFrameHolder.getReferenceFrame());
   }

   @Override
   public FramePolytopeHalfEdge getHalfEdge(PolytopeHalfEdgeReadOnly polytopeHalfEdge)
   {
      return new FramePolytopeHalfEdge(referenceFrameHolder.getReferenceFrame(), polytopeHalfEdge);
   }

}
