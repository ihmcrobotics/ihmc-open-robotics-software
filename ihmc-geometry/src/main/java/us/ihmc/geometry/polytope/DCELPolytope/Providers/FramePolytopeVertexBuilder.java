package us.ihmc.geometry.polytope.DCELPolytope.Providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.ConvexPolytopeBasics;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FrameConvexPolytopeFace;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FramePolytopeHalfEdge;
import us.ihmc.geometry.polytope.DCELPolytope.Frame.FramePolytopeVertex;

public class FramePolytopeVertexBuilder implements PolytopeVertexProvider<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace>
{
   private final ReferenceFrameHolder referenceFrameHolder;
   
   public FramePolytopeVertexBuilder(ReferenceFrameHolder referenceFrameHolder)
   {
      this.referenceFrameHolder = referenceFrameHolder;
   }

   @Override
   public FramePolytopeVertex getVertex()
   {
      return new FramePolytopeVertex(referenceFrameHolder.getReferenceFrame());
   }

   @Override
   public FramePolytopeVertex getVertex(double x, double y, double z)
   {
      return new FramePolytopeVertex(referenceFrameHolder.getReferenceFrame(), x, y, z);
   }

   @Override
   public FramePolytopeVertex getVertex(double[] coords)
   {
      return new FramePolytopeVertex(referenceFrameHolder.getReferenceFrame(), coords[0], coords[0], coords[2]);
   }

   @Override
   public FramePolytopeVertex getVertex(Point3DReadOnly vertexToAdd)
   {
      return new FramePolytopeVertex(referenceFrameHolder.getReferenceFrame(), vertexToAdd);
   }
   
   public FramePolytopeVertex getVertex(FramePoint3D vertexToAdd)
   {
      return new FramePolytopeVertex(vertexToAdd.getReferenceFrame(), vertexToAdd.getPoint());
   }
}
