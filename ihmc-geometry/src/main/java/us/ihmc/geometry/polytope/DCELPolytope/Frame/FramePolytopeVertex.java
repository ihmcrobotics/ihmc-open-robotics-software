package us.ihmc.geometry.polytope.DCELPolytope.Frame;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.PolytopeVertexBasics;

public class FramePolytopeVertex extends PolytopeVertexBasics<FramePolytopeVertex, FramePolytopeHalfEdge, FrameConvexPolytopeFace>  implements FrameSimplex, ReferenceFrameHolder
{
   private FramePoint3D point = new FramePoint3D();
   private FramePoint3D pointToReturn = new FramePoint3D();
   
   public FramePolytopeVertex()
   {
   }
   
   public FramePolytopeVertex(ReferenceFrame frame)
   {
      this.point.setToZero(frame);
   }

   public FramePolytopeVertex(ReferenceFrame frame, double x, double y, double z)
   {
      this.point.setIncludingFrame(frame, x, y, z);
   }

   public FramePolytopeVertex(ReferenceFrame frame, Point3DReadOnly vertex)
   {
      this.point.setIncludingFrame(frame, vertex);
   }
   
   public FramePolytopeVertex(FramePoint3D vertex)
   {
      this.point.setIncludingFrame(vertex);
   }
   
   public ReferenceFrame getReferenceFrame()
   {
      return point.getReferenceFrame();
   }
   
   public void changeFrame(ReferenceFrame referenceFrame)
   {
      point.changeFrame(referenceFrame);
   }
   
   @Override
   public FramePoint3D getPosition()
   {
      pointToReturn.setIncludingFrame(point);
      pointToReturn.changeFrame(ReferenceFrame.getWorldFrame());
      return pointToReturn;
   }

   @Override
   protected Point3DBasics getPointObjectReference()
   {
      getPosition();
      pointToReturn.changeFrame(ReferenceFrame.getWorldFrame());
      return pointToReturn;
   }
}
