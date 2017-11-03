package us.ihmc.geometry.polytope.DCELPolytope.Frame;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.geometry.polytope.SupportingVertexHolder;

public interface FrameSupportingVertexHolder extends SupportingVertexHolder, ReferenceFrameHolder
{
   default void getSupportingVertex(FrameVector3D supportDirection, FramePoint3D supportingVertexToPack)
   {
      checkReferenceFrameMatch(supportDirection);
      supportingVertexToPack.setIncludingFrame(getReferenceFrame(), getSupportingVertex(supportDirection.getVector()));
   }
}
