package us.ihmc.geometry.polytope.DCELPolytope.Frame;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.geometry.polytope.DCELPolytope.Basics.SimplexBasics;

public interface FrameSimplex extends SimplexBasics, ReferenceFrameHolder
{
   default FrameSimplex getSmallestSimplexMemberReference(FramePoint3D point)
   {
      checkReferenceFrameMatch(point);
      return (FrameSimplex) getSmallestSimplexMemberReference(point.getPoint());
   }
}
