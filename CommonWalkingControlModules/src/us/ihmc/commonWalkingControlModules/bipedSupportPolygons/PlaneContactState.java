package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.List;

import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface PlaneContactState extends ContactState
{
   public abstract ReferenceFrame getPlaneFrame();
   public abstract List<FramePoint2d> getContactPoints2d();
}
