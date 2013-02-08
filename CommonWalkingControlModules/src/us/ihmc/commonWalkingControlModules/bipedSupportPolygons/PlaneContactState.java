package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.List;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface PlaneContactState
{
   public abstract List<FramePoint> getContactPoints();
   public abstract ReferenceFrame getBodyFrame();
   public abstract boolean inContact();
   public abstract ReferenceFrame getPlaneFrame();
   public abstract List<FramePoint2d> getContactPoints2d();
   public abstract double getCoefficientOfFriction();
}
