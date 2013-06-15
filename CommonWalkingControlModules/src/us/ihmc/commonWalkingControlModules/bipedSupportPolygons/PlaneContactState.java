package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.List;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface PlaneContactState
{
   public static final double DEFAULT_WRHO = 0.002;
   
   public abstract List<FramePoint> getContactPoints();
   public abstract ReferenceFrame getBodyFrame();
   public abstract boolean inContact();
   public abstract ReferenceFrame getPlaneFrame();
   public abstract List<FramePoint2d> getContactPoints2d();
   public abstract FrameVector getContactNormalFrameVector();
   public abstract double getCoefficientOfFriction();
   public abstract int getNumberOfContactPoints();

   // TODO: Probably get rid of that. Now, it is used for smooth unload/load transitions in the CarIngressEgressController.
   public void setRhoContactRegularization(double wRho);
   public double getRhoContactRegularization();
   public void resetContactRegularization();
}
