package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.List;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface PlaneContactState
{
   public static final double DEFAULT_WRHO = 0.001;
   
   public abstract ReferenceFrame getBodyFrame();
   public abstract ReferenceFrame getPlaneFrame();
   public abstract boolean inContact();
   public abstract FrameVector getContactNormalFrameVector();
   public abstract List<FramePoint> getContactFramePoints();
   public abstract List<FramePoint2d> getContactFramePoints2d();
   public abstract double getCoefficientOfFriction();
   public abstract int getNumberOfContactPoints();
   
   public abstract List<? extends ContactPoint> getContactPoints();

   // TODO: Probably get rid of that. Now, it is used for smooth unload/load transitions in the CarIngressEgressController.
   public void setRhoContactRegularization(double wRho);
   public double getRhoContactRegularization();
   public void resetContactRegularization();
}
