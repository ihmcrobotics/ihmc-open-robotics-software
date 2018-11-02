package us.ihmc.commonWalkingControlModules.calculators;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface Omega0CalculatorInterface
{
   public abstract double computeOmega0(SideDependentList<FramePoint2D> cops, SpatialForce totalGroundReactionWrench);
}
