package us.ihmc.commonWalkingControlModules.calculators;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SpatialForce;

public interface Omega0CalculatorInterface
{
   public abstract double computeOmega0(SideDependentList<FramePoint2D> cops, SpatialForce totalGroundReactionWrench);
}
