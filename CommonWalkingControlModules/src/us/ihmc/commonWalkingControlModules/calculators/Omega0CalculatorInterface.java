package us.ihmc.commonWalkingControlModules.calculators;

import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public interface Omega0CalculatorInterface
{
   public abstract double computeOmega0(SideDependentList<FramePoint2d> cops, SpatialForceVector totalGroundReactionWrench);
}
