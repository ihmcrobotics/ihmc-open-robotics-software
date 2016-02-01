package us.ihmc.commonWalkingControlModules.calculators;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SpatialForceVector;

public interface Omega0CalculatorInterface
{
   public abstract double computeOmega0(SideDependentList<FramePoint2d> cops, SpatialForceVector totalGroundReactionWrench);
}
