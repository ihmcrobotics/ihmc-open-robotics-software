package us.ihmc.commonWalkingControlModules.calculators;

import us.ihmc.robotics.geometry.FramePoint2D;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SpatialForceVector;

public interface Omega0CalculatorInterface
{
   public abstract double computeOmega0(SideDependentList<FramePoint2D> cops, SpatialForceVector totalGroundReactionWrench);
}
