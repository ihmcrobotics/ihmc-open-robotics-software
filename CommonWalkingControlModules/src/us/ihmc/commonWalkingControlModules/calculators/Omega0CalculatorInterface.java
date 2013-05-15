package us.ihmc.commonWalkingControlModules.calculators;

import java.util.List;

import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public interface Omega0CalculatorInterface
{
   public abstract double computeOmega0(List<FramePoint2d> cop2ds, SpatialForceVector totalGroundReactionWrench);
}
