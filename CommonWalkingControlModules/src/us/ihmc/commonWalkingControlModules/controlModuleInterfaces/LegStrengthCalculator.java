package us.ihmc.commonWalkingControlModules.controlModuleInterfaces;

import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.robotSide.SideDependentList;

public interface LegStrengthCalculator
{
   public abstract void getLegStrengths(SideDependentList<Double> legStrengthsToPack, SideDependentList<FramePoint2d> virtualToePoints, FramePoint2d coPDesired);
}
