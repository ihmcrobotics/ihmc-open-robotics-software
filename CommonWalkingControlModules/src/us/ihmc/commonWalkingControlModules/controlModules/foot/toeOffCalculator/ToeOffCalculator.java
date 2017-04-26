package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator;

import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ToeOffCalculator
{
   public void clear();

   public ToeOffEnum getEnum();

   public void setExitCMP(FramePoint exitCMP, RobotSide trailingLeg);

   public void computeToeOffContactPoint(FramePoint2d desiredCMP, RobotSide trailingLeg);

   public void getToeOffContactPoint(FramePoint2d contactPointToPack, RobotSide trailingLeg);

   public void computeToeOffContactLine(FramePoint2d desiredCMP, RobotSide trailingLeg);

   public void getToeOffContactLine(FrameLineSegment2d contactLineToPack, RobotSide trailingLeg);
}
