package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ToeOffCalculator
{
   public void clear();

   public ToeOffEnum getEnum();

   public void setExitCMP(FramePoint3D exitCMP, RobotSide trailingLeg);

   public void computeToeOffContactPoint(FramePoint2D desiredCMP, RobotSide trailingLeg);

   public void getToeOffContactPoint(FramePoint2D contactPointToPack, RobotSide trailingLeg);

   public void computeToeOffContactLine(FramePoint2D desiredCMP, RobotSide trailingLeg);

   public void getToeOffContactLine(FrameLineSegment2d contactLineToPack, RobotSide trailingLeg);
}
