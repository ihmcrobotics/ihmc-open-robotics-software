package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

public interface ToeOffCalculator
{
   void clear();

   ToeOffEnum getEnum();

   void setExitCMP(FramePoint3DReadOnly exitCMP, RobotSide trailingLeg);

   void computeToeOffContactPoint(FramePoint2DReadOnly desiredCMP, RobotSide trailingLeg);

   void getToeOffContactPoint(FramePoint2DBasics contactPointToPack, RobotSide trailingLeg);

   void computeToeOffContactLine(FramePoint2DReadOnly desiredCMP, RobotSide trailingLeg);

   void getToeOffContactLine(FrameLineSegment2DBasics contactLineToPack, RobotSide trailingLeg);
}
