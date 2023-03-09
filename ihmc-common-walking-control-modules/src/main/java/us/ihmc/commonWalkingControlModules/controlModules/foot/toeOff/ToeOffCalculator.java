package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;

public interface ToeOffCalculator extends SCS2YoGraphicHolder
{
   void clear();

   ToeOffEnum getEnum();

   void setExitCMP(FramePoint3DReadOnly exitCMP, RobotSide trailingLeg);

   void computeToeOffContactPoint(FramePoint2DReadOnly desiredCMP, RobotSide trailingLeg);

   void getToeOffContactPoint(FramePoint2DBasics contactPointToPack, RobotSide trailingLeg);

   void computeToeOffContactLine(FramePoint2DReadOnly desiredCMP, RobotSide trailingLeg);

   void getToeOffContactLine(FrameLineSegment2DBasics contactLineToPack, RobotSide trailingLeg);

   @Override
   default YoGraphicDefinition getSCS2YoGraphics()
   {
      return null;
   }
}
