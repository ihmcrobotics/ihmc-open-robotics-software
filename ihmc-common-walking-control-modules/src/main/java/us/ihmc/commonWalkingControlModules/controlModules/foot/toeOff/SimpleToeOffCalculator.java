package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.robotics.robotSide.SideDependentList;

public class SimpleToeOffCalculator implements ToeOffCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private static final String namePrefix = "simple";

   private final SideDependentList<ContactableFoot> feet;

   private final FramePoint2D toeOffContactPoint2d = new FramePoint2D();
   private final FrameLineSegment2D toeOffContactLine2d = new FrameLineSegment2D();

   private final YoBoolean hasComputedToeOffContactPoint;
   private final YoBoolean hasComputedToeOffContactLine;

   public SimpleToeOffCalculator(SideDependentList<ContactableFoot> feet, YoRegistry parentRegistry)
   {
      this.feet = feet;

      hasComputedToeOffContactPoint = new YoBoolean(namePrefix + "HasComputedToeOffContactPoint", registry);
      hasComputedToeOffContactLine = new YoBoolean(namePrefix + "HasComputedToeOffContactLine", registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public ToeOffEnum getEnum()
   {
      return ToeOffEnum.SIMPLE;
   }

   @Override
   public void clear()
   {
      hasComputedToeOffContactPoint.set(false);
      hasComputedToeOffContactLine.set(false);
   }

   @Override
   public void setExitCMP(FramePoint3DReadOnly exitCMP, RobotSide trailingLeg)
   {
   }

   @Override
   public void computeToeOffContactPoint(FramePoint2DReadOnly desiredCMP, RobotSide trailingLeg)
   {
      feet.get(trailingLeg).getToeOffContactPoint(toeOffContactPoint2d);
      hasComputedToeOffContactPoint.set(true);
   }

   @Override
   public void getToeOffContactPoint(FramePoint2DBasics contactPointToPack, RobotSide trailingLeg)
   {
      if (!hasComputedToeOffContactPoint.getBooleanValue())
         computeToeOffContactPoint(null, trailingLeg);

      contactPointToPack.set(toeOffContactPoint2d);
   }

   @Override
   public void computeToeOffContactLine(FramePoint2DReadOnly desiredCMP, RobotSide trailingLeg)
   {
      feet.get(trailingLeg).getToeOffContactLine(toeOffContactLine2d);
      hasComputedToeOffContactLine.set(true);
   }

   @Override
   public void getToeOffContactLine(FrameLineSegment2DBasics contactLineToPack, RobotSide trailingLeg)
   {
      if (!hasComputedToeOffContactLine.getBooleanValue())
         computeToeOffContactLine(null, trailingLeg);

      contactLineToPack.set(toeOffContactLine2d);
   }
}
