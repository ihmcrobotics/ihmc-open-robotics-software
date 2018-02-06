package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator;

import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class SimpleToeOffCalculator implements ToeOffCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private static final String namePrefix = "simple";

   private final SideDependentList<ContactableFoot> feet;

   private final FramePoint2D toeOffContactPoint2d = new FramePoint2D();
   private final FrameLineSegment2D toeOffContactLine2d = new FrameLineSegment2D();

   private final YoBoolean hasComputedToeOffContactPoint;
   private final YoBoolean hasComputedToeOffContactLine;

   public SimpleToeOffCalculator(SideDependentList<ContactableFoot> feet, YoVariableRegistry parentRegistry)
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
   public void setExitCMP(FramePoint3D exitCMP, RobotSide trailingLeg)
   {
   }

   @Override
   public void computeToeOffContactPoint(FramePoint2D desiredCMP, RobotSide trailingLeg)
   {
      feet.get(trailingLeg).getToeOffContactPoint(toeOffContactPoint2d);
      hasComputedToeOffContactPoint.set(true);
   }

   @Override
   public void getToeOffContactPoint(FramePoint2D contactPointToPack, RobotSide trailingLeg)
   {
      if (!hasComputedToeOffContactPoint.getBooleanValue())
         computeToeOffContactPoint(null, trailingLeg);

      contactPointToPack.set(toeOffContactPoint2d);
   }

   @Override
   public void computeToeOffContactLine(FramePoint2D desiredCMP, RobotSide trailingLeg)
   {
      feet.get(trailingLeg).getToeOffContactLine(toeOffContactLine2d);
      hasComputedToeOffContactLine.set(true);
   }

   @Override
   public void getToeOffContactLine(FrameLineSegment2D contactLineToPack, RobotSide trailingLeg)
   {
      if (!hasComputedToeOffContactLine.getBooleanValue())
         computeToeOffContactLine(null, trailingLeg);

      contactLineToPack.set(toeOffContactLine2d);
   }
}
