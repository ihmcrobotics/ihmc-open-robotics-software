package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.EnumMap;

public class WrapperForMultipleToeOffCalculators implements ToeOffCalculator
{
   private final String namePostfix = getClass().getSimpleName();

   private final YoEnum<ToeOffEnum> activeToeOffCalculator;
   private final EnumMap<ToeOffEnum, ToeOffCalculator> toeOffCalculators;

   public WrapperForMultipleToeOffCalculators(EnumMap<ToeOffEnum, ToeOffCalculator> toeOffCalculators, YoVariableRegistry registry)
   {
      this.toeOffCalculators = toeOffCalculators;

      activeToeOffCalculator = YoEnum.create("ActiveToeOffCalculator", ToeOffEnum.class, registry);
      activeToeOffCalculator.set(ToeOffEnum.CENTROID_PROJECTION);
   }

   @Override
   public ToeOffEnum getEnum()
   {
      return activeToeOffCalculator.getEnumValue();
   }

   @Override
   public void clear()
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(activeToeOffCalculator.getEnumValue());
      currentCalculator.clear();
   }

   @Override
   public void setExitCMP(FramePoint3D exitCMP, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(activeToeOffCalculator.getEnumValue());
      currentCalculator.setExitCMP(exitCMP, trailingLeg);
   }

   @Override
   public void computeToeOffContactPoint(FramePoint2D desiredCMP, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(activeToeOffCalculator.getEnumValue());
      currentCalculator.computeToeOffContactPoint(desiredCMP, trailingLeg);
   }

   @Override
   public void getToeOffContactPoint(FramePoint2D contactPointToPack, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(activeToeOffCalculator.getEnumValue());
      currentCalculator.getToeOffContactPoint(contactPointToPack, trailingLeg);
   }

   @Override
   public void computeToeOffContactLine(FramePoint2D desiredCMP, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(activeToeOffCalculator.getEnumValue());
      currentCalculator.computeToeOffContactLine(desiredCMP, trailingLeg);
   }

   @Override
   public void getToeOffContactLine(FrameLineSegment2d contactLineToPack, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(activeToeOffCalculator.getEnumValue());
      currentCalculator.getToeOffContactLine(contactLineToPack, trailingLeg);
   }
}
