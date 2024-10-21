package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOff;

import java.util.EnumMap;

import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

public class WrapperForMultipleToeOffCalculators implements ToeOffCalculator
{
   private final String namePostfix = getClass().getSimpleName();

   private final YoEnum<ToeOffEnum> activeToeOffCalculator;
   private final EnumMap<ToeOffEnum, ToeOffCalculator> toeOffCalculators;

   public WrapperForMultipleToeOffCalculators(EnumMap<ToeOffEnum, ToeOffCalculator> toeOffCalculators, YoRegistry registry)
   {
      this.toeOffCalculators = toeOffCalculators;

      activeToeOffCalculator = new YoEnum<>("ActiveToeOffCalculator", registry, ToeOffEnum.class);
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
   public void setExitCMP(FramePoint3DReadOnly exitCMP, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(activeToeOffCalculator.getEnumValue());
      currentCalculator.setExitCMP(exitCMP, trailingLeg);
   }

   @Override
   public void computeToeOffContactPoint(FramePoint2DReadOnly desiredCMP, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(activeToeOffCalculator.getEnumValue());
      currentCalculator.computeToeOffContactPoint(desiredCMP, trailingLeg);
   }

   @Override
   public void getToeOffContactPoint(FramePoint2DBasics contactPointToPack, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(activeToeOffCalculator.getEnumValue());
      currentCalculator.getToeOffContactPoint(contactPointToPack, trailingLeg);
   }

   @Override
   public void computeToeOffContactLine(FramePoint2DReadOnly desiredCMP, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(activeToeOffCalculator.getEnumValue());
      currentCalculator.computeToeOffContactLine(desiredCMP, trailingLeg);
   }

   @Override
   public void getToeOffContactLine(FrameLineSegment2DBasics contactLineToPack, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(activeToeOffCalculator.getEnumValue());
      currentCalculator.getToeOffContactLine(contactLineToPack, trailingLeg);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      for (ToeOffCalculator calculator : toeOffCalculators.values())
         group.addChild(calculator.getSCS2YoGraphics());
      return group;
   }
}
