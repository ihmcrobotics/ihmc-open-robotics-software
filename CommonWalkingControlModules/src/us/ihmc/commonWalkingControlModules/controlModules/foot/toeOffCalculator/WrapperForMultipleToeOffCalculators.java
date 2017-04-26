package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;

public class WrapperForMultipleToeOffCalculators implements ToeOffCalculator
{
   private final String namePostfix = getClass().getSimpleName();

   private final EnumYoVariable<ToeOffEnum> activeToeOffCalculator;
   private final EnumMap<ToeOffEnum, ToeOffCalculator> toeOffCalculators;

   public WrapperForMultipleToeOffCalculators(EnumMap<ToeOffEnum, ToeOffCalculator> toeOffCalculators, YoVariableRegistry registry)
   {
      this.toeOffCalculators = toeOffCalculators;

      activeToeOffCalculator = EnumYoVariable.create("ActiveToeOffCalculator", ToeOffEnum.class, registry);
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
   public void setExitCMP(FramePoint exitCMP, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(activeToeOffCalculator.getEnumValue());
      currentCalculator.setExitCMP(exitCMP, trailingLeg);
   }

   @Override
   public void computeToeOffContactPoint(FramePoint2d desiredCMP, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(activeToeOffCalculator.getEnumValue());
      currentCalculator.computeToeOffContactPoint(desiredCMP, trailingLeg);
   }

   @Override
   public void getToeOffContactPoint(FramePoint2d contactPointToPack, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(activeToeOffCalculator.getEnumValue());
      currentCalculator.getToeOffContactPoint(contactPointToPack, trailingLeg);
   }

   @Override
   public void computeToeOffContactLine(FramePoint2d desiredCMP, RobotSide trailingLeg)
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
