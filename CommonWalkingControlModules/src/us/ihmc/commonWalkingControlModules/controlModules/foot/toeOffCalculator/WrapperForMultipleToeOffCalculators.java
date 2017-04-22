package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;

public class WrapperForMultipleToeOffCalculators implements ToeOffCalculator
{
   private final String namePostfix = getClass().getSimpleName();

   private final IntegerYoVariable toeOffCalculatorIndex;
   private final ArrayList<ToeOffCalculator> toeOffCalculators;

   public WrapperForMultipleToeOffCalculators(ArrayList<ToeOffCalculator> toeOffCalculators, YoVariableRegistry registry)
   {
      this.toeOffCalculators = toeOffCalculators;

      toeOffCalculatorIndex = new IntegerYoVariable("ToeOffCalculatorIndex", registry);
   }

   public void clear()
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(toeOffCalculatorIndex.getIntegerValue());
      currentCalculator.clear();
   }

   public void setExitCMP(FramePoint exitCMP, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(toeOffCalculatorIndex.getIntegerValue());
      currentCalculator.setExitCMP(exitCMP, trailingLeg);
   }

   public void computeToeOffContactPoint(FramePoint2d desiredCMP, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(toeOffCalculatorIndex.getIntegerValue());
      currentCalculator.computeToeOffContactPoint(desiredCMP, trailingLeg);
   }

   public void getToeOffContactPoint(FramePoint2d contactPointToPack, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(toeOffCalculatorIndex.getIntegerValue());
      currentCalculator.getToeOffContactPoint(contactPointToPack, trailingLeg);
   }

   public void computeToeOffContactLine(FramePoint2d desiredCMP, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(toeOffCalculatorIndex.getIntegerValue());
      currentCalculator.computeToeOffContactLine(desiredCMP, trailingLeg);
   }

   public void getToeOffContactLine(FrameLineSegment2d contactLineToPack, RobotSide trailingLeg)
   {
      ToeOffCalculator currentCalculator = toeOffCalculators.get(toeOffCalculatorIndex.getIntegerValue());
      currentCalculator.getToeOffContactLine(contactLineToPack, trailingLeg);
   }
}
