package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.LegStrengthCalculator;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;


public class TeeterTotterLegStrengthCalculator implements LegStrengthCalculator
{
   private final YoVariableRegistry registry = new YoVariableRegistry("LegStrengthCalculator");
   private final double epsilonBetweenEndPoints = 1e-3;
   private final double epsilonSum = 1e-2;
   private final SideDependentList<DoubleYoVariable> legStrengths = new SideDependentList<DoubleYoVariable>();
   
   public TeeterTotterLegStrengthCalculator(YoVariableRegistry parentRegistry)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         legStrengths.put(robotSide, new DoubleYoVariable(robotSide.getCamelCaseNameForStartOfExpression() + "LegStrength", registry));
      }
      parentRegistry.addChild(registry);
   }

   public void packLegStrengths(SideDependentList<Double> legStrengths, SideDependentList<FramePoint2d> virtualToePoints, FramePoint2d coPDesired)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         virtualToePoints.get(robotSide).changeFrame(coPDesired.getReferenceFrame());
      }

      FrameLineSegment2d vtpToVtpSegment = new FrameLineSegment2d(virtualToePoints.get(RobotSide.LEFT), virtualToePoints.get(RobotSide.RIGHT));
      if (vtpToVtpSegment.isBetweenEndpoints(coPDesired, epsilonBetweenEndPoints))
      {
         double vtpToVtp = vtpToVtpSegment.length();
         for (RobotSide robotSide : RobotSide.values)
         {
            double otherVtpToCoP = virtualToePoints.get(robotSide.getOppositeSide()).distance(coPDesired);
            double legStrength = otherVtpToCoP / vtpToVtp;
            legStrengths.put(robotSide, legStrength);
         }
      }
      else
      {
         RobotSide sideThatCoPIsClosestTo = determineSideThatCoPIsClosestTo(virtualToePoints, coPDesired);
         legStrengths.put(sideThatCoPIsClosestTo, 1.0);
         legStrengths.put(sideThatCoPIsClosestTo.getOppositeSide(), 0.0);
      }

      double sum = MathTools.sumDoubles(legStrengths.values());
      if (Math.abs(sum - 1.0) > epsilonSum)
      {
         throw new RuntimeException("leftStrength + rightStrength != 1.0. legStrengths =\n" + legStrengths + "\n" + "sum =  " + sum);
      }

      LegStrengthCalculatorTools.clip(legStrengths);
      LegStrengthCalculatorTools.normalize(legStrengths);
      for (RobotSide robotSide : RobotSide.values)
      {
         this.legStrengths.get(robotSide).set(legStrengths.get(robotSide));
      }
   }

   private RobotSide determineSideThatCoPIsClosestTo(SideDependentList<FramePoint2d> virtualToePoints, FramePoint2d coPDesired)
   {
      RobotSide ret = null;
      double minDistanceSquared = Double.POSITIVE_INFINITY;
      for (RobotSide robotSide : RobotSide.values)
      {
         double distanceSquared = virtualToePoints.get(robotSide).distanceSquared(coPDesired);
         if (distanceSquared < minDistanceSquared)
         {
            ret = robotSide;
            minDistanceSquared = distanceSquared;
         }
      }

      return ret;
   }
}
