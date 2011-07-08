package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.MathTools;

public class LegStrengthCalculatorTools
{
   /**
    * Clip leg strengths to [0, 1]
    * @param legStrengths
    */
   public static void clip(SideDependentList<Double> legStrengths)
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         legStrengths.put(robotSide, MathTools.clipToMinMax(legStrengths.get(robotSide), 0.0, 1.0));
      }
   }

   /**
    *  Normalize leg strengths so that the sum is 1
    * @param legStrengths
    */
   public static void normalize(SideDependentList<Double> legStrengths)
   {
      double sum = MathTools.sumDoubles(legStrengths.values());
      double scaling = 1.0 / sum;
      for (RobotSide robotSide : RobotSide.values())
      {
         legStrengths.put(robotSide, legStrengths.get(robotSide) * scaling);
      }
   }
}
