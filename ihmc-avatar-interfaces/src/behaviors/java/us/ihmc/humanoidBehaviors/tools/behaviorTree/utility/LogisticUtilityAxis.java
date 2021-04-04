package us.ihmc.humanoidBehaviors.tools.behaviorTree.utility;

import java.util.function.DoubleSupplier;

public class LogisticUtilityAxis extends UtilityAxis
{
   /**
    * @param m slope of the line at inflection point
    * @param k vertical size of the curve
    * @param b y-intercept(vertical shift)
    * @param c x-intercept of the inflection point (horizontal shift)
    */
   public LogisticUtilityAxis(double m, double k, double b, double c, DoubleSupplier xSupplier)
   {
      super(m, k, b, c, xSupplier);
   }

   @Override
   public double calculateInternal(double x)
   {
      return k * (1.0 / (1 + Math.pow(1000.0 * Math.E * m, -1.0 * x + c))) + b;
   }
}
