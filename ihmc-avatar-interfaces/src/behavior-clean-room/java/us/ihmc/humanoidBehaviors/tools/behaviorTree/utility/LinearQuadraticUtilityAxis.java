package us.ihmc.humanoidBehaviors.tools.behaviorTree.utility;

import java.util.function.DoubleSupplier;

public class LinearQuadraticUtilityAxis extends UtilityAxis
{
   /**
    * @param m slope
    * @param k exponent
    * @param b y-intercept (vertical shift)
    * @param c x-intercept (horizontal shift)
    */
   public LinearQuadraticUtilityAxis(double m, double k, double b, double c, DoubleSupplier xSupplier)
   {
      super(m, k, b, c, xSupplier);
   }

   @Override
   public double calculateInternal(double x)
   {
      return m * Math.pow(x - c, k) + b;
   }
}
