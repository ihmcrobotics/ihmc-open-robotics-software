package us.ihmc.yoVariables.filters;

import us.ihmc.commons.AngleTools;
import us.ihmc.commons.MathTools;

public class AlphaFilterTools
{
   /**
    * This computes the alpha
    * variable for an alpha filtered yo variable that is equivalent to a first order low pass frequnecy at filter {@param breakFrequencyInHertz}.
    *
    * <p>
    * If this
    * calculation is done as part of a double provider, consider using {@link AlphaBasedOnBreakFrequencyProvider}, which only updates the alpha estimate
    * on a change in the break frequency, saving computation.
    * </p>
    *
    * @return alpha value
    */
   public static double computeAlphaGivenBreakFrequencyProperly(double breakFrequencyInHertz, double dt)
   {
      if (Double.isInfinite(breakFrequencyInHertz))
         return 0.0;

      double omega = AngleTools.TwoPI * breakFrequencyInHertz;
      double alpha = (1.0 - omega * dt / 2.0) / (1.0 + omega * dt / 2.0);
      alpha = MathTools.clamp(alpha, 0.0, 1.0);
      return alpha;
   }

   /**
    * This computes the break frequency of a first order low-pass filter given an alpha value.
    *
    * @return alpha value
    */
   public static double computeBreakFrequencyGivenAlpha(double alpha, double dt)
   {
      return (1.0 - alpha) / (Math.PI * dt * (1.0 + alpha));
   }
}
