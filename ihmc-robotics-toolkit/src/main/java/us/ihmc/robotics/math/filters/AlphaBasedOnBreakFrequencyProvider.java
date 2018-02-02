package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;

/**
 * This class calculates the alpha value given a break frequency provider
 * 
 * The value gets cached to avoid unneccessary calculations
 * 
 * @author jesper
 *
 */
public class AlphaBasedOnBreakFrequencyProvider implements DoubleProvider
{

   private final DoubleProvider breakFrequencyProvider;
   private final double dt;

   private double previousBreakFrequency = Double.NaN;
   private double alpha = 0.0;

   public AlphaBasedOnBreakFrequencyProvider(DoubleProvider breakFrequencyProvider, double dt)
   {
      this.breakFrequencyProvider = breakFrequencyProvider;
      this.dt = dt;
   }

   @Override
   /**
    * Get the desired alpha value, based on the break frequency given by the breakFrequencyProvider
    * 
    * The value gets cached, and checked based on the current value of the breakFrequencyProvider.
    * This will be safe to rewind.
    * 
    */
   public double getValue()
   {
      double currentBreakFrequency = breakFrequencyProvider.getValue();
      if (currentBreakFrequency != previousBreakFrequency)
      {
         alpha = AlphaFilteredYoVariable.computeAlphaGivenBreakFrequencyProperly(currentBreakFrequency, dt);
         previousBreakFrequency = currentBreakFrequency;
      }

      return alpha;
   }

}
