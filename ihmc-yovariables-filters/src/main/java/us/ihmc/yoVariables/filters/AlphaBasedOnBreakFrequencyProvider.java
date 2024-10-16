package us.ihmc.yoVariables.filters;

import us.ihmc.yoVariables.providers.DoubleProvider;

/**
 * This class calculates the alpha value given a break frequency provider
 * 
 * The value gets cached to avoid unnecessary calculations
 * 
 * @author Jesper Smith
 *
 */
public class AlphaBasedOnBreakFrequencyProvider implements DoubleProvider
{

   private final DoubleProvider breakFrequencyProvider;
   private final double dt;

   private double previousBreakFrequency = Double.NaN;
   private double alpha = 0.0;

   /**
    * Create a new provider using the break frequency provided by double provider
    * 
    * @param breakFrequencyProvider
    * @param dt
    */
   public AlphaBasedOnBreakFrequencyProvider(DoubleProvider breakFrequencyProvider, double dt)
   {
      this.breakFrequencyProvider = breakFrequencyProvider;
      this.dt = dt;
   }
   
   /**
    * Get the desired alpha value, based on the break frequency given by the breakFrequencyProvider
    * 
    * The value gets cached, and checked based on the current value of the breakFrequencyProvider.
    * This will be safe to rewind.
    * 
    * @return alpha variable based on the value of breakFrequencyProvider and dt
    */
   @Override
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
