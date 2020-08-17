package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

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
    * Create a new provider backed by a frequency with the given name.
    *  
    * @param name Name of the break frequency parameter
    * @param dt Time step
    * @param registry Parent registry for the break frequency parameter
    * @param defaultBreakFrequency Default value for the break frequency
    */
   public AlphaBasedOnBreakFrequencyProvider(String name, double dt, YoRegistry registry, double defaultBreakFrequency)
   {
      this(new DoubleParameter(name, registry, defaultBreakFrequency), dt);
   }

   /**
    * Create a new provider backed by a frequency with the given name.
    *  
    * @param name Name of the break frequency parameter
    * @param dt Time step
    * @param registry Parent registry for the break frequency parameter
    */
   public AlphaBasedOnBreakFrequencyProvider(String name, double dt, YoRegistry registry)
   {
      this.breakFrequencyProvider = new DoubleParameter(name, registry, Double.NaN);
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
