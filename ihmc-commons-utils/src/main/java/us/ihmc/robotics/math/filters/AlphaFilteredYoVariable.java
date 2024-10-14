package us.ihmc.robotics.math.filters;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * @author jrebula
 *         </p>
 *         <p>
 *         LittleDogVersion06:
 *         us.ihmc.LearningLocomotion.Version06.util.YoAlphaFilteredVariable,
 *         9:34:00 AM, Aug 29, 2006
 *         </p>=
 *         <p>
 *         A YoAlphaFilteredVariable is a filtered version of an input YoVar.
 *         Either a YoVariable holding the unfiltered val is passed in to the
 *         constructor and update() is called every tick, or update(double) is
 *         called every tick. The YoAlphaFilteredVariable updates it's val
 *         with the current filtered version using
 *         </p>
 *         <pre>
 *            filtered_{n} = alpha * filtered_{n-1} + (1 - alpha) * raw_{n}
 *         </pre>
 *
 *          For alpha=0 -> no filtered
 *         For alpha=1 -> 100% filtered, no use of raw signal
 */
public class AlphaFilteredYoVariable extends YoDouble implements ProcessingYoVariable
{
   private final DoubleProvider alphaVariable;

   private final YoDouble position;
   protected final YoBoolean hasBeenCalled;

   /**
    * Please use
    * @param namePrefix
    * @param initialValue
    * @param registry
    * @return
    */
   @Deprecated
   public static DoubleProvider createAlphaYoDouble(String namePrefix, double initialValue, YoRegistry registry)
   {
      return VariableTools.createAlphaYoDouble(namePrefix, "", initialValue, registry);
   }


   public AlphaFilteredYoVariable(String name, YoRegistry registry, double alpha)
   {
      this(name, registry, alpha, null);
   }

   public AlphaFilteredYoVariable(String name, YoRegistry registry, double alpha, YoDouble positionVariable)
   {
      this(name, "", registry, VariableTools.createAlphaYoDouble(name, "", alpha, registry), positionVariable);
   }

   public AlphaFilteredYoVariable(String name, YoRegistry registry, DoubleProvider alphaVariable)
   {
      this(name, "", registry, alphaVariable, null);
   }

   public AlphaFilteredYoVariable(String name, String description, YoRegistry registry, DoubleProvider alphaVariable)
   {
      this(name, description, registry, alphaVariable, null);
   }

   public AlphaFilteredYoVariable(String name, YoRegistry registry, DoubleProvider alphaVariable, YoDouble positionVariable)
   {
      this(name, "", registry, alphaVariable, positionVariable);
   }

   public AlphaFilteredYoVariable(String name, String description, YoRegistry registry, DoubleProvider alphaVariable, YoDouble positionVariable)
   {
      super(name, description, registry);
      this.hasBeenCalled = VariableTools.createHasBeenCalledYoBoolean(name, "", registry);
      this.position = positionVariable;

      if (alphaVariable == null)
         alphaVariable = VariableTools.createAlphaYoDouble(name, "", 0.0, registry);
      this.alphaVariable = alphaVariable;

      reset();
   }

   @Override
   public void reset()
   {
      hasBeenCalled.set(false);
   }

   @Override
   public void update()
   {
      if (position == null)
      {
         throw new NullPointerException("YoAlphaFilteredVariable must be constructed with a non null "
               + "position variable to call update(), otherwise use update(double)");
      }

      update(position.getDoubleValue());
   }

   public void update(double currentPosition)
   {
      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         set(currentPosition);
      }
      else
      {
         double alpha = alphaVariable.getValue();
         set(EuclidCoreTools.interpolate(currentPosition, getDoubleValue(), alpha));
      }
   }

   /**
    * This method is replaced by computeAlphaGivenBreakFrequencyProperly. It is fine to keep using this method is currently using it, knowing that
    * the actual break frequency is not exactly what you are asking for.
    * 
    * @param breakFrequencyInHertz
    * @param dt
    * @return
    */
   @Deprecated
   public static double computeAlphaGivenBreakFrequency(double breakFrequencyInHertz, double dt)
   {
      if (Double.isInfinite(breakFrequencyInHertz))
         return 0.0;

      double alpha = 1.0 - breakFrequencyInHertz * AngleTools.TwoPI * dt;

      alpha = MathTools.clamp(alpha, 0.0, 1.0);

      return alpha;
   }

   /**
    * This method has been replaced with {@link AlphaFilterTools#computeAlphaGivenBreakFrequencyProperly(double, double)}. It computes the alpha
    * variable for an alpha filtered yo variable that is equivalent to a first order low pass frequnecy at filter {@param breakFrequencyInHertz}
    * @return alpha value
    */
   @Deprecated
   public static double computeAlphaGivenBreakFrequencyProperly(double breakFrequencyInHertz, double dt)
   {
      return AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(breakFrequencyInHertz, dt);
   }

   /**
    * This method has been replaced with {@link AlphaFilterTools#computeBreakFrequencyGivenAlpha(double, double)}.
    * This computes the break frequency of a first order low-pass filter given an alpha value.
    *
    * @return alpha value
    */
   @Deprecated
   public static double computeBreakFrequencyGivenAlpha(double alpha, double dt)
   {
      return AlphaFilterTools.computeBreakFrequencyGivenAlpha(alpha, dt);
   }

   public boolean getHasBeenCalled()
   {
      return hasBeenCalled.getBooleanValue();
   }

}
