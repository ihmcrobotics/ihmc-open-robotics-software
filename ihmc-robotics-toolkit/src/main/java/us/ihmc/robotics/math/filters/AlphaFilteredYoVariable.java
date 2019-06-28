package us.ihmc.robotics.math.filters;

import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
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

   public static DoubleProvider createAlphaYoDouble(String namePrefix, double initialValue, YoVariableRegistry registry)
   {
      YoDouble maxRate = new YoDouble(namePrefix + "AlphaVariable", registry);
      maxRate.set(initialValue);
      return maxRate;
   }

   public AlphaFilteredYoVariable(String name, YoVariableRegistry registry, double alpha)
   {
      this(name, registry, alpha, null);
   }

   public AlphaFilteredYoVariable(String name, YoVariableRegistry registry, double alpha, YoDouble positionVariable)
   {
      this(name, "", registry, createAlphaYoDouble(name, alpha, registry), positionVariable);
   }

   public AlphaFilteredYoVariable(String name, YoVariableRegistry registry, DoubleProvider alphaVariable)
   {
      this(name, "", registry, alphaVariable, null);
   }

   public AlphaFilteredYoVariable(String name, String description, YoVariableRegistry registry, DoubleProvider alphaVariable)
   {
      this(name, description, registry, alphaVariable, null);
   }

   public AlphaFilteredYoVariable(String name, YoVariableRegistry registry, DoubleProvider alphaVariable, YoDouble positionVariable)
   {
      this(name, "", registry, alphaVariable, positionVariable);
   }

   public AlphaFilteredYoVariable(String name, String description, YoVariableRegistry registry, DoubleProvider alphaVariable, YoDouble positionVariable)
   {
      super(name, description, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", description, registry);
      this.position = positionVariable;

      if (alphaVariable == null)
         alphaVariable = createAlphaYoDouble(name, 0.0, registry);
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
         set(alpha * getDoubleValue() + (1.0 - alpha) * currentPosition);
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

      double alpha = 1.0 - breakFrequencyInHertz * 2.0 * Math.PI * dt;

      alpha = MathTools.clamp(alpha, 0.0, 1.0);

      return alpha;
   }

   public static double computeAlphaGivenBreakFrequencyProperly(double breakFrequencyInHertz, double dt)
   {
      if (Double.isInfinite(breakFrequencyInHertz))
         return 0.0;

      double omega = 2.0 * Math.PI * breakFrequencyInHertz;
      double alpha = (1.0 - omega * dt / 2.0) / (1.0 + omega * dt / 2.0);
      alpha = MathTools.clamp(alpha, 0.0, 1.0);
      return alpha;
   }

   public static double computeBreakFrequencyGivenAlpha(double alpha, double dt)
   {
      return (1.0 - alpha) / (Math.PI * dt + Math.PI * alpha * dt);
   }

   public static void main(String[] args)
   {
      double dt = 1 / 1e3;

      for (double i = 2; i < 1.0 / dt; i = i * 1.2)
      {
         double alpha = computeAlphaGivenBreakFrequency(i, dt);
         double alphaProperly = computeAlphaGivenBreakFrequencyProperly(i, dt);
         System.out.println("freq=" + i + ", alpha=" + alpha + ", alphaProperly=" + alphaProperly);
      }

      System.out.println(computeBreakFrequencyGivenAlpha(0.8, 0.006));
      System.out.println(computeAlphaGivenBreakFrequencyProperly(20, 0.006));
      System.out.println(computeAlphaGivenBreakFrequencyProperly(20, 0.003));
   }

   public boolean getHasBeenCalled()
   {
      return hasBeenCalled.getBooleanValue();
   }

}
