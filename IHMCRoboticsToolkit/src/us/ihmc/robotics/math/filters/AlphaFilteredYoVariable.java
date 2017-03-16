package us.ihmc.robotics.math.filters;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

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
public class AlphaFilteredYoVariable extends DoubleYoVariable implements ProcessingYoVariable
{
   private final DoubleYoVariable alphaVariable;

   private final DoubleYoVariable position;
   protected final BooleanYoVariable hasBeenCalled;

   public AlphaFilteredYoVariable(String name, YoVariableRegistry registry, double alpha)
   {
      this(name, "", registry, alpha, null, null);
   }

   public AlphaFilteredYoVariable(String name, YoVariableRegistry registry, DoubleYoVariable alphaVariable)
   {
      this(name, "", registry, 0.0, alphaVariable, null);
   }

   public AlphaFilteredYoVariable(String name, String description, YoVariableRegistry registry, DoubleYoVariable alphaVariable)
   {
      this(name, description, registry, 0.0, alphaVariable, null);
   }

   public AlphaFilteredYoVariable(String name, YoVariableRegistry registry, double alpha, DoubleYoVariable positionVariable)
   {
      this(name, "", registry, alpha, null, positionVariable);
   }

   public AlphaFilteredYoVariable(String name, YoVariableRegistry registry, DoubleYoVariable alphaVariable, DoubleYoVariable positionVariable)
   {
      this(name, "", registry, 0.0, alphaVariable, positionVariable);
   }

   public AlphaFilteredYoVariable(String name, String description, YoVariableRegistry registry, DoubleYoVariable alphaVariable, DoubleYoVariable positionVariable)
   {
      this(name, description, registry, 0.0, alphaVariable, positionVariable);
   }

   private AlphaFilteredYoVariable(String name, String description, YoVariableRegistry registry, double alpha, DoubleYoVariable alphaVariable, DoubleYoVariable positionVariable)
   {
      super(name, description, registry);
      this.hasBeenCalled = new BooleanYoVariable(name + "HasBeenCalled", description, registry);
      position = positionVariable;

      if (alphaVariable != null)
         this.alphaVariable = alphaVariable;
      else
         this.alphaVariable = new DoubleYoVariable(name + "AlphaVariable", registry);

      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

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


      set(alphaVariable.getDoubleValue() * getDoubleValue() + (1.0 - alphaVariable.getDoubleValue()) * currentPosition);

   }

   public void setAlpha(double alpha)
   {
      this.alphaVariable.set(alpha);
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

      for (double i = 2; i < 1.0/dt; i = i * 1.2)
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
