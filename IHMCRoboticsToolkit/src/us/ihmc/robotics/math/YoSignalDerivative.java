package us.ihmc.robotics.math;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;

/**
 * This class computes the derivative of the input signal in two different ways.
 * The first way is the common differentiation using the DT.
 * The second way is a less noisy differentiation that waits for a change in the signal. In this case can 
 * be specified a tolerance to detect the change in the signal.
 *
 * @author not attributable
 */

public class YoSignalDerivative
{
   public enum DifferentiationMode
   {
      ON_SIGNAL_CHANGE, USING_DT
   }

   private static final double DEFAULT_TOLERANCE = Double.MIN_VALUE;
   private final String name;

   private EnumYoVariable<DifferentiationMode> differentiationMode;
   private DoubleYoVariable previousSignal;
   private DoubleYoVariable previousDerivative;
   private DoubleYoVariable previousTime;
   private DoubleYoVariable timeAtLastSignalChange;
   private DoubleYoVariable tolerance;
   private DoubleYoVariable lastSignalChange;

   public YoSignalDerivative(String name, YoVariableRegistry registry)
   {
      this.name = name;

      differentiationMode = new EnumYoVariable<DifferentiationMode>(name + "_differentiationMode", registry, DifferentiationMode.class);
      previousDerivative = new DoubleYoVariable(name + "_previousDerivative", registry);
      previousSignal = new DoubleYoVariable(name + "_previousSignal", registry);
      timeAtLastSignalChange = new DoubleYoVariable(name + "_timeAtLastSignalChange", registry);
      previousTime = new DoubleYoVariable(name + "_previousTime", registry);
      tolerance = new DoubleYoVariable(name + "_tolerance", registry);
      lastSignalChange = new DoubleYoVariable(name + "_lastSignalChange", registry);

      tolerance.set(DEFAULT_TOLERANCE);

      resetToZero();
   }

   public void setDifferentiationMode(DifferentiationMode mode)
   {
      differentiationMode.set(mode);
   }

   public void initialize(DifferentiationMode mode, double initialSignal, double initialTime, double initialDerivative)
   {
      initialize(mode, DEFAULT_TOLERANCE, initialSignal, initialTime, initialDerivative);
   }

   public void initialize(DifferentiationMode mode, double tolerance, double initialSignal, double initialTime, double initialDerivative)
   {
      setDifferentiationMode(mode);
      previousDerivative.set(initialDerivative);
      previousSignal.set(initialSignal);
      previousTime.set(initialTime);
      lastSignalChange.set(initialSignal);
      timeAtLastSignalChange.set(initialTime);
      this.tolerance.set(tolerance);
   }

   public double getDerivative(double signal, double time)
   {
      double ret = 0.0;

      switch (differentiationMode.getEnumValue())
      {
      case USING_DT:
      {
         ret = (signal - previousSignal.getDoubleValue()) / (time - previousTime.getDoubleValue());
         break;
      }

      case ON_SIGNAL_CHANGE:
      {
         if (Math.abs(signal - lastSignalChange.getDoubleValue()) < tolerance.getDoubleValue())
         {
            ret = previousDerivative.getDoubleValue();
         }
         else
         {
            ret = (signal - lastSignalChange.getDoubleValue()) / (time - timeAtLastSignalChange.getDoubleValue());
            timeAtLastSignalChange.set(time);
            lastSignalChange.set(signal);
         }

         break;
      }
      }

      previousSignal.set(signal);
      previousTime.set(time);
      previousDerivative.set(ret);

      return ret;
   }

   public void resetToZero()
   {
      differentiationMode.set(DifferentiationMode.USING_DT);
      previousDerivative.set(0.0);
      previousSignal.set(0.0);
      timeAtLastSignalChange.set(0.0);
      previousTime.set(0.0);
      lastSignalChange.set(0.0);
   }

   public DifferentiationMode getDifferentiationMode()
   {
      return differentiationMode.getEnumValue();
   }

   public String getName()
   {
      return name;
   }

}