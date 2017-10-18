package us.ihmc.robotics.math;

import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This class produces a square dither signal in the case were the controller frequency is close to the dither
 * frequency. In that case a function generator is not useful.
 *
 * @author unknownid
 *
 */
public class DitherProducer
{
   private final YoVariableRegistry registry;
   private final YoDouble desiredDitherFrequency;
   private final YoDouble ditherFrequency;
   private final YoDouble amplitude;
   private final double maxFrequency;

   private final YoDouble dither;

   public DitherProducer(String namePrefix, YoVariableRegistry parentRegistry, double controlDT)
   {
      registry = new YoVariableRegistry(namePrefix);
      desiredDitherFrequency = new YoDouble(namePrefix + "_dither_frequency_desired", registry);
      desiredDitherFrequency.set(0.0);
      desiredDitherFrequency.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            checkFrequency();
         }
      });

      ditherFrequency = new YoDouble(namePrefix + "_dither_frequency", registry);
      ditherFrequency.set(0.0);

      amplitude = new YoDouble(namePrefix + "_dither_amplitude", registry);
      amplitude.set(0.0);

      dither = new YoDouble(namePrefix + "_dither_output", registry);
      dither.set(0.0);

      maxFrequency = 1.0 / (2.0 * controlDT);
      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   protected void checkFrequency()
   {
      if (desiredDitherFrequency.getDoubleValue() >= maxFrequency)
      {
         ditherFrequency.set(maxFrequency);
         return;
      }

      if (desiredDitherFrequency.getDoubleValue() <= 0.0)
      {
         ditherFrequency.set(0.0);
         return;
      }

      double lowerMatch = maxFrequency;
      while (lowerMatch > desiredDitherFrequency.getDoubleValue())
      {
         lowerMatch = lowerMatch / 2.0;
      }
      double upperMatch = lowerMatch * 2.0;

      if (desiredDitherFrequency.getDoubleValue() - lowerMatch < upperMatch - desiredDitherFrequency.getDoubleValue())
      {
         ditherFrequency.set(lowerMatch);
      }
      else
      {
         ditherFrequency.set(upperMatch);
      }
   }

   public void setFrequency(double desiredFrequency)
   {
      desiredDitherFrequency.set(desiredFrequency);
   }

   public double getFrequency()
   {
      return ditherFrequency.getDoubleValue();
   }

   public void setAmplitude(double desiredAmplitude)
   {
      amplitude.set(desiredAmplitude);
   }

   public double getAmplitude()
   {
      return amplitude.getDoubleValue();
   }

   public double getValue(double time)
   {
      double controlDT = 1.0 / (2.0 * maxFrequency);
      long tick = (long) (time / controlDT);

      int ticks = (int) (1.0 / (ditherFrequency.getDoubleValue() * controlDT));
      tick = tick % ticks;

      if(2 * tick >= ticks)
      {
         dither.set(amplitude.getDoubleValue());
      }
      else
      {
         dither.set(- amplitude.getDoubleValue());
      }

      return dither.getDoubleValue();
   }

//   public static void main(String... args)
//   {
//      double controlDT = 0.004;
//      double desFreq = 1000;
//      YoVariableRegistry registry = new YoVariableRegistry("test");
//      YoDouble time = new YoDouble("time", registry);
//      YoDouble signal = new YoDouble("signal", registry);
//      DitherProducer ditherProducer = new DitherProducer("dither", registry, controlDT);
//
//      ditherProducer.setFrequency(desFreq);
//      ditherProducer.setAmplitude(1.0);
//      System.out.println("controlDT = " + controlDT + ", desired = " + desFreq + ", actual = " + ditherProducer.getFrequency());
//
//      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Test"));
//      scs.addYoVariableRegistry(registry);
//      scs.startOnAThread();
//
//      for (double t = 0.0; t < 20.0; t+=controlDT)
//      {
//         time.set(t);
//         signal.set(ditherProducer.getValue(t));
//         scs.tickAndUpdate();
//      }
//   }
}
