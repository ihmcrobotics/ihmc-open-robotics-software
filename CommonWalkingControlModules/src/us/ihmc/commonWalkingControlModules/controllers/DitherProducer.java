package us.ihmc.commonWalkingControlModules.controllers;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;

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
   private final DoubleYoVariable desiredDitherFrequency;
   private final DoubleYoVariable ditherFrequency;
   private final DoubleYoVariable amplitude;
   private final double maxFrequency;
   
   private final DoubleYoVariable dither;
   
   public DitherProducer(String namePrefix, YoVariableRegistry parentRegistry, double controlDT)
   {
      registry = new YoVariableRegistry(namePrefix);
      desiredDitherFrequency = new DoubleYoVariable(namePrefix + "dither_frequency_desired", registry);
      desiredDitherFrequency.set(0.0);
      desiredDitherFrequency.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            checkFrequency();
         }
      });
      
      ditherFrequency = new DoubleYoVariable(namePrefix + "dither_frequency", registry);
      ditherFrequency.set(0.0);
      
      amplitude = new DoubleYoVariable(namePrefix + "dither_amplitude", registry);
      amplitude.set(0.0);
      
      dither = new DoubleYoVariable(namePrefix + "dither_output", registry);
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
//      DoubleYoVariable time = new DoubleYoVariable("time", registry);
//      DoubleYoVariable signal = new DoubleYoVariable("signal", registry);
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
