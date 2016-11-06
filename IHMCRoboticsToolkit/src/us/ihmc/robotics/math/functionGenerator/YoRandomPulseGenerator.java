package us.ihmc.robotics.math.functionGenerator;

import java.util.Random;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


/**
 * User: pneuhaus
 * Date: 11/2/13
 * Time: 3:27 PM
 */
public class YoRandomPulseGenerator
{
   private YoVariableRegistry registry;

   private DoubleYoVariable time;

   private final DoubleYoVariable percentChancePerSecond;
   private final DoubleYoVariable amplitudeMax;
   private final DoubleYoVariable amplitudeUsed;
   private final DoubleYoVariable durationMax, durationUsed;
   private final DoubleYoVariable timeStartPulse;
   private final DoubleYoVariable timeLastRolledTheDie;

   private final BooleanYoVariable pulseStarted;
   private final DoubleYoVariable value;
   private final DoubleYoVariable randomNumber;

   private final Random random = new Random();//(1776L);

   public YoRandomPulseGenerator(String name, YoVariableRegistry parentRegistry)
   {
      this(name, null, parentRegistry);
   }

   public YoRandomPulseGenerator(String name, DoubleYoVariable time, YoVariableRegistry parentRegistry)
   {
      registry = new YoVariableRegistry(name + "YoPulseGen");

      percentChancePerSecond = new DoubleYoVariable(name + "PercentChancePerSecond", registry);
      amplitudeMax = new DoubleYoVariable(name + "AmplitudeMax", registry);
      amplitudeUsed = new DoubleYoVariable(name + "AmplitudeUsed", registry);
      durationMax = new DoubleYoVariable(name + "DurationMax", registry);
      durationUsed = new DoubleYoVariable(name + "DurationUsed", registry);

      timeStartPulse = new DoubleYoVariable(name + "TimeStartPulse", registry);
      timeLastRolledTheDie = new DoubleYoVariable(name + "TimeLastRolledTheDie", registry);
      pulseStarted = new BooleanYoVariable(name + "PulseStarted", registry);
      value = new DoubleYoVariable(name + "Value", registry);
      randomNumber = new DoubleYoVariable(name + "RandomNumber", registry);

      this.time = time;

      if (parentRegistry != null)
         parentRegistry.addChild(registry);
   }

   public void setPercentChancePerSecond(double percentChancePerSecond)
   {
      this.percentChancePerSecond.set(percentChancePerSecond);
   }

   public void setAmplitudeMax(double amplitudeMax)
   {
      this.amplitudeMax.set(amplitudeMax);
   }

   public void setDuration(double durationMax)
   {
      this.durationMax.set(durationMax);
   }

   public double getValue()
   {
      if (this.time == null)
         throw new RuntimeException(
             "YoRandomPulseGenerator wasn't created with a time YoVariable. Need to create with a time variable or call getValue(double time) instead");

      return this.getValue(time.getDoubleValue());
   }

   public double getValue(double time)
   {
      if (pulseStarted.getBooleanValue())
      {
         double percentIntoPulse;
         if (durationUsed.getDoubleValue() > 0.0)
            percentIntoPulse = (time - timeStartPulse.getDoubleValue()) / durationUsed.getDoubleValue();
         else
            percentIntoPulse = 1.0;

         percentIntoPulse = MathTools.clipToMinMax(percentIntoPulse, 0.0, 1.0);

         double temp = (Math.sin(2.0 * Math.PI * (percentIntoPulse - 0.25)) + 1.0)/2.0;
         value.set(amplitudeUsed.getDoubleValue() * temp);

         if (percentIntoPulse >= 1.0)
            pulseStarted.set(false);
      }
      else
      {
         // Only "roll the die" every second
         double period = time - timeLastRolledTheDie.getDoubleValue();
         if (period >= 1.0)
         {
            double randomNumber = random.nextDouble();
            timeLastRolledTheDie.set(time);

            this.randomNumber.set(randomNumber);

            if (randomNumber <= percentChancePerSecond.getDoubleValue())
            {
               pulseStarted.set(true);
               timeStartPulse.set(time);

               double alpha = 0.3;
               amplitudeUsed.set(amplitudeMax.getDoubleValue() * (alpha + (1.0 - alpha) * random.nextDouble()));

               alpha = 0.1;
               durationUsed.set(durationMax.getDoubleValue() * (alpha + (1.0 - alpha) * random.nextDouble()));



               // This will make sure that the die is rolled right after the pulse
               timeLastRolledTheDie.set(time - 1.0);
            }

         }

         value.set(0.0);


      }

      return value.getDoubleValue();
   }
}
