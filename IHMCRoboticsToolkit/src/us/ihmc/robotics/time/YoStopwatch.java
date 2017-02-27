package us.ihmc.robotics.time;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;

public class YoStopwatch
{
   private final DoubleYoVariable timeProviderYoVariable;

   private DoubleYoVariable currentTimeYoVariable;
   private DoubleYoVariable lastTimeYoVariable;
   private DoubleYoVariable deltaTimeYoVariable;
   private DoubleYoVariable startTimeYoVariable;
   private LongYoVariable numLapsYoVariable;
   private DoubleYoVariable deltaSumYoVariable;

   private double currentTime;
   private double lastTime;
   private double deltaTime;
   private double startTime;
   private long numLaps;
   private double deltaSum;

   public YoStopwatch(String name, DoubleYoVariable timeYoVariable, YoVariableRegistry registry)
   {
      this(timeYoVariable);

      currentTimeYoVariable = new DoubleYoVariable(name + "CurrentTime", registry);
      lastTimeYoVariable = new DoubleYoVariable(name + "LastTime", registry);
      deltaTimeYoVariable = new DoubleYoVariable(name + "DeltaTime", registry);
      startTimeYoVariable = new DoubleYoVariable(name + "StartTime", registry);
      numLapsYoVariable = new LongYoVariable(name + "NumLaps", registry);
      deltaSumYoVariable = new DoubleYoVariable(name + "DeltaSum", registry);
   }

   public YoStopwatch(DoubleYoVariable timeYoVariable)
   {
      this.timeProviderYoVariable = timeYoVariable;
   }

   public YoStopwatch start()
   {
      reset();

      return this;
   }

   public void resetLap()
   {
      if (currentTimeYoVariable == null)
      {
         lastTime = timeProviderYoVariable.getDoubleValue();
      }
      else
      {
         lastTimeYoVariable.set(timeProviderYoVariable.getDoubleValue());
      }
   }

   public void reset()
   {
      if (currentTimeYoVariable == null)
      {
         startTime = timeProviderYoVariable.getDoubleValue();
         lastTime = startTime;

         numLaps = 0;
         deltaSum = 0.0;
      }
      else
      {
         startTimeYoVariable.set(timeProviderYoVariable.getDoubleValue());
         lastTimeYoVariable.set(startTimeYoVariable.getDoubleValue());

         numLapsYoVariable.set(0);
         deltaSumYoVariable.set(0.0);
      }
   }

   public double lap()
   {
      if (currentTimeYoVariable == null)
      {
         currentTime = timeProviderYoVariable.getDoubleValue();
         deltaTime = currentTime - lastTime;
         lastTime = currentTime;

         // for average lap
         numLaps++;
         deltaSum += deltaTime;

         return deltaTime;
      }
      else
      {
         currentTimeYoVariable.set(timeProviderYoVariable.getDoubleValue());
         deltaTimeYoVariable.set(currentTimeYoVariable.getDoubleValue() - lastTimeYoVariable.getDoubleValue());
         lastTimeYoVariable.set(currentTimeYoVariable.getDoubleValue());

         // for average lap
         numLapsYoVariable.set(numLapsYoVariable.getLongValue() + 1);
         deltaSumYoVariable.set(deltaSumYoVariable.getDoubleValue() + deltaTimeYoVariable.getDoubleValue());

         return deltaTimeYoVariable.getDoubleValue();
      }
   }

   public double averageLap()
   {
      if (currentTimeYoVariable == null)
      {
         return deltaSum / numLaps;
      }
      else
      {
         return deltaSumYoVariable.getDoubleValue() / numLapsYoVariable.getLongValue();
      }
   }

   public double totalElapsed()
   {
      if (currentTimeYoVariable == null)
      {
         return timeProviderYoVariable.getDoubleValue() - startTime;
      }
      else
      {
         return timeProviderYoVariable.getDoubleValue() - startTimeYoVariable.getDoubleValue();
      }
   }

   public double lapElapsed()
   {
      if (currentTimeYoVariable == null)
      {
         return timeProviderYoVariable.getDoubleValue() - lastTime;
      }
      else
      {
         return timeProviderYoVariable.getDoubleValue() - lastTimeYoVariable.getDoubleValue();
      }
   }
}
