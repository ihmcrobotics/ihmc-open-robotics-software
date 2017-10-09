package us.ihmc.robotics.time;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class YoStopwatch
{
   private final YoDouble timeProviderYoVariable;

   private YoDouble yoLapStart;
   private YoLong yoLapCount;
   private YoDouble yoRecordedLapTotal;
   private YoBoolean yoSuspended;
   private YoDouble yoSuspendStart;
   private YoDouble yoResumedSuspensionTotal;

   private double lapStart;
   private long lapCount;
   private double recordedLapTotal;
   private boolean suspended;
   private double suspendStart;
   private double resumedSuspensionTotal;

   public YoStopwatch(String name, YoDouble timeYoVariable, YoVariableRegistry registry)
   {
      this(timeYoVariable);

      yoLapStart = new YoDouble(name + "LapStart", registry);
      yoLapCount = new YoLong(name + "LapCount", registry);
      yoRecordedLapTotal = new YoDouble(name + "RecordedLapTotal", registry);
      yoSuspended = new YoBoolean(name + "Suspended", registry);
      yoSuspendStart = new YoDouble(name + "SuspendStart", registry);
      yoResumedSuspensionTotal = new YoDouble(name + "ResumedSuspensionTotal", registry);
      yoLapStart.setToNaN();
   }

   public YoStopwatch(YoDouble timeYoVariable)
   {
      this.timeProviderYoVariable = timeYoVariable;
      lapStart = Double.NaN;
   }

   public YoStopwatch start()
   {
      reset();

      return this;
   }

   public void resetLap()
   {
      if (yoLapStart == null)
      {
         lapStart = now();
      }
      else
      {
         yoLapStart.set(now());
      }

      resetSuspension();
   }

   public void reset()
   {
      if (yoLapStart == null)
      {
         lapStart = now();

         resetSuspension();

         lapCount = 0;
         recordedLapTotal = 0.0;
      }
      else
      {
         yoLapStart.set(now());

         resetSuspension();

         yoLapCount.set(0);
         yoRecordedLapTotal.set(0.0);
      }
   }

   public double lap()
   {
      double now = now();
      double lapDuration = lapElapsed(now);
      if (yoLapStart == null)
      {
         lapStart = now;

         resetSuspension();

         // for average lap
         lapCount++;
         recordedLapTotal += lapDuration;
      }
      else
      {
         yoLapStart.set(now);

         resetSuspension();

         // for average lap
         yoLapCount.add(1);
         yoRecordedLapTotal.add(lapDuration);
      }
      return lapDuration;
   }

   public double averageLap()
   {
      if (yoLapStart == null)
      {
         return recordedLapTotal / lapCount;
      }
      else
      {
         return yoRecordedLapTotal.getDoubleValue() / yoLapCount.getLongValue();
      }
   }

   public double lapElapsed()
   {
      return lapElapsed(now());
   }

   public double totalElapsed()
   {
      if (yoLapStart == null)
      {
         return recordedLapTotal + lapElapsed(now());
      }
      else
      {
         return yoRecordedLapTotal.getDoubleValue() + lapElapsed(now());
      }
   }

   public void suspend()
   {
      if (yoLapStart == null)
      {
         if (!suspended)
         {
            suspended = true;
            suspendStart = now();
         }
      }
      else
      {
         if (!yoSuspended.getBooleanValue())
         {
            yoSuspended.set(true);
            yoSuspendStart.set(now());
         }
      }
   }

   public void resume()
   {
      if (yoLapStart == null)
      {
         if (suspended)
         {
            suspended = false;
            resumedSuspensionTotal += now() - suspendStart;
         }
      }
      else
      {
         if (yoSuspended.getBooleanValue())
         {
            yoSuspended.set(false);
            yoResumedSuspensionTotal.add(now() - yoSuspendStart.getDoubleValue());
         }
      }
   }

   private double lapElapsed(double now)
   {
      if (yoLapStart == null)
      {
         double lapElapsed = now - lapStart;
         lapElapsed -= resumedSuspensionTotal;
         if (suspended)
         {
            lapElapsed -= (now - suspendStart);
         }
         return lapElapsed;
      }
      else
      {
         double lapElapsed = now - yoLapStart.getDoubleValue();
         lapElapsed -= yoResumedSuspensionTotal.getDoubleValue();
         if (yoSuspended.getBooleanValue())
         {
            lapElapsed -= (now - yoSuspendStart.getDoubleValue());
         }
         return lapElapsed;
      }
   }

   private double now()
   {
      return timeProviderYoVariable.getDoubleValue();
   }

   private void resetSuspension()
   {
      if (yoLapStart == null)
      {
         suspended = false;
         resumedSuspensionTotal = 0.0;
      }
      else
      {
         yoSuspended.set(false);
         yoResumedSuspensionTotal.set(0.0);
      }
   }
}
