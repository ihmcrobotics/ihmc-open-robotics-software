package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class RampedAlphaFilteredYoVariable extends AlphaFilteredYoVariable
{
   private final YoDouble time, startTime;

   private final YoDouble startAlpha, endAlpha, rampTime;

   public RampedAlphaFilteredYoVariable(String name, YoVariableRegistry registry, YoDouble positionVariable, YoDouble time)
   {
      super(name, registry, 0.0, positionVariable);

      this.time = time;

      this.startAlpha = new YoDouble(name + "RampStartAlpha", registry);
      this.endAlpha = new YoDouble(name + "RampEndAlpha", registry);
      this.rampTime = new YoDouble(name + "RampTime", registry);
      this.startTime = new YoDouble(name + "RampStartTime", registry);
   }

   public void resetFilter()
   {
      startTime.set(time.getDoubleValue());
   }

   public void update()
   {
      if (rampTime.getDoubleValue() < 1e-7)
      {
         this.setAlpha(endAlpha.getDoubleValue());
      }

      else
      {
         double percent = (time.getDoubleValue() - startTime.getDoubleValue()) / rampTime.getDoubleValue();

         if (percent < 0.0)
            percent = 0.0;
         if (percent > 1.0)
            percent = 1.0;

         this.setAlpha(startAlpha.getDoubleValue() + percent * (endAlpha.getDoubleValue() - startAlpha.getDoubleValue()));
      }

      super.update();
   }

   public void setStartAlpha(double startAlpha)
   {
      this.startAlpha.set(startAlpha);
   }

   public void setEndAlpha(double endAlpha)
   {
      this.endAlpha.set(endAlpha);
   }

   public void setRampTime(double rampTime)
   {
      this.rampTime.set(rampTime);
   }

   public double getStartAlphas()
   {
      return startAlpha.getDoubleValue();
   }

   public double getEndAlphas()
   {
      return endAlpha.getDoubleValue();
   }

   public double getRampTime()
   {
      return rampTime.getDoubleValue();
   }
}
