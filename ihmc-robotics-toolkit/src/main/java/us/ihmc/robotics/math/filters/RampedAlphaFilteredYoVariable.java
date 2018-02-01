package us.ihmc.robotics.math.filters;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * <p>
 * Title:
 * </p>
 *
 * <p>
 * Description:
 * </p>
 *
 * <p>
 * Copyright: Copyright (c) 2006
 * </p>
 *
 * <p>
 * Company:
 * </p>
 *
 * @author not attributable
 * @version 1.0
 */
public class RampedAlphaFilteredYoVariable extends YoDouble
{
   private final YoDouble time, startTime;

   private final YoDouble alphaVariable;
   private final YoDouble startAlpha, endAlpha, rampTime;

   private final YoDouble position;
   protected final YoBoolean hasBeenCalled;

   public RampedAlphaFilteredYoVariable(String name, YoVariableRegistry registry, YoDouble positionVariable, YoDouble time)
   {
      super(name, registry);
      this.hasBeenCalled = new YoBoolean(name + "HasBeenCalled", registry);
      this.alphaVariable = new YoDouble(name + "AlphaVariable", registry);
      this.position = positionVariable;

      this.time = time;

      this.startAlpha = new YoDouble(name + "RampStartAlpha", registry);
      this.endAlpha = new YoDouble(name + "RampEndAlpha", registry);
      this.rampTime = new YoDouble(name + "RampTime", registry);
      this.startTime = new YoDouble(name + "RampStartTime", registry);

      reset();
   }

   public void reset()
   {
      hasBeenCalled.set(false);
   }

   public void resetFilter()
   {
      startTime.set(time.getDoubleValue());
   }

   public void update()
   {
      if (rampTime.getDoubleValue() < 1e-7)
      {
         this.alphaVariable.set(endAlpha.getDoubleValue());
      }

      else
      {
         double percent = (time.getDoubleValue() - startTime.getDoubleValue()) / rampTime.getDoubleValue();

         if (percent < 0.0)
            percent = 0.0;
         if (percent > 1.0)
            percent = 1.0;

         this.alphaVariable.set(startAlpha.getDoubleValue() + percent * (endAlpha.getDoubleValue() - startAlpha.getDoubleValue()));
      }

      if (!hasBeenCalled.getBooleanValue())
      {
         hasBeenCalled.set(true);
         set(position.getDoubleValue());
      }

      set(alphaVariable.getDoubleValue() * getDoubleValue() + (1.0 - alphaVariable.getDoubleValue()) * position.getDoubleValue());
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
