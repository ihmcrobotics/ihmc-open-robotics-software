package us.ihmc.robotics.controllers;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;

public class YoLimitedPIDGains extends YoPIDGains
{
   private final RateLimitedYoVariable limitedKp;
   private final RateLimitedYoVariable limitedKi;
   private final RateLimitedYoVariable limitedKd;

   private final DoubleYoVariable maxKpRate;
   private final DoubleYoVariable maxKiRate;
   private final DoubleYoVariable maxKdRate;

   public YoLimitedPIDGains(String suffix, double controlDT, YoVariableRegistry registry)
   {
      super(suffix, registry);

      maxKpRate = new DoubleYoVariable("maxKpRate" + suffix, registry);
      maxKdRate = new DoubleYoVariable("maxKdRate" + suffix, registry);
      maxKiRate = new DoubleYoVariable("maxKiRate" + suffix, registry);

      limitedKi = new RateLimitedYoVariable("limitedKi" + suffix, registry, maxKiRate, ki, controlDT);
      limitedKp = new RateLimitedYoVariable("limitedKp" + suffix, registry, maxKpRate, kp, controlDT);
      limitedKd = new RateLimitedYoVariable("limitedKd" + suffix, registry, maxKdRate, kd, controlDT);

      maxKpRate.set(Double.POSITIVE_INFINITY);
      maxKdRate.set(Double.POSITIVE_INFINITY);
      maxKiRate.set(Double.POSITIVE_INFINITY);
   }

   public void setMaxKiRate(double maxKiRate)
   {
      this.maxKiRate.set(maxKiRate);
   }

   public void setMaxKpRate(double maxKpRate)
   {
      this.maxKpRate.set(maxKpRate);
   }

   public void setMaxKdRate(double maxKdRate)
   {
      this.maxKdRate.set(maxKdRate);
   }

   @Override
   public double getKp()
   {
      return limitedKp.getDoubleValue();
   }

   @Override
   public double getKd()
   {
      return limitedKd.getDoubleValue();
   }

   @Override
   public DoubleYoVariable getYoKp()
   {
      return limitedKp;
   }

   @Override
   public DoubleYoVariable getYoKd()
   {
      return limitedKd;
   }

   @Override
   public DoubleYoVariable getYoKi()
   {
      return limitedKi;
   }

   public RateLimitedYoVariable getLimitedYoKp()
   {
      return limitedKp;
   }

   public RateLimitedYoVariable getLimitedYoKd()
   {
      return limitedKd;
   }

   public RateLimitedYoVariable getLimitedYoKi()
   {
      return limitedKi;
   }
}
