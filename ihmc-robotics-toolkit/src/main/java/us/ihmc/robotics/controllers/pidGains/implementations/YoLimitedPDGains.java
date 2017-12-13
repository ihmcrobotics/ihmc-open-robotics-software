package us.ihmc.robotics.controllers.pidGains.implementations;

import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoLimitedPDGains extends YoPDGains
{
   private final RateLimitedYoVariable limitedKp;
   private final RateLimitedYoVariable limitedKd;

   private final YoDouble maxKpRate;
   private final YoDouble maxKdRate;

   public YoLimitedPDGains(String suffix, double controlDT, YoVariableRegistry registry)
   {
      super(suffix, registry);

      maxKpRate = new YoDouble("maxKpRate" + suffix, registry);
      maxKdRate = new YoDouble("maxKdRate" + suffix, registry);

      limitedKp = new RateLimitedYoVariable("limitedKp" + suffix, registry, maxKpRate, super.getYoKp(), controlDT);
      limitedKd = new RateLimitedYoVariable("limitedKd" + suffix, registry, maxKdRate, super.getYoKd(), controlDT);

      maxKpRate.set(Double.POSITIVE_INFINITY);
      maxKdRate.set(Double.POSITIVE_INFINITY);
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
   public YoDouble getYoKp()
   {
      return limitedKp;
   }

   @Override
   public YoDouble getYoKd()
   {
      return limitedKd;
   }
}