package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoICPControlGains implements ICPControlGainsProvider
{
   private final String suffix;
   private final YoVariableRegistry registry;

   private final YoDouble kpParallelToMotion;
   private final YoDouble kpOrthogonalToMotion;
   private final YoDouble ki;
   private final YoDouble kiBleedOff;
   private YoDouble feedbackPartMaxRate;

   public YoICPControlGains(String suffix, YoVariableRegistry registry)
   {
      this.suffix = suffix;
      this.registry = registry;

      kpParallelToMotion = new YoDouble("captureKpParallel" + suffix, registry);
      kpOrthogonalToMotion = new YoDouble("captureKpOrthogonal" + suffix, registry);
      ki = new YoDouble("captureKi" + suffix, registry);
      kiBleedOff = new YoDouble("captureKiBleedOff" + suffix, registry);
      kiBleedOff.set(1.0);
   }

   public void setKpParallelToMotion(double kpParallelToMotion)
   {
      this.kpParallelToMotion.set(kpParallelToMotion);
   }

   public void setKpOrthogonalToMotion(double kpOrthogonalToMotion)
   {
      this.kpOrthogonalToMotion.set(kpOrthogonalToMotion);
   }

   public void setKi(double ki)
   {
      this.ki.set(ki);
   }

   public void setKiBleedOff(double kiBleedOff)
   {
      this.kiBleedOff.set(kiBleedOff);
   }

   public void setFeedbackPartMaxRate(double maxRate)
   {
      if (feedbackPartMaxRate == null)
         feedbackPartMaxRate = new YoDouble("feedbackPartMaxRate" + suffix, registry);
      feedbackPartMaxRate.set(maxRate);
   }

   public YoDouble getYoKpParallelToMotion()
   {
      return kpParallelToMotion;
   }

   public YoDouble getYoKpOrthogonalToMotion()
   {
      return kpOrthogonalToMotion;
   }

   public YoDouble getYoKi()
   {
      return ki;
   }

   public YoDouble getYoKiBleedOff()
   {
      return kiBleedOff;
   }

   public YoDouble getFeedbackPartMaxRate()
   {
      return feedbackPartMaxRate;
   }

   public double getKpParallelToMotion()
   {
      return kpParallelToMotion.getDoubleValue();
   }

   public double getKpOrthogonalToMotion()
   {
      return kpOrthogonalToMotion.getDoubleValue();
   }

   public double getKi()
   {
      return ki.getDoubleValue();
   }

   public double getKiBleedOff()
   {
      return kiBleedOff.getDoubleValue();
   }

   public void set(ICPControlGains icpControlGains)
   {
      setKpParallelToMotion(icpControlGains.getKpParallelToMotion());
      setKpOrthogonalToMotion(icpControlGains.getKpOrthogonalToMotion());
      setKi(icpControlGains.getKi());
      setKiBleedOff(icpControlGains.getKiBleedOff());
      setFeedbackPartMaxRate(icpControlGains.getFeedbackPartMaxRate());
   }
}
