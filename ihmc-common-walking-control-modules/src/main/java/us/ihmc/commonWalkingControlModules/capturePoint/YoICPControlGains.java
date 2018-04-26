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
   private final YoDouble integralLeakRatio;
   private final YoDouble maxIntegralError;
   private YoDouble feedbackPartMaxRate;

   public YoICPControlGains(String suffix, YoVariableRegistry registry)
   {
      this.suffix = suffix;
      this.registry = registry;

      kpParallelToMotion = new YoDouble("captureKpParallel" + suffix, registry);
      kpOrthogonalToMotion = new YoDouble("captureKpOrthogonal" + suffix, registry);
      ki = new YoDouble("captureKi" + suffix, registry);
      integralLeakRatio = new YoDouble("captureIntegralLeakRatio" + suffix, registry);
      integralLeakRatio.set(1.0);
      maxIntegralError = new YoDouble("captureMaxIntegralError" + suffix, registry);
      maxIntegralError.set(Double.POSITIVE_INFINITY);
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

   public void setIntegralLeakRatio(double integralLeakRatio)
   {
      this.integralLeakRatio.set(integralLeakRatio);
   }

   public void setMaxIntegralError(double maxIntegralError)
   {
      this.maxIntegralError.set(maxIntegralError);
   }

   public void setFeedbackPartMaxRate(double maxRate)
   {
      if (feedbackPartMaxRate == null)
         feedbackPartMaxRate = new YoDouble("feedbackPartMaxRate" + suffix, registry);
      feedbackPartMaxRate.set(maxRate);
   }

   @Override
   public YoDouble getYoKpParallelToMotion()
   {
      return kpParallelToMotion;
   }

   @Override
   public YoDouble getYoKpOrthogonalToMotion()
   {
      return kpOrthogonalToMotion;
   }

   @Override
   public YoDouble getYoKi()
   {
      return ki;
   }

   @Override
   public YoDouble getYoIntegralLeakRatio()
   {
      return integralLeakRatio;
   }

   @Override
   public YoDouble getYoMaxIntegralError()
   {
      return maxIntegralError;
   }

   @Override
   public YoDouble getYoFeedbackPartMaxRate()
   {
      return feedbackPartMaxRate;
   }

   public void set(ICPControlGainsReadOnly icpControlGains)
   {
      setKpParallelToMotion(icpControlGains.getKpParallelToMotion());
      setKpOrthogonalToMotion(icpControlGains.getKpOrthogonalToMotion());
      setKi(icpControlGains.getKi());
      setIntegralLeakRatio(icpControlGains.getIntegralLeakRatio());
      setMaxIntegralError(icpControlGains.getMaxIntegralError());
      setFeedbackPartMaxRate(icpControlGains.getFeedbackPartMaxRate());
   }
}
