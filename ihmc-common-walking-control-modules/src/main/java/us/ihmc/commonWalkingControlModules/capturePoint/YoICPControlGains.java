package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoICPControlGains implements ICPControlGainsProvider
{
   private final YoDouble kpParallelToMotion;
   private final YoDouble kpOrthogonalToMotion;
   private final YoDouble ki;
   private final YoDouble integralLeakRatio;
   private final YoDouble maxIntegralError;
   private final YoDouble feedbackPartMaxRate;
   private final YoDouble feedbackPartMaxValueParallelToMotion;
   private final YoDouble feedbackPartMaxValueOrthogonalToMotion;

   public YoICPControlGains(String suffix, YoRegistry registry)
   {
      kpParallelToMotion = new YoDouble("captureKpParallel" + suffix, registry);
      kpOrthogonalToMotion = new YoDouble("captureKpOrthogonal" + suffix, registry);
      ki = new YoDouble("captureKi" + suffix, registry);
      integralLeakRatio = new YoDouble("captureIntegralLeakRatio" + suffix, registry);
      maxIntegralError = new YoDouble("captureMaxIntegralError" + suffix, registry);
      feedbackPartMaxRate = new YoDouble("feedbackPartMaxRate" + suffix, registry);
      feedbackPartMaxValueParallelToMotion = new YoDouble("feedbackPartMaxValueParallelToMotion" + suffix, registry);
      feedbackPartMaxValueOrthogonalToMotion = new YoDouble("feedbackPartMaxValueOrthogonalToMotion" + suffix, registry);

      integralLeakRatio.set(1.0);
      maxIntegralError.set(Double.POSITIVE_INFINITY);
      feedbackPartMaxRate.set(Double.POSITIVE_INFINITY);
      feedbackPartMaxValueParallelToMotion.set(Double.POSITIVE_INFINITY);
      feedbackPartMaxValueOrthogonalToMotion.set(Double.POSITIVE_INFINITY);
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
      feedbackPartMaxRate.set(maxRate);
   }

   public void setFeedbackPartMaxValueParallelToMotion(double maxValue)
   {
      feedbackPartMaxValueParallelToMotion.set(maxValue);
   }

   public void setFeedbackPartMaxValueOrthogonalToMotion(double maxValue)
   {
      feedbackPartMaxValueOrthogonalToMotion.set(maxValue);
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

   @Override
   public YoDouble getYoFeedbackPartMaxValueParallelToMotion()
   {
      return feedbackPartMaxValueParallelToMotion;
   }

   @Override
   public YoDouble getYoFeedbackPartMaxValueOrthogonalToMotion()
   {
      return feedbackPartMaxValueOrthogonalToMotion;
   }

   public void set(ICPControlGainsReadOnly icpControlGains)
   {
      setKpParallelToMotion(icpControlGains.getKpParallelToMotion());
      setKpOrthogonalToMotion(icpControlGains.getKpOrthogonalToMotion());
      setKi(icpControlGains.getKi());
      setIntegralLeakRatio(icpControlGains.getIntegralLeakRatio());
      setMaxIntegralError(icpControlGains.getMaxIntegralError());
      setFeedbackPartMaxRate(icpControlGains.getFeedbackPartMaxRate());
      setFeedbackPartMaxValueParallelToMotion(icpControlGains.getFeedbackPartMaxValueParallelToMotion());
      setFeedbackPartMaxValueOrthogonalToMotion(icpControlGains.getFeedbackPartMaxValueOrthogonalToMotion());
   }
}
