package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ParameterizedICPControlGains implements ICPControlGainsProvider
{

   private final DoubleProvider kpParallelToMotion;
   private final DoubleProvider kpOrthogonalToMotion;
   private final DoubleProvider ki;
   private final DoubleProvider integralLeakRatio;
   private final DoubleProvider maxIntegralError;
   private final DoubleProvider feedbackPartMaxRate;

   public ParameterizedICPControlGains(String suffix, boolean rateLimitFeedbackPart, YoVariableRegistry registry)
   {
      kpParallelToMotion = new DoubleParameter("captureKpParallel" + suffix, registry);
      kpOrthogonalToMotion = new DoubleParameter("captureKpOrthogonal" + suffix, registry);
      ki = new DoubleParameter("captureKi" + suffix, registry);
      integralLeakRatio = new DoubleParameter("captureIntegralLeakRatio" + suffix, registry, 1.0);
      maxIntegralError = new DoubleParameter("captureMaxIntegralError" + suffix, registry, Double.POSITIVE_INFINITY);
      if (rateLimitFeedbackPart)
      {
         feedbackPartMaxRate = new DoubleParameter("feedbackPartMaxRate" + suffix, registry);
      }
      else
      {
         feedbackPartMaxRate = null;
      }
   }

   public ParameterizedICPControlGains(String suffix, ICPControlGainsReadOnly defaults, YoVariableRegistry registry)
   {
      kpParallelToMotion = new DoubleParameter("captureKpParallel" + suffix, registry, defaults.getKpParallelToMotion());
      kpOrthogonalToMotion = new DoubleParameter("captureKpOrthogonal" + suffix, registry, defaults.getKpOrthogonalToMotion());
      ki = new DoubleParameter("captureKi" + suffix, registry, defaults.getKi());
      integralLeakRatio = new DoubleParameter("captureIntegralLeakRatio" + suffix, registry, defaults.getIntegralLeakRatio());
      maxIntegralError = new DoubleParameter("captureMaxIntegralError" + suffix, registry, defaults.getMaxIntegralError());
      feedbackPartMaxRate = new DoubleParameter("feedbackPartMaxRate" + suffix, registry, defaults.getFeedbackPartMaxRate());
   }

   @Override
   public DoubleProvider getYoKpParallelToMotion()
   {
      return kpParallelToMotion;
   }

   @Override
   public DoubleProvider getYoKpOrthogonalToMotion()
   {
      return kpOrthogonalToMotion;
   }

   @Override
   public DoubleProvider getYoKi()
   {
      return ki;
   }

   @Override
   public DoubleProvider getYoIntegralLeakRatio()
   {
      return integralLeakRatio;
   }

   @Override
   public DoubleProvider getYoMaxIntegralError()
   {
      return maxIntegralError;
   }

   @Override
   public DoubleProvider getYoFeedbackPartMaxRate()
   {
      return feedbackPartMaxRate;
   }

}
