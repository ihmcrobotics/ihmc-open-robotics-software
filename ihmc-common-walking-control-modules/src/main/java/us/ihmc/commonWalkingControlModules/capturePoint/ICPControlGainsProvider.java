package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.yoVariables.providers.DoubleProvider;

public interface ICPControlGainsProvider extends ICPControlGainsReadOnly
{
   DoubleProvider getYoKpParallelToMotion();

   DoubleProvider getYoKpOrthogonalToMotion();

   DoubleProvider getYoKi();

   DoubleProvider getYoIntegralLeakRatio();

   DoubleProvider getYoMaxIntegralError();

   DoubleProvider getYoFeedbackPartMaxRate();

   @Override
   default double getKpParallelToMotion()
   {
      return getYoKpParallelToMotion().getValue();
   }

   @Override
   default double getKpOrthogonalToMotion()
   {
      return getYoKpOrthogonalToMotion().getValue();
   }

   @Override
   default double getKi()
   {
      return getYoKi().getValue();
   }

   @Override
   default double getIntegralLeakRatio()
   {
      return getYoIntegralLeakRatio().getValue();
   }

   @Override
   default double getMaxIntegralError()
   {
      return getYoMaxIntegralError().getValue();
   }

   @Override
   default double getFeedbackPartMaxRate()
   {
      return getYoFeedbackPartMaxRate().getValue();
   }
}
