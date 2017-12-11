package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.yoVariables.providers.DoubleProvider;

public interface ICPControlGainsProvider
{

   DoubleProvider getYoKpParallelToMotion();

   DoubleProvider getYoKpOrthogonalToMotion();

   DoubleProvider getYoKi();

   DoubleProvider getYoKiBleedOff();

   DoubleProvider getFeedbackPartMaxRate();

}
