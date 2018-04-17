package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.robotics.controllers.pidGains.IntegratorGainsReadOnly;

public interface ICPControlGainsReadOnly extends IntegratorGainsReadOnly
{
   double getKpParallelToMotion();

   double getKpOrthogonalToMotion();

   default double getFeedbackPartMaxRate()
   {
      return Double.POSITIVE_INFINITY;
   }
}
