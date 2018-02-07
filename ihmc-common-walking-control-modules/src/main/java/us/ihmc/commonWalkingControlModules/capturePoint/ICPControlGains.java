package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.robotics.controllers.pidGains.implementations.IntegratorGains;

public class ICPControlGains extends IntegratorGains implements ICPControlGainsReadOnly
{
   private double kpParallelToMotion;
   private double kpOrthogonalToMotion;

   private double feedbackPartMaxRate = Double.POSITIVE_INFINITY;

   @Override
   public double getKpParallelToMotion()
   {
      return kpParallelToMotion;
   }

   public void setKpParallelToMotion(double kpParallelToMotion)
   {
      this.kpParallelToMotion = kpParallelToMotion;
   }

   @Override
   public double getKpOrthogonalToMotion()
   {
      return kpOrthogonalToMotion;
   }

   public void setKpOrthogonalToMotion(double kpOrthogonalToMotion)
   {
      this.kpOrthogonalToMotion = kpOrthogonalToMotion;
   }

   @Override
   public double getFeedbackPartMaxRate()
   {
      return feedbackPartMaxRate;
   }

   public void setFeedbackPartMaxRate(double feedbackPartMaxRate)
   {
      this.feedbackPartMaxRate = feedbackPartMaxRate;
   }
}
