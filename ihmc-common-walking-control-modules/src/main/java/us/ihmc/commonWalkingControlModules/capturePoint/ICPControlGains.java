package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.robotics.controllers.pidGains.implementations.IntegratorGains;

public class ICPControlGains extends IntegratorGains implements ICPControlGainsReadOnly
{
   private double kpParallelToMotion;
   private double kpOrthogonalToMotion;

   private double feedbackPartMaxRate = Double.POSITIVE_INFINITY;

   private double feedbackPartMaxValueParallelToMotion = Double.POSITIVE_INFINITY;
   private double feedbackPartMaxValueOrthogonalToMotion = Double.POSITIVE_INFINITY;

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

   @Override
   public double getFeedbackPartMaxValueParallelToMotion()
   {
      return feedbackPartMaxValueParallelToMotion;
   }

   public void setFeedbackPartMaxValueParallelToMotion(double feedbackPartMaxValueParallelToMotion)
   {
      this.feedbackPartMaxValueParallelToMotion = feedbackPartMaxValueParallelToMotion;
   }

   @Override
   public double getFeedbackPartMaxValueOrthogonalToMotion()
   {
      return feedbackPartMaxValueOrthogonalToMotion;
   }

   public void setFeedbackPartMaxValueOrthogonalToMotion(double feedbackPartMaxValueOrthogonalToMotion)
   {
      this.feedbackPartMaxValueOrthogonalToMotion = feedbackPartMaxValueOrthogonalToMotion;
   }
}
