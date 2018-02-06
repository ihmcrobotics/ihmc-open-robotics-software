package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.robotics.controllers.pidGains.implementations.IntegratorGains;

public class ICPControlGains extends IntegratorGains
{
   private double kpParallelToMotion;
   private double kpOrthogonalToMotion;

   private double feedbackPartMaxRate;

   public double getKpParallelToMotion()
   {
      return kpParallelToMotion;
   }

   public void setKpParallelToMotion(double kpParallelToMotion)
   {
      this.kpParallelToMotion = kpParallelToMotion;
   }

   public double getKpOrthogonalToMotion()
   {
      return kpOrthogonalToMotion;
   }

   public void setKpOrthogonalToMotion(double kpOrthogonalToMotion)
   {
      this.kpOrthogonalToMotion = kpOrthogonalToMotion;
   }

   public double getFeedbackPartMaxRate()
   {
      return feedbackPartMaxRate;
   }

   public void setFeedbackPartMaxRate(double feedbackPartMaxRate)
   {
      this.feedbackPartMaxRate = feedbackPartMaxRate;
   }
}
