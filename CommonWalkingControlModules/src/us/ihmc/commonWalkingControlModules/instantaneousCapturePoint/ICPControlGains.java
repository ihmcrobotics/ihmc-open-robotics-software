package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.DoubleYoVariable;

public class ICPControlGains
{
   private final String suffix;
   private final YoVariableRegistry registry;

   private final DoubleYoVariable kpParallelToMotion;
   private final DoubleYoVariable kpOrthogonalToMotion;
   private final DoubleYoVariable ki;
   private final DoubleYoVariable kiBleedOff;
   private DoubleYoVariable feedbackPartMaxRate;

   public ICPControlGains(String suffix, YoVariableRegistry registry)
   {
      this.suffix = suffix;
      this.registry = registry;

      kpParallelToMotion = new DoubleYoVariable("captureKpParallel" + suffix, registry);
      kpOrthogonalToMotion = new DoubleYoVariable("captureKpOrthogonal" + suffix, registry);
      ki = new DoubleYoVariable("captureKi" + suffix, registry);
      kiBleedOff = new DoubleYoVariable("captureKiBleedOff" + suffix, registry);
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
         feedbackPartMaxRate = new DoubleYoVariable("feedbackPartMaxRate" + suffix, registry);
      feedbackPartMaxRate.set(maxRate);
   }

   public DoubleYoVariable getYoKpParallelToMotion()
   {
      return kpParallelToMotion;
   }

   public DoubleYoVariable getYoKpOrthogonalToMotion()
   {
      return kpOrthogonalToMotion;
   }

   public DoubleYoVariable getYoKi()
   {
      return ki;
   }

   public DoubleYoVariable getYoKiBleedOff()
   {
      return kiBleedOff;
   }

   public DoubleYoVariable getFeedbackPartMaxRate()
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
}
