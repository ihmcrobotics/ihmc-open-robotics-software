package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

public class ICPControlGains
{
   private double kpParallelToMotion;
   private double kpOrthogonalToMotion;
   private double ki;
   private double kiBleedOff = 1.0;

   private boolean useRawCMP = false;
   private boolean useHackToReduceFeedForward = true;

   private double cmpFilterBreakFrequencyInHertz = Double.POSITIVE_INFINITY;
   private double cmpRateLimit = Double.POSITIVE_INFINITY;
   private double cmpAccelerationLimit = Double.POSITIVE_INFINITY;

   public ICPControlGains()
   {
   }

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

   public double getKi()
   {
      return ki;
   }

   public void setKi(double ki)
   {
      this.ki = ki;
   }

   public double getKiBleedOff()
   {
      return kiBleedOff;
   }

   public void setKiBleedOff(double kiBleedOff)
   {
      this.kiBleedOff = kiBleedOff;
   }

   public boolean useRawCMP()
   {
      return useRawCMP;
   }

   public void setUseRawCMP(boolean useRawCMP)
   {
      this.useRawCMP = useRawCMP;
   }

   public double getCMPFilterBreakFrequencyInHertz()
   {
      return cmpFilterBreakFrequencyInHertz;
   }

   public void setCMPFilterBreakFrequencyInHertz(double cmpFilterBreakFrequencyInHertz)
   {
      this.cmpFilterBreakFrequencyInHertz = cmpFilterBreakFrequencyInHertz;
   }

   public double getCMPRateLimit()
   {
      return cmpRateLimit;
   }

   public void setCMPRateLimit(double cmpRateLimit)
   {
      this.cmpRateLimit = cmpRateLimit;
   }

   public double getCMPAccelerationLimit()
   {
      return cmpAccelerationLimit;
   }

   public void setCMPAccelerationLimit(double cmpAccelerationLimit)
   {
      this.cmpAccelerationLimit = cmpAccelerationLimit;
   }

   public boolean useHackToReduceFeedForward()
   {
      return useHackToReduceFeedForward;
   }

   public void setUseHackToReduceFeedForward(boolean useHackToReduceFeedForward)
   {
      this.useHackToReduceFeedForward = useHackToReduceFeedForward;
   }
}
