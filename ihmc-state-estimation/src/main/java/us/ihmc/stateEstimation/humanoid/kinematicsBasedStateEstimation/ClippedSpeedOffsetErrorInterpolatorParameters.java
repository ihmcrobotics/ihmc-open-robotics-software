package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

public class ClippedSpeedOffsetErrorInterpolatorParameters
{
   private boolean isRotationCorrectionEnabled = true;

   private double breakFrequency = 0.6;

   private double zDeadzoneSize = 0.014;
   private double yDeadzoneSize = 0.014;
   private double xDeadzoneSize = 0.014;
   private double yawDeadzoneInDegrees = 1.0;

   private double maxTranslationalCorrectionSpeed = 0.05;
   private double maxRotationalCorrectionSpeed = 0.05;

   public boolean getIsRotationCorrectionEnabled()
   {
      return isRotationCorrectionEnabled;
   }

   public void setIsRotationCorrectionEnabled(boolean isRotationCorrectionEnabled)
   {
      this.isRotationCorrectionEnabled = isRotationCorrectionEnabled;
   }

   public double getBreakFrequency()
   {
      return breakFrequency;
   }

   public void setBreakFrequency(double breakFrequency)
   {
      this.breakFrequency = breakFrequency;
   }

   public double getZDeadzoneSize()
   {
      return zDeadzoneSize;
   }

   public void setZDeadzoneSize(double zDeadzoneSize)
   {
      this.zDeadzoneSize = zDeadzoneSize;
   }

   public double getYDeadzoneSize()
   {
      return yDeadzoneSize;
   }

   public void setYDeadzoneSize(double yDeadzoneSize)
   {
      this.yDeadzoneSize = yDeadzoneSize;
   }

   public double getXDeadzoneSize()
   {
      return xDeadzoneSize;
   }

   public void setXDeadzoneSize(double xDeadzoneSize)
   {
      this.xDeadzoneSize = xDeadzoneSize;
   }

   public double getYawDeadzoneInDegrees()
   {
      return yawDeadzoneInDegrees;
   }

   public void setYawDeadzoneInDegrees(double yawDeadzoneInDegrees)
   {
      this.yawDeadzoneInDegrees = yawDeadzoneInDegrees;
   }

   public double getMaxTranslationalCorrectionSpeed()
   {
      return maxTranslationalCorrectionSpeed;
   }

   public void setMaxTranslationalCorrectionSpeed(double maxTranslationalCorrectionSpeed)
   {
      this.maxTranslationalCorrectionSpeed = maxTranslationalCorrectionSpeed;
   }

   public double getMaxRotationalCorrectionSpeed()
   {
      return maxRotationalCorrectionSpeed;
   }

   public void setMaxRotationalCorrectionSpeed(double maxRotationalCorrectionSpeed)
   {
      this.maxRotationalCorrectionSpeed = maxRotationalCorrectionSpeed;
   }

   public void setDeadZoneSizes(double xDeadzoneSize, double yDeadzoneSize, double zDeadzoneSize, double yawDeadzoneSize)
   {
      setXDeadzoneSize(xDeadzoneSize);
      setYDeadzoneSize(yDeadzoneSize);
      setZDeadzoneSize(zDeadzoneSize);
      setYawDeadzoneInDegrees(yawDeadzoneSize);
      
   }

   public void setMaximumTranslationAndRotationSpeed(double maxTranslationalCorrectionSpeed, double maxRotationalCorrectionSpeed)
   {
      setMaxTranslationalCorrectionSpeed(maxTranslationalCorrectionSpeed);
      setMaxRotationalCorrectionSpeed(maxRotationalCorrectionSpeed);
   }

}
