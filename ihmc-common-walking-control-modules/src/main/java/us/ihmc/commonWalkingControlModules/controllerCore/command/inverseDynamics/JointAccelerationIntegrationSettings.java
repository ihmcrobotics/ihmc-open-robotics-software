package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

public class JointAccelerationIntegrationSettings
{
   private double alphaPosition = Double.NaN;
   private double alphaVelocity = Double.NaN;
   private double maxPositionError = Double.NaN;
   private double maxVelocity = Double.NaN;

   public JointAccelerationIntegrationSettings()
   {
   }

   public JointAccelerationIntegrationSettings(double alphaPosition, double alphaVelocity, double maxPositionError, double maxVelocity)
   {
      this.alphaPosition = alphaPosition;
      this.alphaVelocity = alphaVelocity;
      this.maxPositionError = maxPositionError;
      this.maxVelocity = maxVelocity;
   }

   public double getAlphaPosition()
   {
      return alphaPosition;
   }

   public void setAlphaPosition(double alphaPosition)
   {
      this.alphaPosition = alphaPosition;
   }

   public double getAlphaVelocity()
   {
      return alphaVelocity;
   }

   public void setAlphaVelocity(double alphaVelocity)
   {
      this.alphaVelocity = alphaVelocity;
   }

   public double getMaxPositionError()
   {
      return maxPositionError;
   }

   public void setMaxPositionError(double maxPositionError)
   {
      this.maxPositionError = maxPositionError;
   }

   public double getMaxVelocity()
   {
      return maxVelocity;
   }

   public void setMaxVelocity(double maxVelocity)
   {
      this.maxVelocity = maxVelocity;
   }
}
