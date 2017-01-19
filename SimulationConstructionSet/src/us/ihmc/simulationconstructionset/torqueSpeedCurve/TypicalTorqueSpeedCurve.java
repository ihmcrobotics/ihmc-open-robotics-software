package us.ihmc.simulationconstructionset.torqueSpeedCurve;

import us.ihmc.robotics.MathTools;

public class TypicalTorqueSpeedCurve implements TorqueSpeedCurve
{
   private double maxSpeed, maxTorque, maxSpeedAtMaxTorque;

   public TypicalTorqueSpeedCurve()
   {
   }

   public TypicalTorqueSpeedCurve(double maxSpeed, double maxTorque, double maxSpeedAtMaxTorque)
   {
      this.maxSpeed = maxSpeed;
      this.maxTorque = maxTorque;
      this.maxSpeedAtMaxTorque = maxSpeedAtMaxTorque;
   }

   public void setMaxSpeed(double maxSpeed)
   {
      this.maxSpeed = maxSpeed;
   }

   public void setMaxTorque(double maxTorque)
   {
      this.maxTorque = maxTorque;
   }

   public void setMaxSpeedAtMaxTorque(double maxSpeedAtMaxTorque)
   {
      this.maxSpeedAtMaxTorque = maxSpeedAtMaxTorque;
   }

   @Override
   public double limitTorque(double torque, double speed)
   {
      if ((speed < 0.0) && (torque > 0.0))
      {
         return Math.min(torque, maxTorque);
      }

      if ((speed > 0.0) && (torque < 0.0))
      {
         return Math.max(torque, -maxTorque);
      }

      double absoluteSpeed = Math.abs(speed);
      if (absoluteSpeed > maxSpeed)
         return 0.0;

      if (absoluteSpeed < maxSpeedAtMaxTorque)
      {
         return MathTools.clipToMinMax(torque, -maxTorque, maxTorque);
      }

      double percent = 1.0 - (absoluteSpeed - maxSpeedAtMaxTorque) / (maxSpeed - maxSpeedAtMaxTorque);
      double maxScaledTorque = percent * maxTorque;

      return MathTools.clipToMinMax(torque, -maxScaledTorque, maxScaledTorque);
   }

}
