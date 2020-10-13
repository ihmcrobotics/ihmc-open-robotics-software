package us.ihmc.sensorProcessing.outputData;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoLowLevelState implements LowLevelStateReadOnly
{
   private YoDouble position;
   private YoDouble velocity;
   private YoDouble acceleration;
   private YoDouble effort;
   private YoBoolean isPositionValid;
   private YoBoolean isVelocityValid;
   private YoBoolean isAccelerationValid;
   private YoBoolean isEffortValid;

   public YoLowLevelState(String namePrefix, YoRegistry registry)
   {
      this.position = new YoDouble(namePrefix + "_DesiredPosition", registry);
      this.velocity = new YoDouble(namePrefix + "_DesiredVelocity", registry);
      this.acceleration = new YoDouble(namePrefix + "_DesiredAcceleration", registry);
      this.effort = new YoDouble(namePrefix + "_DesiredEffort", registry);
      this.isPositionValid = new YoBoolean(namePrefix + "_IsPositionValid", registry);
      this.isVelocityValid = new YoBoolean(namePrefix + "_IsVelocityValid", registry);
      this.isAccelerationValid = new YoBoolean(namePrefix + "_IsAccelerationValid", registry);
      this.isEffortValid = new YoBoolean(namePrefix + "_IsEffortValid", registry);
   }

   public void set(LowLevelStateReadOnly other)
   {
      clear();
      if (other.isPositionValid())
         this.setPosition(other.getPosition());
      if (other.isVelocityValid())
         this.setVelocity(other.getVelocity());
      if (other.isAccelerationValid())
         this.setAcceleration(other.getAcceleration());
      if (other.isEffortValid())
         this.setEffort(other.getEffort());
   }

   public void clear()
   {
      this.isPositionValid.set(false);
      this.isVelocityValid.set(false);
      this.isAccelerationValid.set(false);
      this.isEffortValid.set(false);
   }

   public void setPosition(double position)
   {
      this.position.set(position);
      this.isPositionValid.set(true);
   }

   public void setVelocity(double velocity)
   {
      this.velocity.set(velocity);
      this.isVelocityValid.set(true);
   }

   public void setAcceleration(double acceleration)
   {
      this.acceleration.set(acceleration);
      this.isAccelerationValid.set(true);
   }

   public void setEffort(double effort)
   {
      this.effort.set(effort);
      this.isEffortValid.set(true);
   }

   @Override
   public double getPosition()
   {
      if (!isPositionValid())
      {
         throw new RuntimeException("Position data is invalid.");
      }
      return position.getDoubleValue();
   }

   @Override
   public double getVelocity()
   {
      if (!isVelocityValid())
      {
         throw new RuntimeException("Velocity data is invalid.");
      }
      return velocity.getDoubleValue();
   }

   @Override
   public double getAcceleration()
   {
      if (!isAccelerationValid())
      {
         throw new RuntimeException("Acceleration data is invalid.");
      }
      return acceleration.getDoubleValue();
   }

   @Override
   public double getEffort()
   {
      if (!isEffortValid())
      {
         throw new RuntimeException("Effort data is invalid.");
      }
      return effort.getDoubleValue();
   }

   public void setPositionValid(boolean isPositionValid)
   {
      this.isPositionValid.set(isPositionValid);
   }

   public void setVelocityValid(boolean isVelocityValid)
   {
      this.isVelocityValid.set(isVelocityValid);
   }

   public void setAccelerationValid(boolean isAccelerationValid)
   {
      this.isAccelerationValid.set(isAccelerationValid);
   }

   public void setEffortValid(boolean isEffortValid)
   {
      this.isEffortValid.set(isEffortValid);
   }

   @Override
   public boolean isPositionValid()
   {
      return isPositionValid.getBooleanValue();
   }

   @Override
   public boolean isVelocityValid()
   {
      return isVelocityValid.getBooleanValue();
   }

   @Override
   public boolean isAccelerationValid()
   {
      return isAccelerationValid.getBooleanValue();
   }

   @Override
   public boolean isEffortValid()
   {
      return isEffortValid.getBooleanValue();
   }
}
