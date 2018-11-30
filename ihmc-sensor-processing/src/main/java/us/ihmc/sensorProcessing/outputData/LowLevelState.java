package us.ihmc.sensorProcessing.outputData;

public class LowLevelState implements LowLevelStateReadOnly
{
   private double position;
   private double velocity;
   private double acceleration;
   private double effort;
   private boolean isPositionValid;
   private boolean isVelocityValid;
   private boolean isAccelerationValid;
   private boolean isEffortValid;

   public LowLevelState()
   {
      this.position = 0.0;
      this.velocity = 0.0;
      this.effort = 0.0;
      this.isPositionValid = false;
      this.isVelocityValid = false;
      this.isAccelerationValid = false;
      this.isEffortValid = false;
   }

   public LowLevelState(double position, double velocity, double acceleration, double effort)
   {
      this.position = position;
      this.velocity = velocity;
      this.acceleration = acceleration;
      this.effort = effort;
      this.isPositionValid = true;
      this.isVelocityValid = true;
      this.isAccelerationValid = true;
      this.isEffortValid = true;
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
      this.isPositionValid = false;
      this.isVelocityValid = false;
      this.isAccelerationValid = false;
      this.isEffortValid = false;
   }

   public void setPosition(double position)
   {
      this.position = position;
      this.isPositionValid = true;
   }

   public void setVelocity(double velocity)
   {
      this.velocity = velocity;
      this.isVelocityValid = true;
   }

   public void setAcceleration(double acceleration)
   {
      this.acceleration = acceleration;
      this.isAccelerationValid = true;
   }

   public void setEffort(double effort)
   {
      this.effort = effort;
      this.isEffortValid = true;
   }

   @Override
   public double getPosition()
   {
      if (!isPositionValid())
      {
         throw new RuntimeException("Position data is invalid.");
      }
      return position;
   }

   @Override
   public double getVelocity()
   {
      if (!isVelocityValid())
      {
         throw new RuntimeException("Velocity data is invalid.");
      }
      return velocity;
   }

   @Override
   public double getAcceleration()
   {
      if (!isAccelerationValid())
      {
         throw new RuntimeException("Acceleration data is invalid.");
      }
      return acceleration;
   }

   @Override
   public double getEffort()
   {
      if (!isEffortValid())
      {
         throw new RuntimeException("Effort data is invalid.");
      }
      return effort;
   }

   public void setPositionValid(boolean isPositionValid)
   {
      this.isPositionValid = isPositionValid;
   }

   public void setVelocityValid(boolean isVelocityValid)
   {
      this.isVelocityValid = isVelocityValid;
   }

   public void setAccelerationValid(boolean isAccelerationValid)
   {
      this.isAccelerationValid = isAccelerationValid;
   }

   public void setEffortValid(boolean isEffortValid)
   {
      this.isEffortValid = isEffortValid;
   }

   @Override
   public boolean isPositionValid()
   {
      return isPositionValid;
   }

   @Override
   public boolean isVelocityValid()
   {
      return isVelocityValid;
   }

   @Override
   public boolean isAccelerationValid()
   {
      return isAccelerationValid;
   }

   @Override
   public boolean isEffortValid()
   {
      return isEffortValid;
   }
}
