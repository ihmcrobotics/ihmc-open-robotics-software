package us.ihmc.sensorProcessing.sensorProcessors;

public class OneDoFJointState implements OneDoFJointStateReadOnly
{
   private final String jointName;
   private double position;
   private double velocity;
   private double acceleration;
   private double effort;
   private boolean isJointEnabled;

   public OneDoFJointState(String jointName)
   {
      this.jointName = jointName;
   }

   public void setPosition(double position)
   {
      this.position = position;
   }

   public void setVelocity(double velocity)
   {
      this.velocity = velocity;
   }

   public void setAcceleration(double acceleration)
   {
      this.acceleration = acceleration;
   }

   public void setEffort(double effort)
   {
      this.effort = effort;
   }

   public void setJointEnabled(boolean isJointEnabled)
   {
      this.isJointEnabled = isJointEnabled;
   }

   @Override
   public String getJointName()
   {
      return jointName;
   }

   @Override
   public double getPosition()
   {
      return position;
   }

   @Override
   public double getVelocity()
   {
      return velocity;
   }

   @Override
   public double getAcceleration()
   {
      return acceleration;
   }

   @Override
   public double getEffort()
   {
      return effort;
   }

   @Override
   public boolean isJointEnabled()
   {
      return isJointEnabled;
   }
}
