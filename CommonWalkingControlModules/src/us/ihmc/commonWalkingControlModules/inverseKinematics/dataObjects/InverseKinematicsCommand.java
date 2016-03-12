package us.ihmc.commonWalkingControlModules.inverseKinematics.dataObjects;

public interface InverseKinematicsCommand<T extends InverseKinematicsCommand<T>>
{
   public enum InverseKinematicsCommandWeightLevels
   {
      HIGH, MEDIUM, LOW, HARD_CONSTRAINT;

      public double getWeightValue()
      {
         switch (this)
         {
         case HARD_CONSTRAINT:
            return Double.POSITIVE_INFINITY;
         case HIGH:
            return 10.0;
         case LOW:
            return 0.10;
         case MEDIUM:
         default:
            return 1.00;
         }
      }
   }

   public enum InverseKinematicsCommandType
   {
      JOINTSPACE, TASKSPACE, MOMENTUM,
      PRIVILEGED_CONFIGURATION,
      COMMAND_LIST
   };

   public abstract void set(T other);

   public abstract InverseKinematicsCommandType getCommandType();

   public abstract void setWeight(double weight);

   public abstract void setWeightLevel(InverseKinematicsCommandWeightLevels weightLevel);

   public abstract boolean isHardConstraint();
}
