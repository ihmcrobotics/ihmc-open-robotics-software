package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

public interface InverseDynamicsCommand<T extends InverseDynamicsCommand<T>>
{
   public enum InverseDynamicsCommandWeightLevels
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

   public enum InverseDynamicsCommandType
   {
      TASKSPACE_MOTION, TASKSPACE_POINT_MOTION, JOINTSPACE_MOTION, MOMENTUM_RATE,
      PRIVILEGED_CONFIGURATION,
      EXTERNAL_WRENCH, PLANE_CONTACT_STATE,
      COMMAND_LIST, PLANE_CONTACT_STATE_POOL
   };

   public abstract void set(T other);

   public abstract InverseDynamicsCommandType getCommandType();
}
