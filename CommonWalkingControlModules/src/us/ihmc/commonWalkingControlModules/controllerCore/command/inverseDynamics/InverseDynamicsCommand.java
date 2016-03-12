package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

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

   public abstract void set(T other);

   public abstract ControllerCoreCommandType getCommandType();
}
