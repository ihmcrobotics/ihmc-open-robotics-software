package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

public abstract class InverseDynamicsCommand<T extends InverseDynamicsCommand<T>>
{
   public enum InverseDynamicsCommandType {TASKSPACE_MOTION, TASKSPACE_POINT_MOTION, JOINTSPACE_MOTION, MOMENTUM_RATE, COMMAND_LIST, EXTERNAL_WRENCH};
   private final InverseDynamicsCommandType commandType;

   public InverseDynamicsCommand(InverseDynamicsCommandType commandType)
   {
      this.commandType = commandType;
   }

   public abstract void set(T other);

   public InverseDynamicsCommandType getCommandType()
   {
      return commandType;
   }
}
