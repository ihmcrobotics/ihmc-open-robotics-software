package us.ihmc.commonWalkingControlModules.inverseKinematics.dataObjects;

public abstract class InverseKinematicsCommand<T extends InverseKinematicsCommand<T>>
{
   public enum InverseKinematicsCommandType
   {
      JOINTSPACE, TASKSPACE, MOMENTUM,
      PRIVILIEGED_CONFIGURATION,
      COMMAND_LIST
   };

   private final InverseKinematicsCommandType commandType;

   public InverseKinematicsCommand(InverseKinematicsCommandType commandType)
   {
      this.commandType = commandType;
   }

   public abstract void set(T other);

   public InverseKinematicsCommandType getCommandType()
   {
      return commandType;
   }
}
