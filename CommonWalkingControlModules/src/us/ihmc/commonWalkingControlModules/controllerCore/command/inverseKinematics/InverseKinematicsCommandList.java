package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

public class InverseKinematicsCommandList implements InverseKinematicsCommand<InverseKinematicsCommandList>
{
   private final List<InverseKinematicsCommand<?>> commandList = new ArrayList<>();

   public InverseKinematicsCommandList()
   {
   }

   public void addCommand(InverseKinematicsCommand<?> command)
   {
      if (command != null)
         commandList.add(command);
   }

   public void clear()
   {
      commandList.clear();
   }

   public InverseKinematicsCommand<?> getCommand(int commandIndex)
   {
      return commandList.get(commandIndex);
   }

   public InverseKinematicsCommand<?> pollCommand()
   {
      if (commandList.isEmpty())
         return null;
      else
         return commandList.remove(getNumberOfCommands() - 1);
   }

   public int getNumberOfCommands()
   {
      return commandList.size();
   }

   @Override
   public void set(InverseKinematicsCommandList other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfCommands(); i++)
         addCommand(other.getCommand(i));
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.COMMAND_LIST;
   }

   @Override
   public String toString()
   {
      return "Nb of commands: " + getNumberOfCommands() + "\n" + commandList;
   }
}
