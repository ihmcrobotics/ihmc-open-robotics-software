package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

public class InverseDynamicsCommandList implements InverseDynamicsCommand<InverseDynamicsCommandList>
{
   private final List<InverseDynamicsCommand<?>> commandList = new ArrayList<>();

   public InverseDynamicsCommandList()
   {
   }

   public void addCommand(InverseDynamicsCommand<?> command)
   {
      if (command != null)
         commandList.add(command);
   }

   public void clear()
   {
      commandList.clear();
   }

   public InverseDynamicsCommand<?> getCommand(int commandIndex)
   {
      return commandList.get(commandIndex);
   }

   public InverseDynamicsCommand<?> pollCommand()
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

   public boolean isEmpty()
   {
      return commandList.isEmpty();
   }

   @Override
   public void set(InverseDynamicsCommandList other)
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
