package us.ihmc.commonWalkingControlModules.controllerCore.command.jacobianTranspose;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;

import java.util.ArrayList;
import java.util.List;

public class JacobianTransposeCommandList implements JacobianTransposeCommand<JacobianTransposeCommandList>
{
   private final List<JacobianTransposeCommand<?>> commandList = new ArrayList<>();

   public JacobianTransposeCommandList()
   {
   }

   public void addCommand(JacobianTransposeCommand<?> command)
   {
      if (command != null)
         commandList.add(command);
   }

   public void clear()
   {
      commandList.clear();
   }

   public JacobianTransposeCommand<?> getCommand(int commandIndex)
   {
      return commandList.get(commandIndex);
   }

   public JacobianTransposeCommand<?> pollCommand()
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
   public void set(JacobianTransposeCommandList other)
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
}
