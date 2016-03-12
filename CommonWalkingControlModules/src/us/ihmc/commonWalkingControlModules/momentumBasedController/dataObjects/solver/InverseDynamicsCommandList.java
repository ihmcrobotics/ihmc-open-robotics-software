package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver;

import java.util.ArrayList;
import java.util.List;

public class InverseDynamicsCommandList implements InverseDynamicsCommand<InverseDynamicsCommandList>
{
   private final List<InverseDynamicsCommand<?>> commandList = new ArrayList<>();

   public InverseDynamicsCommandList()
   {
   }

   public void addCommand(InverseDynamicsCommand<?> command)
   {
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

   public int getNumberOfCommands()
   {
      return commandList.size();
   }

   @Override
   public void set(InverseDynamicsCommandList other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfCommands(); i++)
         addCommand(other.getCommand(i));
   }

   @Override
   public InverseDynamicsCommandType getCommandType()
   {
      return InverseDynamicsCommandType.COMMAND_LIST;
   }
}
