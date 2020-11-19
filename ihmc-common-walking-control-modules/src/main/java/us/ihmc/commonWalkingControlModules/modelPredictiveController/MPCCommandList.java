package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCCommandType;

import java.util.ArrayList;
import java.util.List;

public class MPCCommandList implements MPCCommand<MPCCommandList>
{
   private int commandId;
   private final List<MPCCommand<?>> commandList = new ArrayList<>();

   @Override
   public MPCCommandType getCommandType()
   {
      return MPCCommandType.LIST;
   }

   public void addCommand(MPCCommand<?> command)
   {
      if (command != null)
      {
         if (MPCCommandType.LIST == command.getCommandType())
            addCommandList((MPCCommandList) command);
         else
            commandList.add(command);
      }
   }

   public void addCommandList(MPCCommandList commandList)
   {
      if (commandList == null)
         return;

      for (int i = 0; i < commandList.getNumberOfCommands(); i++)
         addCommand(commandList.getCommand(i));
   }

   public void clear()
   {
      commandId = 0;
      commandList.clear();
   }

   public MPCCommand<?> getCommand(int commandIndex)
   {
      return commandList.get(commandIndex);
   }

   public int getNumberOfCommands()
   {
      return commandList.size();
   }

   public void setCommandId(int id)
   {
      commandId = id;
   }

   public int getCommandId()
   {
      return commandId;
   }

   public void set(MPCCommandList other)
   {
      clear();
      setCommandId(other.getCommandId());
      for (int i = 0; i < other.getNumberOfCommands(); i++)
         addCommand(other.getCommand(i));
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof MPCCommandList)
      {
         MPCCommandList other = (MPCCommandList) object;

         if (commandId != other.commandId)
            return false;
         if (getNumberOfCommands() != other.getNumberOfCommands())
            return false;
         for (int commandIndex = 0; commandIndex < getNumberOfCommands(); commandIndex++)
         {
            if (!getCommand(commandIndex).equals(other.getCommand(commandIndex)))
               return false;
         }

         return true;
      }
      else
      {
         return false;
      }
   }
}
