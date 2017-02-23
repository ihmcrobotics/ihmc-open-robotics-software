package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandType;
import us.ihmc.robotics.screwTheory.RigidBody;

public class VirtualWrenchCommandList
{
   private final List<VirtualWrenchCommand> commandList = new ArrayList<>();
   private final List<RigidBody> commandBodies = new ArrayList<>();

   public VirtualWrenchCommandList()
   {
   }

   public void addCommand(VirtualWrenchCommand command)
   {
      if (command != null)
      {
         commandList.add(command);
         commandBodies.add(command.getControlledBody());
      }
   }

   public void clear()
   {
      commandList.clear();
      commandBodies.clear();
   }

   public VirtualWrenchCommand getCommand(int commandIndex)
   {
      return commandList.get(commandIndex);
   }

   public VirtualWrenchCommand pollCommand()
   {
      if (commandList.isEmpty() || commandBodies.isEmpty())
         return null;
      else
      {
         commandBodies.remove(getNumberOfCommands() - 1);
         return commandList.remove(getNumberOfCommands() - 1);
      }
   }

   public int getNumberOfCommands()
   {
      return commandList.size();
   }

   public boolean isEmpty()
   {
      return commandList.isEmpty();
   }

   public boolean containsBody(RigidBody rigidBody)
   {
      return commandBodies.contains(rigidBody);
   }

   public void set(VirtualWrenchCommandList other)
   {
      clear();
      for (int i = 0; i < other.getNumberOfCommands(); i++)
         addCommand(other.getCommand(i));
   }

   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.COMMAND_LIST;
   }
}
