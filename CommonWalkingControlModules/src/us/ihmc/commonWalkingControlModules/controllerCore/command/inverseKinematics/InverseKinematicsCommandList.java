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
   public void setWeight(double weight)
   {
      for (int i = 0; i < getNumberOfCommands(); i++)
         commandList.get(i).setWeight(weight);
   }

   @Override
   public void setWeightLevel(InverseKinematicsCommandWeightLevels weightLevel)
   {
      setWeight(weightLevel.getWeightValue());
   }

   @Override
   public boolean isHardConstraint()
   {
      return false;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.COMMAND_LIST;
   }
}
