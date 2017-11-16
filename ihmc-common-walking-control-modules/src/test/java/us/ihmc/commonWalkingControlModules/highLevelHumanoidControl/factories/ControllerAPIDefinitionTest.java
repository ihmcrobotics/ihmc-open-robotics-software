package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import static org.junit.Assert.assertEquals;

import java.util.List;
import java.util.Random;

import org.apache.commons.lang3.NotImplementedException;
import org.junit.Test;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.robotics.lists.GenericTypeBuilder;

public class ControllerAPIDefinitionTest
{

   @Test(timeout = 30000)
   public void testCommandSetters()
   {
      Random random = new Random();
      List<Class<? extends Command<?, ?>>> controllerSupportedCommands = ControllerAPIDefinition.getControllerSupportedCommands();
      for(int i = 0; i < controllerSupportedCommands.size(); i++)
      {
         Class<? extends Command<?, ?>> clazz = controllerSupportedCommands.get(i);
         Command command = getInstanceUsingEmptyConstructor(clazz);
         Command otherCommand = getInstanceUsingEmptyConstructor(clazz);
         
         try
         {
            double delayTime = random.nextDouble() * random.nextInt(1000);
            command.setExecutionDelayTime(delayTime);
            
            if(command instanceof QueueableCommand)
            {
               QueueableCommand queueableCommand = (QueueableCommand) command;
               queueableCommand.setCommandId(random.nextInt(1000));
               queueableCommand.setExecutionMode(ExecutionMode.values[random.nextInt(ExecutionMode.values.length)]);
               queueableCommand.setPreviousCommandId(random.nextInt(1000));
            }
               
            otherCommand.set(command);
            
            assertEquals(otherCommand.getClass() + " set method doesn't set delayTime correctly", delayTime, otherCommand.getExecutionDelayTime(), 1e-8);
            if(otherCommand instanceof QueueableCommand)
            {
               QueueableCommand queueableCommand = (QueueableCommand) command;
               QueueableCommand otherQueueableCommand = (QueueableCommand) otherCommand;
               
               assertEquals(otherCommand.getClass() + " set method doesn't set command id correctly", queueableCommand.getCommandId(), ((QueueableCommand) otherCommand).getCommandId());
               assertEquals(otherCommand.getClass() + " set method doesn't set execution mode correctly", queueableCommand.getExecutionMode(), ((QueueableCommand) otherCommand).getExecutionMode());
               assertEquals(otherCommand.getClass() + " set method doesn't set previous command id correctly", queueableCommand.getPreviousCommandId(), ((QueueableCommand) otherCommand).getPreviousCommandId());
            }
         }
         catch (NotImplementedException e) 
         {
         }
      }
   }
   
   private Command<?, ?> getInstanceUsingEmptyConstructor(Class clazz)
   {
      Command<?, ?> command;
      GenericTypeBuilder builder = GenericTypeBuilder.createBuilderWithEmptyConstructor(clazz);
      command = (Command<?, ?>) builder.newInstance();
      return command;
   }

}
