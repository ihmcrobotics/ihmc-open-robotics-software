package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.apache.commons.lang3.NotImplementedException;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.lists.GenericTypeBuilder;

public class CommandConsumerWithDelayBuffersTest
{

   @Test
   public void testConstructor()
   {
      List<Class<? extends Command<?, ?>>> controllerSupportedCommands = ControllerAPIDefinition.getControllerSupportedCommands();
      CommandInputManager commandInputManager = new CommandInputManager(controllerSupportedCommands);
      DoubleYoVariable yoTime = new DoubleYoVariable("yoTime", null);
      CommandConsumerWithDelayBuffers commandConsumer = new CommandConsumerWithDelayBuffers(commandInputManager, yoTime);
      assertNotNull(commandConsumer);
      
      for(Class clazz: controllerSupportedCommands)
      {
         assertFalse(commandConsumer.isNewCommandAvailable(clazz));
         assertNull(commandConsumer.pollNewestCommand(clazz));
         assertEquals(0, commandConsumer.pollNewCommands(clazz).size());
      }
   }

   @Test
   public <C extends Command<C, ?>, M extends Packet<M>> void testIsNewCommandAvailableWithNoDelays() throws SecurityException, InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      Random random = new Random(100);
      List<Class<? extends Command<?, ?>>> controllerSupportedCommands = ControllerAPIDefinition.getControllerSupportedCommands();
      CommandInputManager commandInputManager = new CommandInputManager(controllerSupportedCommands);
      DoubleYoVariable yoTime = new DoubleYoVariable("yoTime", null);
      CommandConsumerWithDelayBuffers commandConsumer = new CommandConsumerWithDelayBuffers(commandInputManager, yoTime);
      for(Class<? extends Command<?, ?>> clazz: controllerSupportedCommands)
      {
         Command<?,M> command = getCommand(random, clazz);
         
         if(command.isCommandValid())
         {
            try
            {
               commandInputManager.submitCommand((C) command);
               commandConsumer.update();
               assertTrue(commandConsumer.isNewCommandAvailable(clazz));
            }
            catch (NotImplementedException e) 
            {
            }
         }
      }
      
      for(Class<? extends Command<?, ?>> clazz: controllerSupportedCommands)
      {
         //check if a command is available
         if(commandConsumer.isNewCommandAvailable(clazz))
         {
            //if it is available pop it
            assertNotNull(commandConsumer.pollNewestCommand((Class<C>) clazz));
         }
         else
         {
            //if it says there is nothing check that is true
            assertNull(commandConsumer.pollNewestCommand((Class<C>) clazz));
         }
         
         //we only added one command and then popped so now they should all be empty
         assertFalse(commandConsumer.isNewCommandAvailable(clazz));
      }
      
//      assertFalse(commandConsumer.isNewCommandAvailable(clazz));
//      assertNull(commandConsumer.pollNewestCommand(clazz));
//      assertEquals(0, commandConsumer.pollNewCommands(clazz).size());
      
   }

   @Test
   public <C extends Command<C, ?>, M extends Packet<M>> void testIsNewCommandAvailableWithDelays() throws SecurityException, InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      Random random = new Random(100);
      List<Class<? extends Command<?, ?>>> controllerSupportedCommands = ControllerAPIDefinition.getControllerSupportedCommands();
      CommandInputManager commandInputManager = new CommandInputManager(controllerSupportedCommands);
      DoubleYoVariable yoTime = new DoubleYoVariable("yoTime", null);
      CommandConsumerWithDelayBuffers commandConsumer = new CommandConsumerWithDelayBuffers(commandInputManager, yoTime);
      for(Class<? extends Command<?, ?>> clazz: controllerSupportedCommands)
      {
         yoTime.set(0.0);
         Command<?,M> command = getCommand(random, clazz);
         
         if(command.isCommandValid())
         {
            try
            {
               command.setExecutionDelayTime(random.nextDouble() + 0.1);
               commandInputManager.submitCommand((C) command);
               assertFalse(commandConsumer.isNewCommandAvailable(clazz));
            }
            catch (NotImplementedException e) 
            {
            }
         }
      }
      
      commandConsumer.update();
      
      
      //check no commands are available, (they should all be queued)
      for(Class<? extends Command<?, ?>> clazz: controllerSupportedCommands)
      {
         if(commandConsumer.isNewCommandAvailable(clazz))
         {
            //a command is available, but when we try to get it, it has a delay and gets queued, so the commandConsumer returns null
            assertNull(commandConsumer.pollNewestCommand((Class<C>) clazz));
         }
         
         
         //we only added one command and then popped so now they should all be empty
         assertFalse(commandConsumer.isNewCommandAvailable(clazz));
         assertNull(commandConsumer.pollNewestCommand((Class<C>) clazz));
         assertEquals(0, commandConsumer.pollNewCommands((Class<C>) clazz).size());
      }
      
      yoTime.set(2.0);
      
      //yoTime is now larger than the delay so all commands should be available
      for(Class<? extends Command<?, ?>> clazz: controllerSupportedCommands)
      {
         if(commandConsumer.isNewCommandAvailable(clazz))
         {
            //if it is available pop it
            assertNotNull(commandConsumer.pollNewestCommand((Class<C>) clazz));
            
            //we only added one command and then popped so now they should all be empty
            assertFalse(commandConsumer.isNewCommandAvailable(clazz));
            assertEquals(0, commandConsumer.pollNewCommands((Class<C>) clazz).size());
         }
         else
         {
            //if it says there is nothing check that is true
            assertNull(commandConsumer.pollNewestCommand((Class<C>) clazz));
            assertEquals(0, commandConsumer.pollNewCommands((Class<C>) clazz).size());
         }
      }
   }

   @Test
   public <C extends Command<C, ?>, M extends Packet<M>> void testSendMultipleCommandWithDelays() throws SecurityException, InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      Random random = new Random(10);
      List<Class<? extends Command<?, ?>>> controllerSupportedCommands = ControllerAPIDefinition.getControllerSupportedCommands();
      CommandInputManager commandInputManager = new CommandInputManager(controllerSupportedCommands);
      DoubleYoVariable yoTime = new DoubleYoVariable("yoTime", null);
      CommandConsumerWithDelayBuffers commandConsumer = new CommandConsumerWithDelayBuffers(commandInputManager, yoTime);
      for(Class<? extends Command<?, ?>> clazz: controllerSupportedCommands)
      {
         for (int i = 0; i < CommandConsumerWithDelayBuffers.NUMBER_OF_COMMANDS_TO_QUEUE; i++)
         {
            Command<?, M> command = getCommand(random, clazz);
            
            if (command.isCommandValid())
            {
               try
               {
                  command.setExecutionDelayTime(random.nextDouble() + 0.1);
                  commandInputManager.submitCommand((C) command);
                  commandConsumer.update();
                  assertFalse(commandConsumer.isNewCommandAvailable(clazz));
               }
               catch (NotImplementedException e)
               {
               }
            }
         }
      }
      
      for(Class<? extends Command<?, ?>> clazz: controllerSupportedCommands)
      {
         if(commandConsumer.isNewCommandAvailable(clazz))
         {
            //a command is available, but when we try to get it, it has a delay and gets queued, so the commandConsumer returns null
            assertEquals(0,commandConsumer.pollNewCommands((Class<C>) clazz).size());
         }
         
         
         //we only added one command and then popped so now they should all be empty
         assertFalse(commandConsumer.isNewCommandAvailable(clazz));
         assertNull(commandConsumer.pollNewestCommand((Class<C>) clazz));
         assertEquals(0, commandConsumer.pollNewCommands((Class<C>) clazz).size());
      }
      
      yoTime.set(2.0);
      
      //yoTime is now larger than the delay so all commands should be available
      for(Class<? extends Command<?, ?>> clazz: controllerSupportedCommands)
      {
//      Class clazz = FootstepDataListCommand.class;
//      Class clazz = FootstepDataListCommand.class;
         if(commandConsumer.isNewCommandAvailable(clazz))
         {
            System.out.println(clazz);
            assertEquals(CommandConsumerWithDelayBuffers.NUMBER_OF_COMMANDS_TO_QUEUE, commandConsumer.pollNewCommands((Class<C>) clazz).size());
            //we added several commands and then popped them so now they should all be empty
            assertNull(commandConsumer.pollNewestCommand((Class<C>) clazz));
            assertFalse(commandConsumer.isNewCommandAvailable(clazz));
            
         }
         else
         {
            //if it says there is nothing check that is true
            assertNull(commandConsumer.pollNewestCommand((Class<C>) clazz));
            assertEquals(0, commandConsumer.pollNewCommands((Class<C>) clazz).size());
         }
      }
   }
   
   @Test
   public <C extends Command<C, ?>, M extends Packet<M>> void testQueueingManually()
   {
      Random random = new Random(100);
      List<Class<? extends Command<?, ?>>> controllerSupportedCommands = new ArrayList<>();
      controllerSupportedCommands.add(TestCommand.class);
      CommandInputManager commandInputManager = new CommandInputManager(controllerSupportedCommands);
      DoubleYoVariable yoTime = new DoubleYoVariable("yoTime", null);
      yoTime.set(random.nextDouble() * random.nextInt(1000));
      CommandConsumerWithDelayBuffers commandConsumer = new CommandConsumerWithDelayBuffers(commandInputManager, yoTime);
      
      TestCommand[] commands = new TestCommand[CommandConsumerWithDelayBuffers.NUMBER_OF_COMMANDS_TO_QUEUE];
      ArrayList<TestCommand> randomOrderedCommands = new ArrayList<TestCommand>();
      for(int i = 0; i < commands.length; i++)
      {
         TestCommand command = new TestCommand();
         command.setExecutionDelayTime(i + 0.5);
         command.setData(random.nextLong());
         commands[i] = command;
         randomOrderedCommands.add(random.nextInt(randomOrderedCommands.size() + 1), command);
      }
      
      for(int i = 0; i < randomOrderedCommands.size(); i++)
      {
         commandInputManager.submitCommand(randomOrderedCommands.get(i));
      }
      assertFalse(commandConsumer.isNewCommandAvailable(TestCommand.class));
      commandConsumer.update();
      assertFalse(commandConsumer.isNewCommandAvailable(TestCommand.class));
      assertEquals(0,commandConsumer.pollNewCommands(TestCommand.class).size());
      
      
      double startTime = yoTime.getDoubleValue();
      for(int i = 0; i < commands.length; i++)
      {
         //alternate between exact delay time and past the delay time
         if(random.nextBoolean())
         {
            yoTime.set(startTime + i + 0.5);
         }
         else
         {
            yoTime.set(startTime + i + 0.5 + 1e-8);
         }
         assertTrue(commandConsumer.isNewCommandAvailable(TestCommand.class));
         List<TestCommand> polledCommands = commandConsumer.pollNewCommands(TestCommand.class);
         System.out.println(i);
         assertEquals(1,polledCommands.size());
         
         assertEquals(commands[i].getData(), polledCommands.get(0).getData());
         assertEquals(0,commandConsumer.pollNewCommands(TestCommand.class).size());
         
      }
   }
   
   @Test
   public <C extends Command<C, ?>, M extends Packet<M>> void testSendingNonDelayedCommandWhenCommandsAreDelayedClearsAllDelayedCommands()
   {
      Random random = new Random(100);
      List<Class<? extends Command<?, ?>>> controllerSupportedCommands = new ArrayList<>();
      controllerSupportedCommands.add(TestCommand.class);
      CommandInputManager commandInputManager = new CommandInputManager(controllerSupportedCommands);
      DoubleYoVariable yoTime = new DoubleYoVariable("yoTime", null);
      CommandConsumerWithDelayBuffers commandConsumer = new CommandConsumerWithDelayBuffers(commandInputManager, yoTime);
      
      TestCommand[] commands = new TestCommand[CommandConsumerWithDelayBuffers.NUMBER_OF_COMMANDS_TO_QUEUE - 1];
      ArrayList<TestCommand> randomOrderedCommands = new ArrayList<TestCommand>();
      for(int i = 0; i < commands.length; i++)
      {
         TestCommand command = new TestCommand();
         command.setExecutionDelayTime(i + 0.5);
         command.setData(random.nextLong());
         commands[i] = command;
         randomOrderedCommands.add(random.nextInt(randomOrderedCommands.size() + 1), command);
      }
      
      for(int i = 0; i < randomOrderedCommands.size(); i++)
      {
         commandInputManager.submitCommand(randomOrderedCommands.get(i));
      }
      
      commandConsumer.update();
      assertFalse(commandConsumer.isNewCommandAvailable(TestCommand.class));
      assertEquals(0,commandConsumer.pollNewCommands(TestCommand.class).size());
      
      TestCommand notDelayedCommand = new TestCommand();
      notDelayedCommand.setData(random.nextLong());
      commandInputManager.submitCommand(notDelayedCommand);
      
      commandConsumer.update();
      assertTrue(commandConsumer.isNewCommandAvailable(TestCommand.class));
      List<TestCommand> polledCommands = commandConsumer.pollNewCommands(TestCommand.class);
      assertEquals(1,polledCommands.size());
      assertTrue(notDelayedCommand.equals(polledCommands.get(0)));
      
      yoTime.set(CommandConsumerWithDelayBuffers.NUMBER_OF_COMMANDS_TO_QUEUE);
      assertFalse(commandConsumer.isNewCommandAvailable(TestCommand.class));
      assertEquals(0,commandConsumer.pollNewCommands(TestCommand.class).size());
   }
  
   @Test
   public <C extends Command<C, ?>, M extends Packet<M>> void testFlushCommands()
   {
      Random random = new Random(100);
      List<Class<? extends Command<?, ?>>> controllerSupportedCommands = new ArrayList<>();
      controllerSupportedCommands.add(TestCommand.class);
      CommandInputManager commandInputManager = new CommandInputManager(controllerSupportedCommands);
      DoubleYoVariable yoTime = new DoubleYoVariable("yoTime", null);
      CommandConsumerWithDelayBuffers commandConsumer = new CommandConsumerWithDelayBuffers(commandInputManager, yoTime);
      
      TestCommand[] commands = new TestCommand[CommandConsumerWithDelayBuffers.NUMBER_OF_COMMANDS_TO_QUEUE];
      ArrayList<TestCommand> randomOrderedCommands = new ArrayList<TestCommand>();
      for(int i = 0; i < commands.length; i++)
      {
         TestCommand command = new TestCommand();
         command.setExecutionDelayTime(i + 0.5);
         command.setData(random.nextLong());
         commands[i] = command;
         randomOrderedCommands.add(random.nextInt(randomOrderedCommands.size() + 1), command);
      }
      
      for(int i = 0; i < randomOrderedCommands.size(); i++)
      {
         commandInputManager.submitCommand(randomOrderedCommands.get(i));
      }
      
      commandConsumer.update();
      assertFalse(commandConsumer.isNewCommandAvailable(TestCommand.class));
      assertEquals(0,commandConsumer.pollNewCommands(TestCommand.class).size());

      commandConsumer.flushCommands(TestCommand.class);
      
      assertFalse(commandConsumer.isNewCommandAvailable(TestCommand.class));
      assertEquals(0,commandConsumer.pollNewCommands(TestCommand.class).size());
      
      yoTime.set(CommandConsumerWithDelayBuffers.NUMBER_OF_COMMANDS_TO_QUEUE + 1);
      assertFalse(commandConsumer.isNewCommandAvailable(TestCommand.class));
      assertEquals(0,commandConsumer.pollNewCommands(TestCommand.class).size());
   }
   
   @Test
   public <C extends Command<C, ?>, M extends Packet<M>> void testAddingTooManyCommands()
   {
      Random random = new Random(100);
      List<Class<? extends Command<?, ?>>> controllerSupportedCommands = new ArrayList<>();
      controllerSupportedCommands.add(TestCommand.class);
      CommandInputManager commandInputManager = new CommandInputManager(controllerSupportedCommands);
      DoubleYoVariable yoTime = new DoubleYoVariable("yoTime", null);
      CommandConsumerWithDelayBuffers commandConsumer = new CommandConsumerWithDelayBuffers(commandInputManager, yoTime);
      
      TestCommand[] commands = new TestCommand[CommandConsumerWithDelayBuffers.NUMBER_OF_COMMANDS_TO_QUEUE + 1];
      ArrayList<TestCommand> randomOrderedCommands = new ArrayList<TestCommand>();
      for(int i = 0; i < commands.length; i++)
      {
         TestCommand command = new TestCommand();
         command.setExecutionDelayTime(i + 0.5);
         command.setData(random.nextLong());
         commands[i] = command;
         randomOrderedCommands.add(random.nextInt(randomOrderedCommands.size() + 1), command);
      }
      
      for(int i = 0; i < randomOrderedCommands.size(); i++)
      {
         commandInputManager.submitCommand(randomOrderedCommands.get(i));
         commandConsumer.update();
      }
      
      yoTime.set(CommandConsumerWithDelayBuffers.NUMBER_OF_COMMANDS_TO_QUEUE + 2);
      assertEquals(CommandConsumerWithDelayBuffers.NUMBER_OF_COMMANDS_TO_QUEUE,commandConsumer.pollNewCommands(TestCommand.class).size());
      
      
   }
   
   private <M extends Packet<M>> Command<?, M> getCommand(Random random, Class clazz)
         throws InstantiationException, IllegalAccessException, InvocationTargetException
   {
      Command<?, M> command;
      command = (Command<?, M>) getInstanceUsingRandomConstructor(random, clazz);
      
      if(command == null)
      {
         command = (Command<?, M>) getInstanceUsingEmptyConstructor(clazz);
         if(!command.isCommandValid())
         {
            Class<?> messageClass = command.getMessageClass();
            Packet<M> message = (Packet<M>) getInstanceUsingRandomConstructor(random, messageClass);
            if(message != null)
            {
               command.set((M) message);
            }
         }
      }
      return command;
   }

   private Command<?, ?> getInstanceUsingEmptyConstructor(Class clazz)
   {
      Command<?, ?> command;
      GenericTypeBuilder builder = GenericTypeBuilder.createBuilderWithEmptyConstructor(clazz);
      command = (Command<?, ?>) builder.newInstance();
      return command;
   }

   private Object getInstanceUsingRandomConstructor(Random random, Class clazz)
         throws InstantiationException, IllegalAccessException, InvocationTargetException
   {
      try
      {
         Constructor randomConstructor = clazz.getConstructor(Random.class);
         return randomConstructor.newInstance(random);
      }
      catch (NoSuchMethodException e) 
      {
         
      }
      return null;
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(CommandConsumerWithDelayBuffers.class, CommandConsumerWithDelayBuffersTest.class);
   }

}
