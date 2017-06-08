package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.controllerAPI.command.QueueableCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ChestTrajectoryCommand;

public class QueueableCommandPriorityQueueTest
{

   @Test
   public void testQueueableCommandPriorityQueue()
   {
      CommandPriorityQueue commandPriorityQueue = new CommandPriorityQueue(10);
      for(int i = 0; i < 10; i++)
      {
         ChestTrajectoryCommand chestTrajectoryCommand = new ChestTrajectoryCommand();
         chestTrajectoryCommand.setExecutionDelayTime(i);
         assertTrue(commandPriorityQueue.add(chestTrajectoryCommand));
      }

      for(int i = 0; i < 10; i++)
      {
         ChestTrajectoryCommand chestTrajectoryCommand = new ChestTrajectoryCommand();
         assertFalse(commandPriorityQueue.add(chestTrajectoryCommand));
      }
   }

   @Test
   public void testSameDelay()
   {
      CommandPriorityQueue commandPriorityQueue = new CommandPriorityQueue(10);
      for(int i = 0; i < 10; i++)
      {
         ChestTrajectoryCommand chestTrajectoryCommand = new ChestTrajectoryCommand();
         chestTrajectoryCommand.setExecutionDelayTime(5.0);
         assertTrue(commandPriorityQueue.add(chestTrajectoryCommand));
      }
      
      int index = 0;
      while(commandPriorityQueue.peek() != null)
      {
         Command<?, ?> command = commandPriorityQueue.pop();
         assertEquals(9 - index, commandPriorityQueue.getSize());
         assertEquals(5.0, command.getExecutionDelayTime(), 1e-10);
         index++;
      }
      assertEquals(10, index);
   }

   @Test
   public void testAddingInOrder()
   {
      CommandPriorityQueue commandPriorityQueue = new CommandPriorityQueue(10);
      for(int i = 0; i < 10; i++)
      {
         ChestTrajectoryCommand chestTrajectoryCommand = new ChestTrajectoryCommand();
         chestTrajectoryCommand.setExecutionDelayTime(i);
         assertTrue(commandPriorityQueue.add(chestTrajectoryCommand));
      }
      
      int index = 0;
      while(commandPriorityQueue.peek() != null)
      {
         Command<?, ?> command = commandPriorityQueue.pop();
         assertEquals(index, command.getExecutionDelayTime(), 1e-10);
         index++;
      }
      assertEquals(10, index);
   }

   @Test
   public void testPop()
   {
      CommandPriorityQueue commandPriorityQueue = new CommandPriorityQueue(10);
      ChestTrajectoryCommand[] commands = new ChestTrajectoryCommand[10];
      for(int i = 0; i < 10; i++)
      {
         ChestTrajectoryCommand chestTrajectoryCommand = new ChestTrajectoryCommand();
         chestTrajectoryCommand.setExecutionDelayTime(10 - i);
         commands[9 - i] = chestTrajectoryCommand;
         assertTrue(commandPriorityQueue.add(chestTrajectoryCommand));
      }
      
      int index = 0;
      while(commandPriorityQueue.peek() != null)
      {
         assertEquals(commands[index], commandPriorityQueue.pop());
         index++;
      }
   }

   @Test
   public void testPopExtended()
   {
      int numberOfCommands = 100;
      ArrayList<ChestTrajectoryCommand> commandsInRandomOrder = new ArrayList<>();
      Random random = new Random(100);
      CommandPriorityQueue commandPriorityQueue = new CommandPriorityQueue(numberOfCommands);
      ChestTrajectoryCommand[] commands = new ChestTrajectoryCommand[numberOfCommands];
      
      //get a hundred random numbers
      double[] delays = new double[numberOfCommands];
      for(int i = 0; i < delays.length; i++)
      {
         delays[i] = random.nextDouble() * random.nextInt(1000);
      }
      
      //sort them
      Arrays.sort(delays);
      
      //put the delays in the commands
      //build a list of commands in random order
      for(int i = 0; i < delays.length; i++)
      {
         ChestTrajectoryCommand chestTrajectoryCommand = new ChestTrajectoryCommand();
         chestTrajectoryCommand.setExecutionDelayTime(delays[i]);
         commands[i] = chestTrajectoryCommand;
         commandsInRandomOrder.add(random.nextInt(commandsInRandomOrder.size() + 1), chestTrajectoryCommand);
      }
      
      //put the commands in the priority queue
      for(int i = 0; i < commandsInRandomOrder.size(); i++)
      {
         ChestTrajectoryCommand chestTrajectoryCommand = commandsInRandomOrder.get(i);
         assertTrue(commandPriorityQueue.add(chestTrajectoryCommand));
      }
      
      //test that the output matches the sorted queue
      int index = 0;
      while(commandPriorityQueue.peek() != null)
      {
         assertEquals(commands[index], commandPriorityQueue.pop());
         index++;
      }
   }
   
   @Test
   public void testPeek()
   { 
      CommandPriorityQueue commandPriorityQueue = new CommandPriorityQueue(10);
      ChestTrajectoryCommand chestTrajectoryCommand = new ChestTrajectoryCommand();
      chestTrajectoryCommand.setExecutionDelayTime(5.0);
      assertTrue(commandPriorityQueue.add(chestTrajectoryCommand));
      assertEquals(chestTrajectoryCommand, commandPriorityQueue.peek());
      assertEquals(1, commandPriorityQueue.getSize());
   }
   
   @Test
   public void testPopWhenEmpty()
   { 
      CommandPriorityQueue commandPriorityQueue = new CommandPriorityQueue(10);
      assertNull(commandPriorityQueue.pop());
      assertEquals(0, commandPriorityQueue.getSize());
   }

   @Test
   public void testClear()
   { 
      CommandPriorityQueue commandPriorityQueue = new CommandPriorityQueue(10);
      for(int i = 0; i < 10; i++)
      {
         ChestTrajectoryCommand chestTrajectoryCommand = new ChestTrajectoryCommand();
         chestTrajectoryCommand.setExecutionDelayTime(i);
         assertTrue(commandPriorityQueue.add(chestTrajectoryCommand));
      }
      
      commandPriorityQueue.clear();
      int index = 0;
      while(commandPriorityQueue.peek() != null)
      {
         Command<?, ?> command = commandPriorityQueue.pop();
         assertEquals(index, command.getExecutionDelayTime(), 1e-10);
         index++;
      }
      assertEquals(0, index);
      assertEquals(0, commandPriorityQueue.getSize());
      commandPriorityQueue.clear();
      assertEquals(0, index);
      assertEquals(0, commandPriorityQueue.getSize());
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(CommandPriorityQueue.class, QueueableCommandPriorityQueueTest.class);
   }
}
