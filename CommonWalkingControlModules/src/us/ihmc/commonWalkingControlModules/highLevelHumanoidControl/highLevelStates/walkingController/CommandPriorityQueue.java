package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.command.Command;

/**
 * Used as a priority Queue to maintain a queue of commands that have execution delays
 * Should be Garbage free
 */
public class CommandPriorityQueue
{
   private final int size;
   private int count;
   private Command<?,?>[] activeQueue;
   private Command<?,?>[] inactiveQueue;
   private Command<?,?>[] swapQueue;
   
   public CommandPriorityQueue(int size)
   {
      this.size = size;
      activeQueue  = new Command<?,?>[size];
      inactiveQueue  = new Command<?,?>[size];
   }
  
   public Command<?,?> pop()
   {
      Command<?, ?> command = activeQueue[0];
      activeQueue[0] = null;
      for(int i = 1; i < count; i++)
      {
         activeQueue[i - 1] = activeQueue[i];
         activeQueue[i] = null;
      }
      count--;
      if(count < 0)
      {
         count = 0;
      }
      return command;
   }

   public Command<?,?> peek()
   {
      Command<?, ?> command = activeQueue[0];
      return command;
   }
   
   public boolean add(Command<?, ?> command)
   {
      if(count == size)
      {
         PrintTools.error("Try to add " + command.getClass() + " to the priority queue but the queue was full. Try increasing the queue size");
         return false;
      }
      
      double delay = command.getExecutionDelayTime();
      int newIndex = 0;
      boolean isInserted = false;
      
      for(int i = 0; i < count; i++)
      {
         if(activeQueue[i].getExecutionDelayTime() < delay || isInserted)
         {
            inactiveQueue[newIndex] = activeQueue[i];
            newIndex++;
         }
         else
         {
            inactiveQueue[newIndex] = command;
            newIndex++;
            inactiveQueue[newIndex] = activeQueue[i];
            newIndex++;
            isInserted = true;
         }
      }
      
      if(!isInserted)
      {
         inactiveQueue[newIndex] = command;
      }
      
      swapQueue = activeQueue;
      activeQueue = inactiveQueue;
      inactiveQueue = swapQueue;
      count++;
      return true;
   }
   
   public void clear()
   {
      for(int i = 0; i < activeQueue.length; i++)
      {
         activeQueue[i] = null;
         inactiveQueue[i] = null;
      }
      count = 0;
   }
   
   public int getSize()
   {
      return count;
   }
}
