package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import java.lang.reflect.Array;

import us.ihmc.commons.PrintTools;

/**
 * Should be Garbage free
 */
public class CommandPriorityQueue<T extends Comparable<T>>
{
   private final int size;
   private int count;
   
   // Not the most efficient method but it is garbage free
   // elements are stored smallest to biggest in the array
   // pop takes O(n) and add is O(n)
   // feel free to optimize
   private T[] activeQueue;
   private T[] inactiveQueue;
   private T[] swapQueue;
   
   /**
    * 
    * @param size the max size of the priority queue
    */
   @SuppressWarnings("unchecked")
   public CommandPriorityQueue(int size, Class<?> clazz)
   {
      this.size = size;
      activeQueue  = (T[]) Array.newInstance(clazz, size);
      inactiveQueue  = (T[]) Array.newInstance(clazz, size);
   }
  
   /**
    * removes and returns the smallest comparable in the queue
    * takes O(n)
    * @return the smallest comparable in the queue
    */
   public T pop()
   {
      T comparable = activeQueue[0];
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
      return comparable;
   }

   /**
    * @return the smallest comparable in the queue
    */
   public T peek()
   {
      T comparable = activeQueue[0];
      return comparable;
   }
   
   /**
    * Tries to add an element to the queue. 
    * takes O(n)
    * @param comparable the object to add to the queue
    * @return whether or not the add was successful
    */
   public boolean add(T comparable)
   {
      if(count == size)
      {
         PrintTools.error("Try to add " + comparable.getClass() + " to the priority queue but the queue was full. Try increasing the queue size");
         return false;
      }
      
      int newIndex = 0;
      boolean isInserted = false;
      
      for(int i = 0; i < count; i++)
      {
         if(activeQueue[i].compareTo(comparable) < 0 || isInserted)
         {
            inactiveQueue[newIndex] = activeQueue[i];
            newIndex++;
         }
         else
         {
            inactiveQueue[newIndex] = comparable;
            newIndex++;
            inactiveQueue[newIndex] = activeQueue[i];
            newIndex++;
            isInserted = true;
         }
      }
      
      if(!isInserted)
      {
         inactiveQueue[newIndex] = comparable;
      }
      
      swapQueue = activeQueue;
      activeQueue = inactiveQueue;
      inactiveQueue = swapQueue;
      count++;
      return true;
   }
   
   /**
    * removes all the objects from the queue
    */
   public void clear()
   {
      for(int i = 0; i < activeQueue.length; i++)
      {
         activeQueue[i] = null;
         inactiveQueue[i] = null;
      }
      count = 0;
   }
   
   /**
    * @return the number of elements in the queue
    */
   public int getSize()
   {
      return count;
   }
}
