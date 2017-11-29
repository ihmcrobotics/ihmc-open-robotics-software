package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import us.ihmc.communication.controllerAPI.command.Command;

import java.util.Comparator;

/**
 * Compares the execution times of two commands
 */
public class CommandExecutionTimeComparator implements Comparator<Command<?,?>>
{
   
   /**
    * compares the execution time of {@code commandA} command with {@code commandB }. 
    * @return If commandA's execution time is less than commandB's it returns -1
    * If commandA's execution time is greater than commandB's it returns 1
    * If they are equal it returns 0
    * 
    */
   @Override
   public int compare(Command<?, ?> commandA, Command<?, ?> commandB)
   {
      double diff = commandA.getExecutionTime() - commandB.getExecutionTime();
      
      if(diff < 0)
      {
         return -1;
      }
      
      if(diff > 0)
      {
         return 1;
      }
      
      return 0;
   }

}
