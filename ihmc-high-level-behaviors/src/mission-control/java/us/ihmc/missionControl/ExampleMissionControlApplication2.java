package us.ihmc.missionControl;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.log.LogTools;
import us.ihmc.commons.thread.ExceptionHandlingThreadScheduler;

import java.util.Random;

public class ExampleMissionControlApplication2
{
   private final Random random = new Random();

   public ExampleMissionControlApplication2()
   {
      ExceptionHandlingThreadScheduler updateThreadScheduler = new ExceptionHandlingThreadScheduler("ExampleApplication2",
                                                                                                    DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      updateThreadScheduler.schedule(this::update, 0.5);
   }

   private void update()
   {
      LogTools.info("Random number: " + random.nextDouble() * 5000.0);
   }

   public static void main(String[] args)
   {
      new ExampleMissionControlApplication2();
   }
}
