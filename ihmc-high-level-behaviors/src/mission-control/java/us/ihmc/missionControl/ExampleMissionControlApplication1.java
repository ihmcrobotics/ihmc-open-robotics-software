package us.ihmc.missionControl;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.log.LogTools;
import us.ihmc.commons.thread.ExceptionHandlingThreadScheduler;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

public class ExampleMissionControlApplication1
{
   public ExampleMissionControlApplication1()
   {
      ExceptionHandlingThreadScheduler updateThreadScheduler = new ExceptionHandlingThreadScheduler("ExampleApplication1",
                                                                                                    DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      updateThreadScheduler.schedule(this::update, 0.5);
   }

   private void update()
   {
      LogTools.info("Date: " + LocalDateTime.now().format(DateTimeFormatter.ISO_LOCAL_DATE_TIME));
   }

   public static void main(String[] args)
   {
      new ExampleMissionControlApplication1();
   }
}
