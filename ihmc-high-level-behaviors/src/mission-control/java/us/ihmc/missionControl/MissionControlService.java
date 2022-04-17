package us.ihmc.missionControl;

import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;

public class MissionControlService
{
   private final LinuxResourceMonitor linuxResourceMonitor = new LinuxResourceMonitor();

   public MissionControlService()
   {
      ExceptionHandlingThreadScheduler updateThreadScheduler = new ExceptionHandlingThreadScheduler("MissionControlUpdate",
                                                                                                    DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      updateThreadScheduler.schedule(this::update, 1.0);

//      ROS2



   }

   private void update()
   {
      linuxResourceMonitor.update();

      LogTools.info("RAM: " + FormattingTools.getFormattedDecimal1D(linuxResourceMonitor.getUsedRAMGiB()) + " / " + FormattingTools.getFormattedDecimal1D(linuxResourceMonitor.getTotalRAMGiB()) + " GiB");
   }

   public static void main(String[] args)
   {

      new MissionControlService();
   }
}
