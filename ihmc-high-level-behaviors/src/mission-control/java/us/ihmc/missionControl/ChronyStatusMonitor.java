package us.ihmc.missionControl;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.tools.processManagement.ProcessTools;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;
import us.ihmc.tools.thread.PausablePeriodicThread;

/**
 * Chrony takes a few seconds to give a status so we need to run in a separate thread.
 */
public class ChronyStatusMonitor
{
   private volatile String chronycBestSourceStatus;
   private final PausablePeriodicThread thread;

   public ChronyStatusMonitor()
   {
      thread = new PausablePeriodicThread("ChronyStatusUpdate", 2.0, true, this::checkChronyStatus);
   }

   public void start()
   {
      thread.start();
   }

   public void stop()
   {
      thread.stop();
   }

   public void checkChronyStatus()
   {
      String chronycSources = ProcessTools.execSimpleCommand("chronyc sources");
      String[] lines = chronycSources.split("\\R");
      String chronycBestSourceStatus = "Chrony is not currently synchronized with a source.";
      for (String line : lines)
      {
         if (line.contains("*")) // This is the selected best source by chrony.
         {
            chronycBestSourceStatus = line;
            break;
         }
      }
      this.chronycBestSourceStatus = chronycBestSourceStatus;
   }

   public String getChronycBestSourceStatus()
   {
      return chronycBestSourceStatus;
   }
}
