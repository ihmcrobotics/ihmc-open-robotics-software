package us.ihmc.rdx.ui.processes;

import imgui.internal.ImGui;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public abstract class RestartableProcess
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   volatile boolean starting = false;
   volatile boolean started = false;
   volatile boolean stopping = false;

   private ResettableExceptionHandlingExecutorService executorService;

   public ResettableExceptionHandlingExecutorService getOrCreateExecutor()
   {
      if (executorService == null)
         executorService = MissingThreadTools.newSingleThreadExecutor(getName() + "ManagementThread", true);
      return executorService;
   }

   protected abstract void startInternal();

   protected abstract void stopInternal();

   public abstract String getName();

   public void renderImGuiWidgets()
   {
      ImGui.text(getName() + ":");
      ImGui.sameLine();
      if (started)
      {
         if (ImGui.button(labels.get("Restart")))
         {
            stop();
            start();
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Stop")))
         {
            stop();
         }
      }
      else if (starting)
      {
         ImGui.text("Starting...");
//         ImGui.sameLine();
//         if (ImGui.button(labels.get("Cancel", 0)))
//         {
//            getOrCreateExecutor().interruptAndReset();
//            starting = false;
//            started = false;
//            stopping = false;
//         }
      }
      else if (stopping)
      {
         ImGui.text("Stopping...");
//         ImGui.sameLine();
//         if (ImGui.button(labels.get("Cancel", 1)))
//         {
//            getOrCreateExecutor().interruptAndReset();
//            starting = false;
//            started = false;
//            stopping = false;
//         }
      }
      else
      {
         if (ImGui.button(labels.get("Start")))
         {
            start();
         }
      }
   }

   public void start()
   {
      starting = true;
      getOrCreateExecutor().execute(() ->
      {
         startInternal();
         starting = false;
         started = true;
      });
   }

   public void stop()
   {
      started = false;
      stopping = true;
      getOrCreateExecutor().execute(() ->
      {
         stopInternal();
         stopping = false;
      });
   }

   public void destroy()
   {
      Stopwatch stopwatch = new Stopwatch().start();
      if (starting)
      {
         LogTools.info("Waiting for start...");
         while (starting && stopwatch.totalElapsed() < 10.0)
            ThreadTools.sleep(100);
         if (stopwatch.totalElapsed() < 10.0)
         {
            LogTools.info("{} started.", getName());
            stopwatch.reset();
         }
         else
         {
            LogTools.error("{} ran out of time to wait!", getName());
         }
      }
      if (started && !stopping)
      {
         LogTools.info("Stopping {}...", getName());
         stop();
      }
      if (stopping)
      {
         while (stopping && stopwatch.totalElapsed() < 10.0)
            ThreadTools.sleep(100);
         if (stopwatch.totalElapsed() < 10.0)
         {
            LogTools.info("{} stopped...", getName());
         }
         else
         {
            LogTools.error("{} ran out of time to stop!", getName());
         }
      }
      if (executorService != null && !executorService.isShutdown())
         executorService.destroy();
   }

   public boolean isStarted()
   {
      return started;
   }
}
