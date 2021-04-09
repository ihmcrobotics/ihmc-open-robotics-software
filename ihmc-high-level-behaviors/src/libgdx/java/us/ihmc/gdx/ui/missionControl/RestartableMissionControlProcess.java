package us.ihmc.gdx.ui.missionControl;

import imgui.internal.ImGui;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

public abstract class RestartableMissionControlProcess implements MissionControlProcess
{
   volatile boolean starting = false;
   volatile boolean started = false;
   volatile boolean stopping = false;

   private final ResettableExceptionHandlingExecutorService executorService;

   public RestartableMissionControlProcess()
   {
      executorService = MissingThreadTools.newSingleThreadExecutor(getName() + "ManagementThread", true);
   }

   protected abstract void startInternal();

   protected abstract void stopInternal();

   public abstract String getName();

   @Override
   public void render()
   {
      ImGui.text(getName() + ":");
      ImGui.sameLine();
      if (started)
      {
         if (ImGui.button(ImGuiTools.uniqueLabel(getName(), "Restart")))
         {
            stop();
            start();
         }
         ImGui.sameLine();
         if (ImGui.button(ImGuiTools.uniqueLabel(getName(), "Stop")))
         {
            stop();
         }
      }
      else if (starting)
      {
         ImGui.text("Starting...");
      }
      else if (stopping)
      {
         ImGui.text("Stopping...");
      }
      else
      {
         if (ImGui.button(ImGuiTools.uniqueLabel(getName(), "Start")))
         {
            start();
         }
      }
   }

   public void start()
   {
      starting = true;
      executorService.execute(() ->
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
      executorService.execute(() ->
      {
         stopInternal();
         stopping = false;
      });
   }

   @Override
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
      if (!executorService.isShutdown())
         executorService.destroy();
   }
}
