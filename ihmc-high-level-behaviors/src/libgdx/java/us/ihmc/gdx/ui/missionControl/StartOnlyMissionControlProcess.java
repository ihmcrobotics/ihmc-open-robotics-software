package us.ihmc.gdx.ui.missionControl;

import imgui.internal.ImGui;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import static us.ihmc.gdx.imgui.ImGuiTools.uniqueLabel;

public class StartOnlyMissionControlProcess implements MissionControlProcess
{
   private final String name;
   private final Runnable start;

   private final ResettableExceptionHandlingExecutorService executorService;

   public StartOnlyMissionControlProcess(String name, Runnable start)
   {
      this.name = name;
      this.start = start;

      executorService = MissingThreadTools.newSingleThreadExecutor(name + "ManagementThread", true);
   }

   @Override
   public void renderImGuiWidgets()
   {
      ImGui.text(name + ":");
      ImGui.sameLine();
      if (ImGui.button(uniqueLabel(name, "Start")))
      {
         start();
      }
   }

   public void start()
   {
      executorService.execute(start);
   }

   @Override
   public void destroy()
   {
      executorService.destroy(1500);
   }
}
