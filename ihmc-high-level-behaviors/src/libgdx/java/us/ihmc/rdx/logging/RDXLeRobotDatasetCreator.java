package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2LogSession;

import java.io.File;

public class RDXLeRobotDatasetCreator
{
   private final RDXSCS2LogSession logSession;
   private final RDXPanel panel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private transient final ImString datasetName = new ImString(512);
   private transient final ImString imTaskName = new ImString(512);
   private File logDirectory;

   public RDXLeRobotDatasetCreator(RDXSCS2LogSession logSession)
   {
      this.logSession = logSession;

      panel = new RDXPanel("LeRobot Dataset Creator", this::renderImGuiWidgets);



   }

   public void update()
   {
      if (logSession.getSession().hasSessionStarted())
      {
         File newlogDirectory = logSession.getSession().getLogDataReader().getLogDirectory();

         if (newlogDirectory != logDirectory)
         {
            logDirectory = newlogDirectory;


         }
      }
   }

   private void renderImGuiWidgets()
   {
      if (!logSession.getSession().hasSessionStarted())
      {
         ImGui.text("Session has not started yet.");
         return;
      }

      ImGui.text("Log directory: %s".formatted(logSession.getSession().getLogDataReader().getLogDirectory().getName()));

      ImGui.text("Dataset name:");
      ImGui.sameLine();
      ImGuiTools.inputText(labels.getHidden("datasetName"), datasetName);




      ImGui.text("Task name:");
      ImGuiTools.inputTextMultiline(labels.getHidden("taskName"), imTaskName);

      if (ImGui.button(labels.get("Create Dataset")))
      {
         File logDirectory = logSession.getSession().getLogDataReader().getLogDirectory();
      }
   }

   public RDXPanel getPanel()
   {
      return panel;
   }
}
