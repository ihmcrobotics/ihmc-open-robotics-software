package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDataset;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDatasetTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2LogSession;

import java.io.File;
import java.nio.file.Path;
import java.util.List;

public class RDXLeRobotDatasetCreator
{
   private final RDXSCS2LogSession logSession;
   private final RDXPanel panel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private transient final ImString datasetName = new ImString(512);
   private transient final ImString imTaskName = new ImString(512);
   private List<Path> datasets;
   private LeRobotDataset dataset;

   public RDXLeRobotDatasetCreator(RDXSCS2LogSession logSession)
   {
      this.logSession = logSession;

      panel = new RDXPanel("LeRobot Dataset Creator", this::renderImGuiWidgets, false, true);

      refresh();
   }

   public void update()
   {

   }

   private void renderImGuiWidgets()
   {
      if (ImGui.beginMenuBar())
      {
         if (ImGui.beginMenu(labels.get("Dataset")))
         {
            if (ImGui.menuItem(labels.get("Refresh")))
            {
               refresh();
            }
            ImGui.separator();
            ImGui.text("Datasets:");
            for (Path datasetPath : datasets)
            {
               if (ImGui.radioButton(labels.get(datasetPath.getFileName().toString()), dataset.getDirectory().equals(datasetPath)))
               {
                  if (!dataset.getDirectory().equals(datasetPath))
                     dataset = new LeRobotDataset(datasetPath);
               }
            }
            ImGui.separator();

            ImGui.text("Dataset name:");
            ImGuiTools.inputText(labels.getHidden("datasetName"), datasetName);

            if (ImGui.menuItem(labels.get("Create Dataset")))
            {
               File logDirectory = logSession.getSession().getLogDirectory();
               dataset = new LeRobotDataset(logDirectory.toPath().resolve(datasetName.get().trim()));
               dataset.mkdirs();
               refresh();
            }
            ImGui.endMenu();
         }

         ImGui.endMenuBar();
      }

      if (!logSession.getSession().hasSessionStarted())
      {
         ImGui.text("Session has not started yet.");
         return;
      }

      ImGui.text("Log directory: %s".formatted(logSession.getSession().getLogDataReader().getLogDirectory().getName()));


      if (dataset != null)
      {
         ImGui.text("Task name:");
         ImGuiTools.inputTextMultiline(labels.getHidden("taskName"), imTaskName);

      }
   }

   private Path mkdir(Path path, String name)
   {
      Path newPath = path.resolve(name);
      FileTools.ensureDirectoryExists(newPath, DefaultExceptionHandler.PRINT_MESSAGE);
      return newPath;
   }
   
   private void refresh()
   {
      datasets = LeRobotDatasetTools.findLeRobotDatasetSubdirectories(logSession.getSession().getLogDataReader().getLogDirectory().toPath());
   }

   public RDXPanel getPanel()
   {
      return panel;
   }
}
