package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImString;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDataset;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDatasetEpisode;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDatasetTools;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2LogSession;

import java.awt.*;
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
               if (ImGui.radioButton(labels.get(datasetPath.getFileName().toString()), dataset != null && dataset.getDirectory().equals(datasetPath)))
               {
                  if (dataset == null || !dataset.getDirectory().equals(datasetPath))
                  {
                     dataset = new LeRobotDataset(datasetPath);
                     dataset.loadData();
                     if (!dataset.getTaskNames().isEmpty())
                        imTaskName.set(dataset.getTaskNames().get(dataset.getTaskNames().size() - 1));
                  }
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
               dataset.writeMetaJson();
               datasetName.clear();
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

      if (dataset != null)
      {
         ImGuiTools.separatorText("Selected dataset");

         if (ImGuiTools.textWithUnderlineOnHover("Dataset name: %s".formatted(dataset.getName())) && ImGui.isMouseClicked(ImGuiMouseButton.Left))
            ExceptionTools.handle(() -> Desktop.getDesktop().open(dataset.getDirectory().toFile()), DefaultExceptionHandler.PRINT_MESSAGE);

         ImGui.text("Total frames: %d".formatted(dataset.getTotalFrames()));

         ImGui.separator();
         ImGui.text("Tasks:");
         for (int i = 0; i < dataset.getTaskNames().size(); i++)
         {
            ImGui.text("%d. %s".formatted(i, dataset.getTaskNames().get(i)));
         }

         ImGui.separator();
         ImGui.text("Episodes:");
         for (LeRobotDatasetEpisode episode : dataset.getEpisodes())
         {
            ImGui.text("%s length: %d".formatted(episode.getEpisodeName(), episode.getLength()));
         }

         ImGuiTools.separatorText("New episode");

         ImGui.text("Current task name:");
         ImGuiTools.inputTextMultiline(labels.getHidden("taskName"), imTaskName);

         ImGui.text("Episodes are created for the current SCS 2 in/out points.");
         ImGui.beginDisabled(imTaskName.get().trim().isEmpty());
         if (ImGui.button(labels.get("Add Episode")))
         {
            dataset.addEpisode(imTaskName.get().trim(), logSession.getSession());
         }
         ImGui.endDisabled();

         if (ImGui.button(labels.get("Regenerate Metadata")))
         {
            dataset.regenerateAndRewriteMetadata();
         }
      }
      else
      {
         ImGui.text("Select or create a dataset using the Dataset menu..");
      }
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
