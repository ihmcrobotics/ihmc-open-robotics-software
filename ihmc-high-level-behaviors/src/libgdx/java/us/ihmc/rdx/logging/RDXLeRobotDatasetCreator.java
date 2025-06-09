package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDataset;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDatasetEpisode;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDatasetTools;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.avatar.scs2.SCS2AvatarSimulation;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2LogSession;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.scs2.session.log.LogDataReader;

import java.awt.*;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.Function;
import java.util.function.Supplier;

public class RDXLeRobotDatasetCreator
{
   private final RDXSCS2LogSession logSession;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private transient final ImString datasetName = new ImString(512);
   private transient final ImString imTaskName = new ImString(512);
   private transient final ImInt logPosition = new ImInt();
   private List<Path> datasets;
   private LeRobotDataset dataset;
   private final Queue<Runnable> frameTaskQueue = new ConcurrentLinkedQueue<>();
   private final RDXLeRobotTestSimulator testSimulator;

   public RDXLeRobotDatasetCreator(RDXSCS2LogSession logSession,
                                   RDXBaseUI baseUI,
                                   Function<Pose3DReadOnly, SCS2AvatarSimulation> simulationStarter,
                                   Supplier<KinematicsStreamingToolboxModule> ikStreamingSupplier,
                                   Supplier<DRCRobotModel> robotModelSupplier)
   {
      this.logSession = logSession;

      testSimulator = new RDXLeRobotTestSimulator(simulationStarter, ikStreamingSupplier, robotModelSupplier, baseUI, logSession);

      RDXPanel panel = new RDXPanel("LeRobot Dataset Creator", this::renderImGuiWidgets, false, true);
      panel.addChild(new RDXPanel("Log Scrubber", this::renderLogScrubberWidgets));
      baseUI.getImGuiPanelManager().addPanel(panel);

      // Set recommended settings for generating episodes
      logSession.getSession().setBufferRecordTickPeriod(5);
      logSession.getSession().submitBufferSizeRequest(5000);

      refresh();
   }

   public void update()
   {
      for (int i = 0; i < 50 && !frameTaskQueue.isEmpty(); i++)
         frameTaskQueue.remove().run(); // Run n tasks per frame

      testSimulator.update();
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
         if (logSession.getFirstZEDScrubber() != null)
         {
            ImGui.text("ZED SVO fps: %.3f".formatted(logSession.getFirstZEDScrubber().getFps()));
            if (ImGui.checkbox(labels.get("Record perfect timestamps"), dataset.getUsePerfectTimestamps()))
            {
               dataset.setUsePerfectTimestamps(!dataset.getUsePerfectTimestamps());
            }
         }

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
            dataset.addEpisode(imTaskName.get().trim(), logSession.getSession(), frameTaskQueue::add); // Use queue to maintain framerate
         }
         if (ImGui.button(labels.get("Add Episodes from Boolean")))
         {
            dataset.calculateEpisode(imTaskName.get().trim(), logSession.getSession(), frameTaskQueue::add);
         }
         if (ImGui.button(labels.get("Remove Episode")))
         {
            ImGui.openPopup("Choose Episode to Remove");
         }
         if (ImGui.beginPopup("Choose Episode to Remove"))
         {
            List<LeRobotDatasetEpisode> episodes = dataset.getEpisodes();
            for (int i = 0; i < episodes.size(); i++)
            {
               LeRobotDatasetEpisode episode = episodes.get(i);
               String name = episode.getEpisodeName();
               String selectableLabel = String.format("%d: %s", i, name);
               if (ImGui.selectable(selectableLabel))
               {
                  try
                  {
                     dataset.removeEpisode(i);
                  }
                  catch (IOException e)
                  {
                     throw new RuntimeException(e);
                  }
                  ImGui.closeCurrentPopup();
                  break;
               }
            }
            ImGui.endPopup();
         }
         ImGui.endDisabled();

         if (ImGui.button(labels.get("Regenerate Metadata")))
         {
            dataset.regenerateAndRewriteMetadata();
         }
         if (ImGui.button(labels.get("Write Parquet Data")))
         {
            dataset.writeParquetData();
         }

         testSimulator.renderImGuiWidgets(dataset);
      }
      else
      {
         ImGui.text("Select or create a dataset using the Dataset menu..");
      }
   }

   private void renderLogScrubberWidgets()
   {
      LogDataReader logDataReader = logSession.getSession().getLogDataReader();
      float sliderWidth = ImGui.getColumnWidth();
      ImGui.pushItemWidth(sliderWidth);

      if (dataset != null)
      {
         for (LeRobotDatasetEpisode episode : dataset.getEpisodes())
         {
            var records = episode.getRecords();
            if (!records.isEmpty())
            {
               int episodeStart = records.get(0).ihmcLogPosition();
               ImGuiTools.renderSliderOrProgressNotch((episodeStart / ((float) logDataReader.getNumberOfEntries() - 1)) * sliderWidth, ImGuiTools.DARK_GREEN);
               int episodeEnd = records.get(records.size() - 1).ihmcLogPosition();
               ImGuiTools.renderSliderOrProgressNotch((episodeEnd / ((float) logDataReader.getNumberOfEntries() - 1)) * sliderWidth, ImGuiTools.DARK_RED);
            }
         }
      }

      if (ImGui.sliderInt(labels.getHidden("Log position"), logPosition.getData(), 0, logDataReader.getNumberOfEntries() - 1))
      {
         logSession.getSession().submitLogPositionRequest(logPosition.get());
      }
      else
      {
         logPosition.set(logDataReader.getCurrentLogPosition());
      }
      ImGui.popItemWidth();
   }

   private void refresh()
   {
      datasets = LeRobotDatasetTools.findLeRobotDatasetSubdirectories(logSession.getSession().getLogDataReader().getLogDirectory().toPath());
   }

   public void destroy()
   {
      testSimulator.destroy();
   }

   public RDXLeRobotTestSimulator getTestSimulator()
   {
      return testSimulator;
   }
}
