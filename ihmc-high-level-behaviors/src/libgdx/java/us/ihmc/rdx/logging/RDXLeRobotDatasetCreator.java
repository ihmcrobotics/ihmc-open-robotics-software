package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDataset;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDatasetEpisode;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDatasetTools;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotEpisodeRecord;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.simulation.scs2.RDXSCS2LogSession;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.log.LogDataReader;
import us.ihmc.sensorProcessing.parameters.HumanoidRobotSensorInformation;

import java.awt.*;
import java.io.File;
import java.nio.file.Path;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * UI to be used alongside a {@link RDXSCS2LogSession} in an RDX application to
 * generate LeRobot datasets from IHMC log data.
 * <p>
 * Part of the {@link LeRobotDataset} generation system from IHMC logs.
 */
public class RDXLeRobotDatasetCreator
{
   private final RDXSCS2LogSession logSession;
   private final HumanoidJointNameMap jointMap;
   private final HumanoidRobotSensorInformation sensorInformation;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private transient final ImString datasetName = new ImString(512);
   private transient final ImInt imTaskID = new ImInt();
   private transient final ImString imTaskName = new ImString(512);
   private transient final ImBoolean removalSelectionMode = new ImBoolean();
   private boolean[] episodesToRemove;
   private List<Path> datasets;
   private LeRobotDataset dataset;
   private BooleanSupplier generating;
   private final ImBoolean keepGenerating = new ImBoolean(false);
   private int mouseHoveringEpisode = -1;

   public RDXLeRobotDatasetCreator(RDXSCS2LogSession logSession,
                                   RDXBaseUI baseUI,
                                   HumanoidJointNameMap jointMap,
                                   HumanoidRobotSensorInformation sensorInformation)
   {
      this.logSession = logSession;
      this.jointMap = jointMap;
      this.sensorInformation = sensorInformation;

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
                     dataset = new LeRobotDataset(datasetPath, jointMap, sensorInformation);
                     dataset.loadData();
                     if (!dataset.getTaskNames().isEmpty())
                        imTaskName.set(dataset.getTaskNames().get(dataset.getTaskNames().size() - 1));
                     ImGui.closeCurrentPopup();
                  }
               }
            }
            ImGui.separator();

            ImGui.text("Dataset name:");
            ImGuiTools.inputText(labels.getHidden("datasetName"), datasetName);

            if (ImGui.menuItem(labels.get("Create Dataset")))
            {
               File logDirectory = logSession.getSession().getLogDirectory();
               dataset = new LeRobotDataset(logDirectory.toPath().resolve(datasetName.get().trim()), jointMap, sensorInformation);
               dataset.mkdirs();
               dataset.writeMetaJson();
               datasetName.clear();
               refresh();
               ImGui.closeCurrentPopup();
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
         }
         ImGui.text("Dataset FPS: %.2f".formatted(dataset.getFps()));

         ImGuiTools.separatorText("New episode");

         ImGui.text("Current task name:");
         ImGuiTools.inputTextMultiline(labels.getHidden("taskName"), imTaskName);

         ImGui.setNextItemWidth(100.0f);
         ImGui.inputInt(labels.get("Auto Scrub Task ID Filter"), imTaskID);

         ImGui.text("Add episode:");
         ImGui.sameLine();
         ImGui.beginDisabled(imTaskName.get().trim().isEmpty() || (generating != null && generating.getAsBoolean()));
         if (ImGui.button(labels.get("From Buffer")))
         {
            dataset.addEpisode(imTaskName.get().trim(), logSession.getSession());
         }
         ImGuiTools.previousWidgetTooltip("Add an episode from the current SCS 2 in/out points.");
         ImGui.sameLine();
         if (ImGui.button(labels.get("Auto Scrub")))
         {
            keepGenerating.set(true);
            generating = dataset.addEpisodesAutomatically(imTaskName.get().trim(), imTaskID.get(), logSession.getSession(), keepGenerating::get);
         }
         ImGui.endDisabled();
         if (keepGenerating.get())
         {
            ImGui.sameLine();
            ImGui.pushStyleColor(ImGuiCol.Button, ImGuiTools.DARK_RED);
            if (ImGui.button(labels.get("X")))
               keepGenerating.set(false);
            ImGui.popStyleColor();
         }
         ImGuiTools.previousWidgetTooltip("Scrub the log from the current position, add the next episode using the demonstrationTaskID variable.");

         boolean noEpisodes = dataset == null || dataset.getEpisodes().isEmpty();
         if (noEpisodes)
            removalSelectionMode.set(false);
         if (removalSelectionMode.get())
         {
            ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.DARK_RED);
            if (ImGui.button(labels.get("Remove Selected Episodes")))
            {
               dataset.removeEpisodes(episodesToRemove);
               removalSelectionMode.set(false);
            }
            ImGui.popStyleColor();
            ImGui.sameLine();
            if (ImGui.button(labels.get("Cancel")))
               removalSelectionMode.set(false);
         }
         else
         {
            ImGui.beginDisabled(noEpisodes);
            if (ImGui.checkbox(labels.get("Remove Episode Mode"), removalSelectionMode))
               episodesToRemove = new boolean[dataset.getEpisodes().size()];
            ImGui.endDisabled();
         }

         if (ImGui.collapsingHeader(labels.get("Debug Operations")))
         {
            if (ImGui.button(labels.get("Regenerate Metadata")))
               dataset.regenerateAndRewriteMetadata();
            ImGui.sameLine();
            if (ImGui.button(labels.get("Write Parquet Data")))
               dataset.writeParquetData();
         }

         ImGuiTools.separatorText("Tasks");
         for (int i = 0; i < dataset.getTaskNames().size(); i++)
         {
            ImGui.text("%d. %s".formatted(i, dataset.getTaskNames().get(i)));
         }

         ImGuiTools.separatorText("Episodes");
         ImGui.beginChild(labels.get("Episodes Scroll Area"));
         mouseHoveringEpisode = -1;
         for (int i = 0; i < dataset.getEpisodes().size(); i++)
         {
            LeRobotDatasetEpisode episode = dataset.getEpisodes().get(i);
            List<LeRobotEpisodeRecord> records = episode.getRecords();
            String text = "%s length: %d".formatted(episode.getEpisodeName(), episode.getLength());

            if (removalSelectionMode.get())
            {
               ImGui.pushStyleColor(ImGuiCol.CheckMark, ImGuiTools.DARK_RED);
               if (ImGuiTools.smallCheckbox(labels.getHidden("RemoveEpisode%d".formatted(i)), episodesToRemove[i]))
                  episodesToRemove[i] = !episodesToRemove[i];
               ImGuiTools.previousWidgetTooltip("Remove this episode.");
               ImGui.popStyleColor();
               ImGui.sameLine();
            }

            boolean mouseHoveringNodeLine = ImGuiTools.isItemHovered(ImGui.getContentRegionAvailX(), ImGui.getTextLineHeight());
            if (mouseHoveringNodeLine)
            {
               mouseHoveringEpisode = i;
               ImGui.textColored(ImGuiTools.GRAY, text);

               if (!records.isEmpty() && ImGui.isMouseClicked(ImGuiMouseButton.Left))
               {
                  ThreadTools.startAsDaemon(() -> // Make it easy to see what the episodes are, just click them to play
                  {
                     logSession.getSession().setSessionMode(SessionMode.PAUSE);
                     ThreadTools.park(0.01);
                     logSession.getSession().submitLogPositionRequest(records.get(0).logPosition());
                     ThreadTools.park(0.01);
                     logSession.getSession().setSessionMode(SessionMode.RUNNING);
                  }, "PlayEpisode");
               }
            }
            else
               ImGui.text(text);
            ImGuiTools.previousWidgetTooltip(
               """
               Buffer index: %d -> %d
               Right click for options.
               """.formatted(records.isEmpty() ? -1 : records.get(0).logPosition(),
                             records.isEmpty() ? -1 : records.get(records.size() - 1).logPosition())
            );

            String popupId = "episode_context_menu_" + i;
            if (ImGui.isItemClicked(ImGuiMouseButton.Right))
               ImGui.openPopup(popupId);
            if (ImGui.beginPopup(popupId))
            {
               if (ImGui.menuItem(labels.get("Close")))
                  ImGui.closeCurrentPopup();
               ImGui.endPopup();
            }
         }
         ImGui.endChild();
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
         float sliderEndOffset = ImGui.getStyle().getGrabMinSize() + 3;
         sliderWidth -= sliderEndOffset; // Line up the notches to the left edge of the slider
         for (int i = 0; i < dataset.getEpisodes().size(); i++)
         {
            LeRobotDatasetEpisode episode = dataset.getEpisodes().get(i);
            var records = episode.getRecords();
            if (!records.isEmpty())
            {
               float verticalExtents = i == mouseHoveringEpisode ? 5.0f : 3.0f;
               float notchWidth = i == mouseHoveringEpisode ? 4.0f : 2.0f;
               int episodeStart = records.get(0).logPosition();
               float x = (episodeStart / ((float) logDataReader.getNumberOfEntries() - 1)) * sliderWidth;
               ImGuiTools.renderSliderOrProgressNotch(x, ImGuiTools.DARK_GREEN, verticalExtents, notchWidth);
               int episodeEnd = records.get(records.size() - 1).logPosition();
               x = (episodeEnd / ((float) logDataReader.getNumberOfEntries() - 1)) * sliderWidth;
               ImGuiTools.renderSliderOrProgressNotch(x, ImGuiTools.DARK_RED, verticalExtents, notchWidth);
            }
         }
      }

      logSession.renderLogScrubberWidgets(labels);
      ImGui.popItemWidth();
   }

   private void refresh()
   {
      datasets = LeRobotDatasetTools.findLeRobotDatasetSubdirectories(logSession.getSession().getLogDataReader().getLogDirectory().toPath());
   }

   public void destroy()
   {

   }
}
