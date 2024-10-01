package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import imgui.flag.ImGuiTableColumnFlags;
import imgui.flag.ImGuiTableFlags;
import imgui.type.ImString;
import us.ihmc.avatar.logProcessor.SCS2LogDataProcessor;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;

public class RDXSCS2LogDataProcessor
{
   private final RDXBaseUI baseUI = new RDXBaseUI("RDX Log Data Processor");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ArrayList<Path> logDirectories = new ArrayList<>();
   private final HashMap<Path, SCS2LogDataProcessor> logProcessors = new HashMap<>();
   private final ImString imDirectoryOfLogs;
   private Path directoryOfLogsPath;
   private boolean directoryOfLogsExists;

   public RDXSCS2LogDataProcessor()
   {
      String property = System.getProperty("directory.of.logs");
      imDirectoryOfLogs = new ImString(property == null ? IHMCCommonPaths.LOGS_DIRECTORY.toString() : property, 1000);
      directoryOfLogsPath = Paths.get(imDirectoryOfLogs.get());
      directoryOfLogsExists = Files.exists(directoryOfLogsPath);
      refreshDirectoryListing();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getImGuiPanelManager().addPanel("Main", () ->
            {
               ImGui.text("Directory of logs:");
               ImGui.sameLine();
               if (!directoryOfLogsExists)
                  ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.RED);
               ImGui.inputText(labels.getHidden("directoryOfLogs"), imDirectoryOfLogs);
               if (!directoryOfLogsExists)
                  ImGui.popStyleColor();

               directoryOfLogsExists = Files.exists(directoryOfLogsPath);
               directoryOfLogsPath = Paths.get(imDirectoryOfLogs.get());

               ImGui.beginDisabled(!directoryOfLogsExists);
               if (ImGui.button(labels.get("Refresh")))
                  refreshDirectoryListing();
               ImGui.endDisabled();

               ImGuiTools.separatorText("Logs");

               int tableFlags = ImGuiTableFlags.None;
               tableFlags += ImGuiTableFlags.Resizable;
               tableFlags += ImGuiTableFlags.SizingFixedFit;
               tableFlags += ImGuiTableFlags.Reorderable;
               tableFlags += ImGuiTableFlags.RowBg;
               tableFlags += ImGuiTableFlags.BordersOuter;
               tableFlags += ImGuiTableFlags.BordersV;
               tableFlags += ImGuiTableFlags.NoBordersInBody;

               if (ImGui.beginTable(labels.get("Logs"), 4, tableFlags))
               {
                  float charWidth = ImGuiTools.calcTextSizeX("A");
                  ImGui.tableSetupColumn(labels.get("Name"), ImGuiTableColumnFlags.WidthFixed, 50 * charWidth);
                  ImGui.tableSetupColumn(labels.get("Process"), ImGuiTableColumnFlags.WidthFixed, 15 * charWidth);
                  ImGui.tableSetupColumn(labels.get("Size"), ImGuiTableColumnFlags.WidthFixed, 9 * charWidth);
                  ImGui.tableSetupColumn(labels.get("Footsteps"), ImGuiTableColumnFlags.WidthFixed, 9 * charWidth);

                  ImGui.tableSetupScrollFreeze(0, 1);
                  ImGui.tableHeadersRow();

                  ImGui.tableNextRow();

                  for (Path logDirectory : logDirectories)
                  {
                     SCS2LogDataProcessor logProcessor = logProcessors.get(logDirectory);

                     ImGui.tableNextColumn();
                     ImGui.text(logDirectory.getFileName().toString());

                     ImGui.tableNextColumn();
                     if (logProcessor.isLogValid())
                     {
                        if (logProcessor.isProcessingLog())
                        {
                           ImGui.text("(%.2f%%) %d/%d ".formatted(100.0 * logProcessor.getLogCurrentTick() / (double) logProcessor.getNumberOfEntries(),
                                                                  logProcessor.getLogCurrentTick(),
                                                                  logProcessor.getNumberOfEntries()));
                           ImGui.sameLine();
                           if (ImGuiTools.textWithUnderlineOnHover("Stop") && ImGui.isMouseClicked(ImGuiMouseButton.Left))
                           {
                              logProcessor.gatherStatsAsync();
                           }
                        }
                        else
                        {
                           if (logProcessor.getNumberOfEntries() <= 0)
                           {
                              if (ImGuiTools.textWithUnderlineOnHover("Gather stats") && ImGui.isMouseClicked(ImGuiMouseButton.Left))
                              {
                                 logProcessor.gatherStatsAsync();
                              }
                           }
                           else
                           {
                              if (ImGuiTools.textWithUnderlineOnHover("Process log") && ImGui.isMouseClicked(ImGuiMouseButton.Left))
                              {
                                 logProcessor.processLogAsync();
                              }
                           }
                        }
                     }
                     else
                     {
                        ImGui.textColored(ImGuiTools.RED, "Invalid");
                     }

                     ImGui.tableNextColumn();
                     ImGui.text("" + logProcessor.getNumberOfEntries());
                     ImGui.tableNextColumn();
                     ImGui.text("" + logProcessor.getNumberOfFootstepsStat());
                  }

                  ImGui.endTable();
               }

            });
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   private void refreshDirectoryListing()
   {
      logDirectories.clear();
      try (DirectoryStream<Path> stream = Files.newDirectoryStream(directoryOfLogsPath))
      {
         for (Path path : stream)
         {
            if (Files.exists(path.resolve("robotData.log")))
            {
               SCS2LogDataProcessor logProcessor = logProcessors.get(path);
               if (logProcessor == null)
               {
                  logProcessor = new SCS2LogDataProcessor(path);
                  logProcessors.put(path, logProcessor);
               }
               logDirectories.add(path);
            }
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public static void main(String[] args)
   {
      new RDXSCS2LogDataProcessor();
   }
}
