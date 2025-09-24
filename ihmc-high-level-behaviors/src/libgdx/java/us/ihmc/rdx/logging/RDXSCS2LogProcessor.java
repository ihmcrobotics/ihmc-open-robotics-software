package us.ihmc.rdx.logging;

import com.github.sh0nk.matplotlib4j.Plot;
import com.github.sh0nk.matplotlib4j.PythonExecutionException;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiMouseButton;
import imgui.flag.ImGuiTableColumnFlags;
import imgui.flag.ImGuiTableFlags;
import imgui.type.ImString;
import org.apache.commons.text.WordUtils;
import us.ihmc.avatar.logProcessor.SCS2LogProcessor;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.IHMCCommonPaths;

import java.awt.*;
import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;

/**
 * Application that allows processing entire folders of logs, plotting data,
 * and doing other post-analysis of logged robot runs for the purpose of data
 * processing for scientific publications and other types of log analysis.
 */
public class RDXSCS2LogProcessor
{
   private final RDXBaseUI baseUI = new RDXBaseUI("RDX Log Processor");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ArrayList<Path> logDirectories = new ArrayList<>();
   private final HashMap<Path, SCS2LogProcessor> logProcessors = new HashMap<>();
   private final ImString imDirectoryOfLogs;
   private Path directoryOfLogsPath;
   private boolean directoryOfLogsExists;
   private int autoProcessIndex = -1;

   public RDXSCS2LogProcessor()
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
               ImGui.sameLine();
               if (!logProcessors.isEmpty())
               {
                  if (autoProcessIndex > -1)
                  {
                     if (ImGui.button(labels.get("Stop Processing")))
                        autoProcessIndex = -1;
                  }
                  else
                  {
                     if (ImGui.button(labels.get("Process All")))
                     {
                        refreshDirectoryListing();
                        autoProcessIndex = 0;
                     }
                  }

                  ImGui.sameLine();
                  if (ImGui.button(labels.get("Plot Linear Traversals")))
                  {
                     ThreadTools.startAsDaemon(() ->
                     {
                        try
                        {
                           Plot pyplot = Plot.create();

                           double maxX = 0.0;
                           for (Path logDirectory : logDirectories)
                           {
                              String logFolderName = logDirectory.getFileName().toString();
                              Path pelvisProgressCSV = logDirectory.resolve(logFolderName + "_PelvisDoorProgress.csv");

                              if (Files.exists(pelvisProgressCSV))
                              {
                                 try (BufferedReader reader = Files.newBufferedReader(pelvisProgressCSV))
                                 {
                                    reader.readLine(); // skip header

                                    ArrayList<Double> xs = new ArrayList<>();
                                    ArrayList<Double> ys = new ArrayList<>();

                                    String line;
                                    while ((line = reader.readLine()) != null)
                                    {
                                       String[] values = line.split(",");

                                       double x = Double.parseDouble(values[0]);
                                       xs.add(x);
                                       ys.add(Double.parseDouble(values[1]));

                                       if (x > maxX)
                                          maxX = x;
                                    }

                                    // Convert Pascal case to title case
                                    String afterUnderscore = logFolderName.substring(logFolderName.lastIndexOf("_") + 1);
                                    String titleCaseString = WordUtils.capitalizeFully(afterUnderscore.replaceAll("([a-z])([A-Z])", "$1 $2"));

                                    pyplot.plot().add(xs, ys).label(titleCaseString);
                                 }
                                 catch (IOException e)
                                 {
                                    throw new RuntimeException("Error reading CSV file.", e);
                                 }
                              }
                           }

                           pyplot.plot().add(Arrays.asList(0.0, maxX), Arrays.asList(0.0, 0.0))
                                 .label("Door Threshold")
                                 .color("gray")
                                 .linestyle("--");

                           pyplot.xlabel("Time (s)");
                           pyplot.ylabel("Pelvis Distance to Frame (m)");
                           pyplot.title("Door Traversal Frame-Relative Distance Progress Over Time");
                           pyplot.legend();
                           pyplot.show();
                        }
                        catch (IOException | PythonExecutionException e)
                        {
                           throw new RuntimeException(e);
                        }
                     }, "Plot");
                  }
               }
               ImGui.endDisabled();

               if (autoProcessIndex >= logDirectories.size())
                  autoProcessIndex = -1;
               if (autoProcessIndex > -1)
               {
                  SCS2LogProcessor logProcessor = logProcessors.get(logDirectories.get(autoProcessIndex));
                  if (!logProcessor.isProcessingLog())
                  {
                     if (logProcessor.getLogCurrentTick() == 0)
                        logProcessor.processLogAsync();
                     else
                        ++autoProcessIndex;
                  }
               }

               ImGuiTools.separatorText("Logs");

               int tableFlags = ImGuiTableFlags.None;
               tableFlags += ImGuiTableFlags.Resizable;
               tableFlags += ImGuiTableFlags.SizingFixedFit;
               tableFlags += ImGuiTableFlags.Reorderable;
               tableFlags += ImGuiTableFlags.RowBg;
               tableFlags += ImGuiTableFlags.BordersOuter;
               tableFlags += ImGuiTableFlags.BordersV;
               tableFlags += ImGuiTableFlags.NoBordersInBody;

               if (ImGui.beginTable(labels.get("Logs"), 11, tableFlags))
               {
                  float charWidth = ImGuiTools.calcTextSizeX("A");
                  ImGui.tableSetupColumn(labels.get("Name"), ImGuiTableColumnFlags.WidthFixed, 50 * charWidth);
                  ImGui.tableSetupColumn(labels.get("Process"), ImGuiTableColumnFlags.WidthFixed, 15 * charWidth);
                  ImGui.tableSetupColumn(labels.get("Size"), ImGuiTableColumnFlags.WidthFixed, 9 * charWidth);
                  ImGui.tableSetupColumn(labels.get("Walks"), ImGuiTableColumnFlags.WidthFixed, 5 * charWidth);
                  ImGui.tableSetupColumn(labels.get("Falls"), ImGuiTableColumnFlags.WidthFixed, 5 * charWidth);
                  ImGui.tableSetupColumn(labels.get("Footsteps"), ImGuiTableColumnFlags.WidthFixed, 9 * charWidth);
                  ImGui.tableSetupColumn(labels.get("Coms"), ImGuiTableColumnFlags.WidthFixed, 9 * charWidth);
                  ImGui.tableSetupColumn(labels.get("workingCounterMismatch"), ImGuiTableColumnFlags.WidthFixed, 9 * charWidth);
                  ImGui.tableSetupColumn(labels.get("Plot hand poses"), ImGuiTableColumnFlags.WidthFixed, 12 * charWidth);
                  ImGui.tableSetupColumn(labels.get("Plot ICP error"), ImGuiTableColumnFlags.WidthFixed, 12 * charWidth);
                  ImGui.tableSetupColumn(labels.get("Plot step timings"), ImGuiTableColumnFlags.WidthFixed, 12 * charWidth);

                  ImGui.tableSetupScrollFreeze(0, 1);
                  ImGui.tableHeadersRow();

                  ImGui.tableNextRow();

                  for (Path logDirectory : logDirectories)
                  {
                     SCS2LogProcessor logProcessor = logProcessors.get(logDirectory);

                     ImGui.tableNextColumn();
                     if (ImGuiTools.textWithUnderlineOnHover(logDirectory.getFileName().toString()) && ImGui.isMouseClicked(ImGuiMouseButton.Left))
                     {
                        ExceptionTools.handle(() -> Desktop.getDesktop().open(logDirectory.toFile()), DefaultExceptionHandler.PRINT_MESSAGE);
                     }

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
                              logProcessor.stopProcessing();
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
                     textIfPositive(logProcessor.getNumberOfEntries());
                     ImGui.tableNextColumn();
                     textIfPositive(logProcessor.getNumberOfWalksStat());
                     ImGui.tableNextColumn();
                     textIfPositive(logProcessor.getNumberOfFallsStat());
                     ImGui.tableNextColumn();
                     textIfPositive(logProcessor.getNumberOfFootstepsStat());
                     ImGui.tableNextColumn();
                     textIfPositive(logProcessor.getNumberOfComsStat());
                     ImGui.tableNextColumn();
                     textIfPositive(logProcessor.getWorkingCounterMismatchStat());
                     ImGui.tableNextColumn();
                     if (ImGuiTools.textWithUnderlineOnHover("Plot hand poses") && ImGui.isMouseClicked(ImGuiMouseButton.Left))
                     {
                        ThreadTools.startAsDaemon(() ->
                        {
                           String logFolderName = logDirectory.getFileName().toString();
                           Path pelvisProgressCSV = logDirectory.resolve(logFolderName + "_LeftArmPoses.csv");

                           if (Files.exists(pelvisProgressCSV))
                           {
                              try
                              {
                                 Plot pyplot = Plot.create();

                                 try (BufferedReader reader = Files.newBufferedReader(pelvisProgressCSV))
                                 {
                                    reader.readLine(); // skip header

                                    ArrayList<Double> times = new ArrayList<>();
                                    ArrayList<Double> xs = new ArrayList<>();
                                    ArrayList<Double> ys = new ArrayList<>();
                                    ArrayList<Double> zs = new ArrayList<>();
                                    ArrayList<Double> yaws = new ArrayList<>();
                                    ArrayList<Double> pitches = new ArrayList<>();
                                    ArrayList<Double> rolls = new ArrayList<>();

                                    String line;
                                    while ((line = reader.readLine()) != null)
                                    {
                                       String[] values = line.split(",");

                                       times.add(Double.parseDouble(values[0]));
                                       xs.add(Double.parseDouble(values[1]));
                                       ys.add(Double.parseDouble(values[2]));
                                       zs.add(Double.parseDouble(values[3]));
                                       yaws.add(Double.parseDouble(values[4]));
                                       pitches.add(Double.parseDouble(values[5]));
                                       rolls.add(Double.parseDouble(values[6]));
                                    }

                                    pyplot.plot().add(times, xs).label("x");
                                    pyplot.plot().add(times, ys).label("y");
                                    pyplot.plot().add(times, zs).label("z");
                                    pyplot.plot().add(times, yaws).label("yaw");
                                    pyplot.plot().add(times, pitches).label("pitch");
                                    pyplot.plot().add(times, rolls).label("roll");
                                 }
                                 catch (IOException e)
                                 {
                                    throw new RuntimeException("Error reading CSV file.", e);
                                 }

                                 // Convert Pascal case to title case
                                 String afterUnderscore = logFolderName.substring(logFolderName.lastIndexOf("_") + 1);
                                 String titleCaseString = WordUtils.capitalizeFully(afterUnderscore.replaceAll("([a-z])([A-Z])", "$1 $2"));

                                 pyplot.xlabel("Time (s)");
//                                 pyplot.ylabel("Pelvis Distance to Frame (m)");
                                 pyplot.title("%s Hand Pose Trajectory".formatted(titleCaseString));
                                 pyplot.legend();
                                 pyplot.show();
                              }
                              catch (IOException | PythonExecutionException e)
                              {
                                 throw new RuntimeException(e);
                              }
                           }
                        }, "Plot Hand Poses");
                     }
                     ImGui.tableNextColumn();
                     if (ImGuiTools.textWithUnderlineOnHover("Plot ICP error") && ImGui.isMouseClicked(ImGuiMouseButton.Left))
                     {
                        ThreadTools.startAsDaemon(() ->
                        {
                           String logFolderName = logDirectory.getFileName().toString();
                           Path csvFile = logDirectory.resolve(logFolderName + "_ICPError.csv");

                           if (Files.exists(csvFile))
                           {
                              try
                              {
                                 Plot pyplot = Plot.create();

                                 try (BufferedReader reader = Files.newBufferedReader(csvFile))
                                 {
                                    reader.readLine(); // skip header


                                 }
                                 catch (IOException e)
                                 {
                                    throw new RuntimeException("Error reading CSV file.", e);
                                 }

                                 // Convert Pascal case to title case
                                 String afterUnderscore = logFolderName.substring(logFolderName.lastIndexOf("_") + 1);
                                 String titleCaseString = WordUtils.capitalizeFully(afterUnderscore.replaceAll("([a-z])([A-Z])", "$1 $2"));

                                 pyplot.xlabel("Time (s)");
                                 pyplot.title("%s ICP Error".formatted(titleCaseString));
                                 pyplot.legend();
                                 pyplot.show();
                              }
                              catch (IOException | PythonExecutionException e)
                              {
                                 throw new RuntimeException(e);
                              }
                           }
                        }, "Plot ICP Error");
                     }
                     ImGui.tableNextColumn();
                     if (ImGuiTools.textWithUnderlineOnHover("Plot step timings") && ImGui.isMouseClicked(ImGuiMouseButton.Left))
                     {
                        ThreadTools.startAsDaemon(() ->
                        {
                           String logFolderName = logDirectory.getFileName().toString();
                           Path csvFile = logDirectory.resolve(logFolderName + "_FootstepStateTimings.csv");

                           if (Files.exists(csvFile))
                           {
                              try
                              {
                                 Plot pyplot = Plot.create();

                                 try (BufferedReader reader = Files.newBufferedReader(csvFile))
                                 {
                                    reader.readLine(); // skip header

                                    ArrayList<Double> times = new ArrayList<>();
                                    ArrayList<Integer> ordinals = new ArrayList<>();

                                    String line;
                                    boolean processingLeft = true;
                                    while ((line = reader.readLine()) != null)
                                    {
                                       String[] values = line.split(",");

                                       if (processingLeft && values[0].equals("right"))
                                       {
                                          pyplot.plot().add(times, ordinals).label("left state");
                                          times.clear();
                                          ordinals.clear();
                                          processingLeft = false;
                                       }

                                       double time = Double.parseDouble(values[1]);
                                       if (!times.isEmpty()) // Make square wave
                                       {
                                          times.add(time - 0.001);
                                          ordinals.add(ordinals.get(ordinals.size() - 1));
                                       }

                                       times.add(time);
                                       ordinals.add(Integer.parseInt(values[2]));
                                    }

                                    pyplot.plot().add(times, ordinals).label("right state");
                                 }
                                 catch (IOException e)
                                 {
                                    throw new RuntimeException("Error reading CSV file.", e);
                                 }

                                 // Convert Pascal case to title case
                                 String afterUnderscore = logFolderName.substring(logFolderName.lastIndexOf("_") + 1);
                                 String titleCaseString = WordUtils.capitalizeFully(afterUnderscore.replaceAll("([a-z])([A-Z])", "$1 $2"));

                                 pyplot.xlabel("Time (s)");
                                 pyplot.title("%s Step Timings".formatted(titleCaseString));
                                 pyplot.legend();
                                 pyplot.show();
                              }
                              catch (IOException | PythonExecutionException e)
                              {
                                 throw new RuntimeException(e);
                              }
                           }
                        }, "Plot Step Timings");
                        ThreadTools.startAsDaemon(() ->
                        {
                           String logFolderName = logDirectory.getFileName().toString();
                           Path swingCSVFile = logDirectory.resolve(logFolderName + "_FootstepSwings.csv");
                           Path transferCSVFile = logDirectory.resolve(logFolderName + "_DoubleSupportDurations.csv");

                           if (Files.exists(swingCSVFile))
                           {
                              try
                              {
                                 Plot pyplot = Plot.create();

                                 ArrayList<Double> swingTimes = new ArrayList<>();
                                 ArrayList<Double> swingDurations = new ArrayList<>();
                                 ArrayList<Double> swingDesireds = new ArrayList<>();
                                 try (BufferedReader reader = Files.newBufferedReader(swingCSVFile))
                                 {
                                    reader.readLine(); // skip header

                                    String line;
                                    boolean processingLeft = true;
                                    while ((line = reader.readLine()) != null)
                                    {
                                       String[] values = line.split(",");

                                       swingTimes.add(Double.parseDouble(values[0]));
                                       swingDurations.add(Double.parseDouble(values[1]));
                                       swingDesireds.add(Double.parseDouble(values[2]));
                                    }

                                 }
                                 catch (IOException e)
                                 {
                                    throw new RuntimeException("Error reading CSV file.", e);
                                 }

                                 ArrayList<Double> transferTimes = new ArrayList<>();
                                 ArrayList<Double> transferDurations = new ArrayList<>();
                                 ArrayList<Double> transferDesireds = new ArrayList<>();
                                 try (BufferedReader reader = Files.newBufferedReader(transferCSVFile))
                                 {
                                    reader.readLine(); // skip header

                                    String line;
                                    while ((line = reader.readLine()) != null)
                                    {
                                       String[] values = line.split(",");

                                       transferTimes.add(Double.parseDouble(values[0]));
                                       transferDurations.add(Double.parseDouble(values[1]));
                                       transferDesireds.add(Double.parseDouble(values[2]));
                                    }

                                 }
                                 catch (IOException e)
                                 {
                                    throw new RuntimeException("Error reading CSV file.", e);
                                 }

                                 // Convert Pascal case to title case
                                 String afterUnderscore = logFolderName.substring(logFolderName.lastIndexOf("_") + 1);
                                 String titleCaseString = WordUtils.capitalizeFully(afterUnderscore.replaceAll("([a-z])([A-Z])", "$1 $2"));

                                 pyplot.plot().add(swingTimes, swingDurations).label("Swing durations");
                                 pyplot.plot().add(swingTimes, swingDesireds).label("Swing desireds");
//                                 pyplot.plot().add(transferTimes, transferDurations).label("Transfer durations");
//                                 pyplot.plot().add(transferTimes, transferDesireds).label("Transfer desireds").color("black");

                                 double plotYMax = swingDurations.stream().max(Double::compare).orElse(0.0);
//                                 plotYMax = Math.max(plotYMax, transferDurations.stream().max(Double::compare).orElse(0.0));
                                 pyplot.ylim(0.0, plotYMax + 0.2);

                                 pyplot.xlabel("Time (s)");
                                 pyplot.title("%s Swing and Transfer Durations".formatted(titleCaseString));
                                 pyplot.legend();
                                 pyplot.show();
                              }
                              catch (IOException | PythonExecutionException e)
                              {
                                 throw new RuntimeException(e);
                              }
                           }
                        }, "Plot Swing Durations");
                        ThreadTools.startAsDaemon(() ->
                        {
                           String logFolderName = logDirectory.getFileName().toString();
                           Path csvFile = logDirectory.resolve(logFolderName + "_ICPError.csv");

                           if (Files.exists(csvFile))
                           {
                              try
                              {
                                 Plot pyplot = Plot.create();

                                 ArrayList<Double> times = new ArrayList<>();
                                 ArrayList<Double> errorXs = new ArrayList<>();
                                 ArrayList<Double> errorYs = new ArrayList<>();
                                 try (BufferedReader reader = Files.newBufferedReader(csvFile))
                                 {
                                    reader.readLine(); // skip header
                                    String line;
                                    while ((line = reader.readLine()) != null)
                                    {
                                       String[] values = line.split(",");

                                       times.add(Double.parseDouble(values[0]));
                                       errorXs.add(Double.parseDouble(values[1]));
                                       errorYs.add(Double.parseDouble(values[2]));
                                    }
                                 }
                                 catch (IOException e)
                                 {
                                    throw new RuntimeException("Error reading CSV file.", e);
                                 }

                                 // Convert Pascal case to title case
                                 String afterUnderscore = logFolderName.substring(logFolderName.lastIndexOf("_") + 1);
                                 String titleCaseString = WordUtils.capitalizeFully(afterUnderscore.replaceAll("([a-z])([A-Z])", "$1 $2"));

                                 pyplot.plot().add(times, errorXs).label("ICP Error X");
                                 pyplot.plot().add(times, errorYs).label("ICP Error Y");

                                 pyplot.xlabel("Time (s)");
                                 pyplot.title("%s ICP Error".formatted(titleCaseString));
                                 pyplot.legend();
                                 pyplot.show();
                              }
                              catch (IOException | PythonExecutionException e)
                              {
                                 throw new RuntimeException(e);
                              }
                           }
                        }, "Plot ICP Error");
                     }
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

   private void textIfPositive(int value)
   {
      if (value > -1)
         ImGui.text("" + value);
   }

   private void refreshDirectoryListing()
   {
      logDirectories.clear();
      logProcessors.clear();
      try (DirectoryStream<Path> stream = Files.newDirectoryStream(directoryOfLogsPath))
      {
         for (Path path : stream)
         {
            if (Files.exists(path.resolve("robotData.log")))
            {
               SCS2LogProcessor logProcessor = logProcessors.get(path);
               if (logProcessor == null)
               {
                  logProcessor = new SCS2LogProcessor(path);
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
      logDirectories.sort(Comparator.comparing(Path::toString));
   }

   public static void main(String[] args)
   {
      new RDXSCS2LogProcessor();
   }
}
