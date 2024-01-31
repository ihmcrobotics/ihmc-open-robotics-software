package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import imgui.type.ImString;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.log.LogTools;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLogChannel;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;

import static org.bytedeco.opencv.global.opencv_highgui.destroyAllWindows;

public class RDXPerceptionDataLoaderPanel extends RDXPanel
{
   private PerceptionDataLoader loader;
   private Mat cvImage;
   private final BytePointer imageBytePointer = new BytePointer(1000000);

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString perceptionLogPath = new ImString(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY + "/");

   private final ImInt fileIndex = new ImInt(0);
   private final ImInt topicIndex = new ImInt(0);

   // TODO: Use these image panels instead of OpenCV image visualization
   private final HashMap<String, RDXOpenCVVideoVisualizer> imagePanels = new HashMap<>();
   ArrayList<String> logFilesInDirectory = new ArrayList<>();

   private String[] topicNamesArray;
   private String[] fileNamesArray;

   private boolean modified = false;

   public RDXPerceptionDataLoaderPanel(PerceptionDataLoader loader)
   {
      this("Perception Loader", loader);
   }

   public RDXPerceptionDataLoaderPanel(String panelName, PerceptionDataLoader loader)
   {
      super(panelName);
      setRenderMethod(this::renderImGuiWidgets);
      this.loader = loader;
   }

   public void renderImGuiWidgets()
   {
      ImGuiTools.inputText(labels.get("Perception Log"), perceptionLogPath);

      if (ImGui.button(labels.get("Refresh")))
      {
         File file = new File(perceptionLogPath.get());
         File[] logFiles = file.listFiles();
         logFilesInDirectory.clear();
         for (File logFile : logFiles)
         {
            if (logFile.getName().endsWith(".hdf5"))
            {
               logFilesInDirectory.add(logFile.getName());
            }
         }
         logFilesInDirectory.sort(String::compareTo);
         fileNamesArray = logFilesInDirectory.toArray(new String[0]);
      }

      if (!logFilesInDirectory.isEmpty())
      {
         ImGui.combo("Log Files", fileIndex, fileNamesArray);
      }

      if (ImGui.button(labels.get("Load log")))
      {
         modified = true;

         loader.openLogFile(perceptionLogPath.get() + fileNamesArray[fileIndex.get()]);

         // Get the topics for the current log file
         ArrayList<String> topicNames = new ArrayList<>();
         loader.getChannels().values().forEach(channel -> topicNames.add(channel.getName()));
         topicNamesArray = topicNames.toArray(new String[0]);
      }

      if (!loader.getChannels().isEmpty())
      {
         ImGui.text("Loaded Log: " + loader.getFilePath());
         ImGui.text("Number of topics: " + loader.getChannels().size());
         ImGui.combo("Topic Names", topicIndex, topicNamesArray);

         HashMap<String, PerceptionLogChannel> channels = loader.getChannels();
         for (PerceptionLogChannel channel : channels.values())
         {
            ImGui.text("Channel: " + channel.getName() + " Count: " + channel.getCount() + " Index: " + channel.getIndex());
            ImGui.sameLine();
            if (ImGui.button("Play"))
            {
               channel.setEnabled(true);
            }
         }
      }

      // TODO: Populate ImGui-GDX image panels based on image channels inside the HDF5
      //      if (modified)
      //      {
      //         imagePanels.clear();
      //         loader.getChannels().keySet().forEach(channelName ->
      //                                      {
      //                                         RDXOpenCVVideoVisualizer imagePanel = new RDXOpenCVVideoVisualizer("Perception Log: " + channelName,
      //                                                                                                            channelName,
      //                                                                                                            false);
      //                                         imagePanels.put(channelName, imagePanel);
      //                                         baseUI.getImGuiPanelManager().addPanel(imagePanel.getPanel());
      //                                      });
      //         modified = false;
      //      }

      for (PerceptionLogChannel channel : loader.getChannels().values())
      {
         if (channel.isEnabled())
         {
            if (channel.getIndex() < channel.getCount())
            {
               // TODO: Replace this with setting images into image panels populated earlier
               if (channel.getName().contains("depth"))
               {
                  cvImage = new Mat();
                  loader.loadCompressedDepth(channel.getName(), channel.getIndex(), imageBytePointer, cvImage);
                  PerceptionDebugTools.displayDepth(channel.getName(), cvImage, 1);
               }
               if (channel.getName().contains("color"))
               {
                  cvImage = new Mat();
                  loader.loadCompressedColor(channel.getName(), channel.getIndex(), cvImage);
                  PerceptionDebugTools.displayDepth(channel.getName(), cvImage, 1);
               }

               // TODO: Maybe replace these with trajectory graphics buffer replay
               if (channel.getName().contains("position"))
               {
                  LogTools.info("Position: ({},{}) -> {}", channel.getName(), channel.getIndex(), 0);
               }
               if (channel.getName().contains("orientation"))
               {
                  LogTools.info("Orientation: ({},{}) -> {}", channel.getName(), channel.getIndex(), 0);
               }

               channel.incrementIndex();
            }
            else
            {
               channel.setEnabled(false);
               channel.resetIndex();
               destroyAllWindows();
            }
         }
      }
   }
}
