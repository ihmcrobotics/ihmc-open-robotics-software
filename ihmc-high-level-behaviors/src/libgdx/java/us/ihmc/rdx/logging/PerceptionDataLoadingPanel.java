package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLogChannel;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import java.io.File;
import java.util.ArrayList;

public class PerceptionDataLoadingPanel extends ImGuiPanel
{
   private PerceptionDataLoader loader;
   private RDXPointCloudRenderer pointCloudRenderer;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString perceptionLogPath = new ImString("/home/quantum/.ihmc/logs/perception/");

   private final ImInt fileIndex = new ImInt(0);
   private final ImInt topicIndex = new ImInt(0);
   private final ImInt objectIndex = new ImInt(0);

   ArrayList<String> logFilesInDirectory = new ArrayList<>();

   private String[] topicNamesArray;
   private String[] fileNamesArray;
   private String currentTopic;

   private boolean modified = false;
   private int numDatasetsInCurrentGroup = 0;
   private int numGroupsInCurrentGroup = 0;

   public PerceptionDataLoadingPanel(PerceptionDataLoader loader)
   {
      this("Perception Loader", loader);
   }

   public PerceptionDataLoadingPanel(String panelName, PerceptionDataLoader loader)
   {
      super(panelName);
      setRenderMethod(this::renderImGuiWidgets);
      this.loader = loader;
   }

   public void setPointCloudRenderer(RDXPointCloudRenderer pointCloudRenderer)
   {
      this.pointCloudRenderer = pointCloudRenderer;
   }

   public void renderImGuiWidgets()
   {
      ImGuiTools.inputText(labels.get("Perception Log"), perceptionLogPath);

      if(ImGui.button(labels.get("Refresh")))
      {
         File file = new File(perceptionLogPath.get());
         File[] logFiles = file.listFiles();
         logFilesInDirectory.clear();
         for(File logFile : logFiles)
         {
            if(logFile.getName().endsWith(".hdf5"))
            {
               logFilesInDirectory.add(logFile.getName());
            }
         }
         fileNamesArray = logFilesInDirectory.toArray(new String[0]);
      }

      if(!logFilesInDirectory.isEmpty())
      {
         ImGui.combo("Log Files", fileIndex, fileNamesArray);
      }

      if (ImGui.button(labels.get("Load log")))
      {
         modified = true;

         loader.openLogFile(perceptionLogPath.get() + fileNamesArray[fileIndex.get()]);

         // Get the topics for the current log file
         ArrayList<String> topicNames = new ArrayList<>();
         loader.getChannels().forEach(channel -> topicNames.add(channel.getName()));
         topicNamesArray = topicNames.toArray(new String[0]);
      }

      if (!loader.getChannels().isEmpty())
      {
         ImGui.text("Loaded Log: " + loader.getFilePath());

         ImGui.text("Number of topics: " + loader.getChannels().size());

         ImGui.combo("Topic Names", topicIndex, topicNamesArray);
         currentTopic = loader.getChannels().get(topicIndex.get()).getName();
         ImGui.text("Total Files: " + loader.getChannels().get(topicIndex.get()).getCount());

         ImGui.sliderInt("Object Index", objectIndex.getData(), 0, 10);

         ArrayList<PerceptionLogChannel> channels = loader.getChannels();
         for (PerceptionLogChannel channel : channels)
         {
            ImGui.text("Channel: " + channel.getName() + " Count: " + channel.getCount() + " Index: " + channel.getIndex());
         }
      }
   }

   public boolean isModified()
   {
      return modified;
   }

   public void setModified(boolean modified)
   {
      this.modified = modified;
   }
}
