package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.log.LogTools;
import us.ihmc.perception.logging.PerceptionDataLoader;
import us.ihmc.perception.logging.PerceptionLogChannel;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import java.nio.FloatBuffer;
import java.util.ArrayList;

public class PerceptionLogLoaderPanel extends ImGuiPanel
{
   private PerceptionDataLoader loader;
   private RDXPointCloudRenderer pointCloudRenderer;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString perceptionLogPath = new ImString();
   private final ImInt topicIndex = new ImInt(0);
   private final ImInt objectIndex = new ImInt(0);

   private String[] names;
   private String currentTopic;

   private boolean modified = false;
   private int numDatasetsInCurrentGroup = 0;
   private int numGroupsInCurrentGroup = 0;

   public PerceptionLogLoaderPanel(PerceptionDataLoader loader)
   {
      this("Perception Logging and Loading", loader);
   }

   public PerceptionLogLoaderPanel(String panelName, PerceptionDataLoader loader)
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
      if (ImGui.button(labels.get("Load log")))
      {
         modified = true;
         loader.openLogFile(perceptionLogPath.get());
         ArrayList<String> topicNames = new ArrayList<>();
         loader.getChannels().forEach(channel -> topicNames.add(channel.getName()));
         names = topicNames.toArray(new String[0]);
      }

      if (!loader.getChannels().isEmpty())
      {
         ImGui.text("Loaded Log: " + loader.getFilePath());

         ImGui.text("Number of topics: " + loader.getChannels().size());

         ImGui.combo("Topic Names", topicIndex, names);
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
