package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.logging.PerceptionLogChannel;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

import java.util.ArrayList;

public class PerceptionLoggingPanel extends ImGuiPanel
{
   private PerceptionDataLogger logger;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString perceptionLogPath = new ImString();
   private final ImInt topicIndex = new ImInt(0);
   private final ImInt objectIndex = new ImInt(0);

   private String[] names;
   private String currentTopic;

   private boolean modified = false;
   private boolean loggerEnabled = false;

   private int numDatasetsInCurrentGroup = 0;
   private int numGroupsInCurrentGroup = 0;

   public PerceptionLoggingPanel(PerceptionDataLogger logger)
   {
      this("Perception Logger", logger);
   }

   public PerceptionLoggingPanel(String panelName, PerceptionDataLogger logger)
   {
      super(panelName);
      setRenderMethod(this::renderImGuiWidgets);
      this.logger = logger;
   }

   public void renderImGuiWidgets()
   {
      ImGuiTools.inputText(labels.get("Perception Log"), perceptionLogPath);
      if (ImGui.button(labels.get("Start Logging")))
      {
         modified = true;
         loggerEnabled = true;

         logger.startLogging(perceptionLogPath.get(), "Nadia");

         //ArrayList<String> topicNames = new ArrayList<>();
         //logger.getChannels().forEach(channel -> topicNames.add(channel.getName()));
         //names = topicNames.toArray(new String[0]);
      }

      //if (!logger.getChannels().isEmpty())
      //{
      //   ImGui.text("Loaded Log: " + logger.getFilePath());
      //
      //   ImGui.text("Number of topics: " + logger.getChannels().size());
      //
      //   ImGui.combo("Topic Names", topicIndex, names);
      //   currentTopic = logger.getChannels().get(topicIndex.get()).getName();
      //   ImGui.text("Total Files: " + logger.getChannels().get(topicIndex.get()).getCount());
      //
      //   ImGui.sliderInt("Object Index", objectIndex.getData(), 0, 10);
      //
      //   ArrayList<PerceptionLogChannel> channels = logger.getChannels();
      //   for (PerceptionLogChannel channel : channels)
      //   {
      //      ImGui.text("Channel: " + channel.getName() + " Count: " + channel.getCount() + " Index: " + channel.getIndex());
      //   }
      //}
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
