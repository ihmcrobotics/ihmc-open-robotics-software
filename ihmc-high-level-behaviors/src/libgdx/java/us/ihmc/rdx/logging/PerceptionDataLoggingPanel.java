package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import imgui.type.ImString;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.logging.PerceptionLogChannel;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;

/**
 * Set the `perception.log.directory` JVM property to override the initial log directory.
 * You can also set this in the UI, though.
 */
public class PerceptionDataLoggingPanel extends ImGuiPanel
{
   public static final String PERCEPTION_DIRECTORY_NAME = "perception";
   public static final String PERCEPTION_LOGS_DEFAULT_DIRECTORY
         = System.getProperty("perception.log.directory", IHMCCommonPaths.LOGS_DIRECTORY.resolve(PERCEPTION_DIRECTORY_NAME).toString());

   private PerceptionDataLogger logger;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString perceptionLogPath = new ImString(PERCEPTION_LOGS_DEFAULT_DIRECTORY);
   private final ImInt topicIndex = new ImInt(0);
   private final ImInt objectIndex = new ImInt(0);

   private String[] names;
   private String currentTopic;

   private boolean modified = false;
   private boolean loggerEnabled = false;

   private int numDatasetsInCurrentGroup = 0;
   private int numGroupsInCurrentGroup = 0;

   private final HashMap<String, ImBoolean> channelFlags = new HashMap<>();

   public PerceptionDataLoggingPanel(PerceptionDataLogger logger)
   {
      this("Perception Logger", logger);
   }

   public PerceptionDataLoggingPanel(String panelName, PerceptionDataLogger logger)
   {
      super(panelName);
      setRenderMethod(this::renderImGuiWidgets);
      this.logger = logger;

      for(String channelName : logger.getChannels().keySet())
      {
         channelFlags.put(channelName, new ImBoolean(false));
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Perception Logger");

      for(PerceptionLogChannel channel : logger.getChannels().values())
      {
         ImGui.checkbox(channel.getName() + "\tTotal: " + channel.getCount(), channelFlags.get(channel.getName()));
      }

      ImGuiTools.inputText(labels.get("Log directory"), perceptionLogPath);
      if (ImGui.button(labels.get("Start Logging")))
      {
         modified = true;
         loggerEnabled = true;

         for(String channelName : logger.getChannels().keySet())
         {
            if(channelFlags.get(channelName).get())
            {
               logger.setChannelEnabled(channelName, true);
            }
         }

         SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
         String logFileName = dateFormat.format(new Date()) + "_" + "PerceptionLog.hdf5";
         perceptionLogPath.set(perceptionLogPath.get() + File.separator + logFileName);
         logger.startLogging(perceptionLogPath.get(), "Nadia");
      }

      if(ImGui.button(labels.get("Stop Logging")))
      {
         logger.stopLogging();
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
