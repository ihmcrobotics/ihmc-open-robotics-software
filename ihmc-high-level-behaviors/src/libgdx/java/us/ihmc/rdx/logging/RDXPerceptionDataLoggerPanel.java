package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.logging.PerceptionLogChannel;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;

/**
 * Set the `perception.log.directory` JVM property to override the initial log directory.
 * You can also set this in the UI, though.
 */
public class RDXPerceptionDataLoggerPanel extends RDXPanel
{
   public static final String PERCEPTION_DIRECTORY_NAME = "perception";
   public static final String PERCEPTION_LOGS_DEFAULT_DIRECTORY
         = System.getProperty("perception.log.directory", IHMCCommonPaths.LOGS_DIRECTORY.resolve(PERCEPTION_DIRECTORY_NAME).toString());

   private PerceptionDataLogger logger;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImString perceptionLogPath = new ImString(PERCEPTION_LOGS_DEFAULT_DIRECTORY);

   private boolean modified = false;
   private boolean loggerEnabled = false;

   private final HashMap<String, ImBoolean> channelFlags = new HashMap<>();

   public RDXPerceptionDataLoggerPanel(PerceptionDataLogger logger)
   {
      this("Perception Logger", logger);
   }

   public RDXPerceptionDataLoggerPanel(String panelName, PerceptionDataLogger logger)
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
         FileTools.ensureDirectoryExists(Paths.get(perceptionLogPath.get()), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         logger.startLogging(perceptionLogPath.get() + File.separator + logFileName, "Nadia");
      }

      if(ImGui.button(labels.get("Stop Logging")))
      {
         logger.stopLogging();
      }
   }

   public void destroy()
   {
      logger.stopLogging();
   }
}
