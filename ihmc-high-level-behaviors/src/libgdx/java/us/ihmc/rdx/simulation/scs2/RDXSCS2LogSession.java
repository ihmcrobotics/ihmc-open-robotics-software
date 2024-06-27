package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.LogVideoLoader;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.log.LogDataReader;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.tools.time.DurationFormatter;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

public class RDXSCS2LogSession extends RDXSCS2Session
{
   private LogSession logSession;
   private LogVideoLoader logVideoLoader;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt logPosition = new ImInt();
   private final ArrayList<RDXOpenCVVideoVisualizer> logVideoVisualizers = new ArrayList<>();
   private LogDataReader logDataReader;
   private LogPropertiesReader logProperties;

   public RDXSCS2LogSession(RDXBaseUI baseUI)
   {
      super(baseUI);
   }

   public RDXSCS2LogSession(RDXBaseUI baseUI, RDXPanel plotManagerParentPanel)
   {
      super(baseUI, plotManagerParentPanel);
   }

   public void startSession(String logFilePath, RDXPerceptionVisualizersPanel perceptionVisualizersPanel)
   {
      logSession = null;
      logDataReader = null;
      logProperties = null;

      for (RDXOpenCVVideoVisualizer logVideoVisualizer : logVideoVisualizers)
      {
         perceptionVisualizersPanel.removeVisualizer(logVideoVisualizer);
         logVideoVisualizer.destroy();
      }
      logVideoVisualizers.clear();

      try
      {
         File file = new File(logFilePath);
         LogTools.info("Loading log: {}", file.toPath().getParent().normalize().toAbsolutePath());
         logSession = new LogSession(file.getParentFile(), null);
         logDataReader = logSession.getLogDataReader();
         logProperties = logSession.getLogProperties();
      }
      catch (IOException e)
      {
         LogTools.error("Failed to load log. {}", e.getMessage());
      }

      if (logSession != null)
      {
         startSession(logSession);


//         imagePanels.add(new RDXOpenCVVideoVisualizer("LoggerCameraView", "NadiaNorth", false));
//         imagePanels.add(new RDXOpenCVVideoVisualizer("LoggerCameraView", "NadiaSouth", false));

         for (Runnable onSessionStartedRunnable : getOnSessionStartedRunnables())
         {
            onSessionStartedRunnable.run();
         }
      }
   }

   public void createLogVideoLoader(String logVideoFilePath, String logVideoTimestampFilePath) throws IOException
   {
      logVideoLoader = new LogVideoLoader(logVideoFilePath, logVideoTimestampFilePath);

//      log
   }

   public void update()
   {
      super.update();

      if (getSession().getActiveMode() == SessionMode.RUNNING)
      {
//         Mat mat = logVideoLoader.loadNextFrameAsOpenCVMat(logSession.getLogDataReader().getTimestamp().getValue());
//
//         if (mat != null)
//         {
//            imagePanels.get(0).setImage(mat);
//         }
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (isSessionThreadRunning())
      {
         int timeQueryTimestamp = MathTools.clamp(logPosition.get(), 0, logDataReader.getNumberOfEntries() - 1);
         String format = "%d/%d %s".formatted(logPosition.get(),
                                              logDataReader.getNumberOfEntries(),
                                              DurationFormatter.formatHoursMinutesSecondsMillis(logDataReader.getRelativeTimestamp(timeQueryTimestamp)));

         if (ImGui.sliderInt(labels.get("Log position"), logPosition.getData(), 0, logDataReader.getNumberOfEntries() - 1, format))
         {
            logSession.submitLogPositionRequest(logPosition.get());

         }
         else
         {
            logPosition.set(logSession.getLogDataReader().getCurrentLogPosition());
         }
      }

      super.renderImGuiWidgets();
   }

   public void destroy()
   {

   }
}
