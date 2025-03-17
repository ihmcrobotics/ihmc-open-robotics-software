package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.robotDataLogger.Camera;
import us.ihmc.scs2.session.SessionMode;
import us.ihmc.scs2.session.log.LogDataReader;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.scs2.session.log.ZEDSVOScrubber;
import us.ihmc.tools.time.DurationFormatter;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.zed.global.zed.*;
import static us.ihmc.zed.global.zed.SL_MEM_CPU;

public class RDXSCS2LogSession extends RDXSCS2Session
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt logPosition = new ImInt();
   private LogSession logSession;
   private LogDataReader logDataReader;
   private final List<Runnable> cameraUpdaters = new ArrayList<>();
   private final List<Runnable> destroyables = new ArrayList<>();

   public RDXSCS2LogSession(RDXBaseUI baseUI)
   {
      super(baseUI);
   }

   public void startSession(String logFilePath, RDXPerceptionVisualizersPanel perceptionVisualizersPanel)
   {
      try
      {
         File file = new File(logFilePath);
         LogTools.info("Loading log: {}", file.toPath().getParent().normalize().toAbsolutePath());
         logSession = new LogSession(file.getParentFile(), null);
         logDataReader = logSession.getLogDataReader();
      }
      catch (IOException e)
      {
         LogTools.error("Failed to load log. {}", e.getMessage());
      }

      if (logSession != null)
      {
         startSession(logSession);

         for (int i = 0; i < logSession.getLogProperties().getCameras().size(); i++)
         {
            Camera camera = logSession.getLogProperties().getCameras().get(i);
            LogTools.info("Found camera: %s".formatted(camera.getName()));

            perceptionVisualizersPanel.addVisualizer(new RDXOpenCVVideoVisualizer(camera.getNameAsString(), camera.getNameAsString(), false));
         }

         for (File zedSensorDatFile : ZEDSVOScrubber.findZEDSensorDatFiles(logSession.getLogDirectory()))
         {
            LogTools.info("Found ZED sensor: %s".formatted(zedSensorDatFile.getName()));

            ZEDSVOScrubber zedSVOScrubber = new ZEDSVOScrubber(zedSensorDatFile);
            logSession.getLogDataReader().getTimestamp().addListener(yoTimestamp -> // TODO: Probably not do it this way
            {
               long timestamp = yoTimestamp.getValueAsLongBits();
               zedSVOScrubber.scrub(timestamp);
            });
            RDXOpenCVVideoVisualizer visualizer = new RDXOpenCVVideoVisualizer(zedSVOScrubber.getName(), zedSVOScrubber.getName(), false);
            cameraUpdaters.add(() ->
            {
               int imageHeight = zedSVOScrubber.getImageHeight();
               int imageWidth = zedSVOScrubber.getImageWidth();

               if (imageHeight > 0 && imageWidth > 0)
               {
                  visualizer.updateImageDimensions(imageWidth, imageHeight);

                  Pointer leftColorImageSlMatPointer = zedSVOScrubber.getLeftColorImageSlMatPointer();
                  Mat imageMat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, // BGRA8
                                         sl_mat_get_ptr(leftColorImageSlMatPointer, SL_MEM_CPU), sl_mat_get_step_bytes(leftColorImageSlMatPointer, SL_MEM_CPU));

                  visualizer.setImage(imageMat);
               }
            });
            destroyables.add(zedSVOScrubber::close);
            perceptionVisualizersPanel.addVisualizer(visualizer);
         }

         for (Runnable onSessionStartedRunnable : getOnSessionStartedRunnables())
         {
            onSessionStartedRunnable.run();
         }
      }
   }

   public void update()
   {
      super.update();

      if (getSession().getActiveMode() == SessionMode.RUNNING || getSession().getActiveMode() == SessionMode.PLAYBACK)
      {
         for (Runnable cameraUpdater : cameraUpdaters)
         {
            cameraUpdater.run();
         }
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

   public void destroy(RDXBaseUI baseUI)
   {
      for (Runnable destroyable : destroyables)
         destroyable.run();

      super.destroy(baseUI);
   }
}
