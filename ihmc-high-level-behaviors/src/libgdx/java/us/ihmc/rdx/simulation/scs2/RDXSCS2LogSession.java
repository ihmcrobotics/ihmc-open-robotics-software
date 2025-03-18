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
import us.ihmc.scs2.session.log.LogDataReader;
import us.ihmc.scs2.session.log.LogSession;
import us.ihmc.scs2.session.log.ZEDSVOScrubber;
import us.ihmc.tools.time.DurationFormatter;
import us.ihmc.yoVariables.variable.YoLong;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

import static us.ihmc.zed.global.zed.*;
import static us.ihmc.zed.global.zed.SL_MEM_CPU;

public class RDXSCS2LogSession extends RDXSCS2Session
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt logPosition = new ImInt();
   private int lastUpdatedLogPosition = -1;
   private LogSession logSession;
   private YoLong yoTimestamp;
   private LogDataReader logDataReader;
   private record ZEDLogVideo(ZEDSVOScrubber scrubber, RDXOpenCVVideoVisualizer visualizer) { }
   private final List<ZEDLogVideo> zedLogVideos = new ArrayList<>();

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
         if (!Files.exists(file.toPath().getParent()))
            throw new RuntimeException("Log folder not found.");
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

         yoTimestamp = logSession.getLogDataReader().getTimestamp();

         for (int i = 0; i < logSession.getLogProperties().getCameras().size(); i++)
         {
            Camera camera = logSession.getLogProperties().getCameras().get(i);
            LogTools.info("Found camera: %s".formatted(camera.getName()));
            // TODO: Add Magewell & Blackmagic scrubbers
            perceptionVisualizersPanel.addVisualizer(new RDXOpenCVVideoVisualizer(camera.getNameAsString(), camera.getNameAsString(), false));
         }

         for (File zedSensorDatFile : ZEDSVOScrubber.findZEDSensorDatFiles(logSession.getLogDirectory()))
         {
            LogTools.info("Found ZED sensor: %s".formatted(zedSensorDatFile.getName()));
            ZEDSVOScrubber zedSVOScrubber = new ZEDSVOScrubber(zedSensorDatFile);
            RDXOpenCVVideoVisualizer visualizer = new RDXOpenCVVideoVisualizer(zedSVOScrubber.getName(), zedSVOScrubber.getName(), false);
            visualizer.setActive(true);
            perceptionVisualizersPanel.addVisualizer(visualizer);
            ZEDLogVideo zedLogVideo = new ZEDLogVideo(zedSVOScrubber, visualizer);
            zedLogVideos.add(zedLogVideo);
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

      int currentLogPosition = logSession.getLogDataReader().getCurrentLogPosition();
      if (lastUpdatedLogPosition != currentLogPosition)
      {
         lastUpdatedLogPosition = currentLogPosition;

         for (ZEDLogVideo zedLogVideo : zedLogVideos)
         {
            zedLogVideo.scrubber.scrub(yoTimestamp.getValueAsLongBits());

            int imageHeight = zedLogVideo.scrubber.getImageHeight();
            int imageWidth = zedLogVideo.scrubber.getImageWidth();
            if (imageHeight > 0 && imageWidth > 0)
            {
               zedLogVideo.visualizer.updateImageDimensions(imageWidth, imageHeight);

               Pointer leftColorImageSlMatPointer = zedLogVideo.scrubber.getLeftColorImageSlMatPointer();
               Mat imageMat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, // BGRA8
                                      sl_mat_get_ptr(leftColorImageSlMatPointer, SL_MEM_CPU), sl_mat_get_step_bytes(leftColorImageSlMatPointer, SL_MEM_CPU));

               zedLogVideo.visualizer.setImage(imageMat);
            }
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
      for (ZEDLogVideo zedLogVideo : zedLogVideos)
      {
         zedLogVideo.scrubber.close();
         zedLogVideo.visualizer.destroy();
      }

      super.destroy(baseUI);
   }
}
