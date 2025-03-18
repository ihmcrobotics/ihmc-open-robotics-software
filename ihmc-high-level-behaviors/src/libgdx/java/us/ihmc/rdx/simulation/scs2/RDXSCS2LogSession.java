package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.OpenCVFrameConverter;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.commons.MathTools;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.robotDataLogger.Camera;
import us.ihmc.robotDataLogger.CameraType;
import us.ihmc.robotDataLogger.logger.LogPropertiesReader;
import us.ihmc.scs2.session.log.*;
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
   private LogPropertiesReader logProperties;

   private record ZEDLogVideo(ZEDSVOScrubber scrubber, RDXOpenCVVideoVisualizer visualizer) { }
   private final List<ZEDLogVideo> zedLogVideos = new ArrayList<>();
   private record MagewellLogVideo(MagewellScrubber scrubber, OpenCVFrameConverter.ToMat converter, RDXOpenCVVideoVisualizer visualizer) { }
   private final List<MagewellLogVideo> magewellLogVideos = new ArrayList<>();
   private record BlackmagicLogVideo(BlackMagicScrubber scrubber, RDXOpenCVVideoVisualizer visualizer) { }
   private final List<BlackmagicLogVideo> blackmagicLogVideos = new ArrayList<>();

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
         logProperties = logSession.getLogProperties();
      }
      catch (IOException e)
      {
         LogTools.error("Failed to load log. {}", e.getMessage());
      }

      if (logSession != null)
      {
         startSession(logSession);

         yoTimestamp = logDataReader.getTimestamp();

         for (int i = 0; i < logProperties.getCameras().size(); i++)
         {
            Camera camera = logProperties.getCameras().get(i);
            LogTools.info("Found camera: %s".formatted(camera.getName()));
            try
            {
               RDXOpenCVVideoVisualizer visualizer = new RDXOpenCVVideoVisualizer(camera.getNameAsString(), camera.getNameAsString(), false);
               if (camera.getTypeAsString().equals(CameraType.CAPTURE_CARD_MAGEWELL.toString()))
               {
                  MagewellScrubber magewellScrubber = new MagewellScrubber(camera, logSession.getLogDirectory(), logProperties.getVideo().getHasTimebase());
                  MagewellLogVideo magewellLogVideo = new MagewellLogVideo(magewellScrubber, new OpenCVFrameConverter.ToMat(), visualizer);
                  magewellLogVideos.add(magewellLogVideo);
               }
               else
               {
                  BlackMagicScrubber blackMagicScrubber = new BlackMagicScrubber(camera,
                                                                                 logSession.getLogDirectory(),
                                                                                 logProperties.getVideo().getHasTimebase());
                  BlackmagicLogVideo blackmagicLogVideo = new BlackmagicLogVideo(blackMagicScrubber, visualizer);
                  blackmagicLogVideos.add(blackmagicLogVideo);
               }
               perceptionVisualizersPanel.addVisualizer(visualizer);
            }
            catch (IOException e)
            {
               LogTools.error(e.getMessage());
            }
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

      int currentLogPosition = logDataReader.getCurrentLogPosition();
      if (lastUpdatedLogPosition != currentLogPosition)
      {
         lastUpdatedLogPosition = currentLogPosition;

         for (MagewellLogVideo magewellLogVideo : magewellLogVideos)
         {
            Frame frame = magewellLogVideo.scrubber.readVideoFrame(yoTimestamp.getValueAsLongBits());
            magewellLogVideo.visualizer.updateImageDimensions(frame.imageWidth, frame.imageHeight);
            magewellLogVideo.visualizer.setImage(magewellLogVideo.converter.convertToMat(frame), opencv_imgproc.COLOR_BGR2RGBA);
            frame.close();
         }
         for (BlackmagicLogVideo blackmagicLogVideo : blackmagicLogVideos)
         {
            try
            {
               YUVPicture yuvPicture = blackmagicLogVideo.scrubber.readVideoFrame(yoTimestamp.getValueAsLongBits());
               blackmagicLogVideo.visualizer.updateImageDimensions(yuvPicture.getWidth(), yuvPicture.getHeight());
               // TODO: Convert YUVPicture to Mat
               yuvPicture.delete();
            }
            catch (Exception e)
            {
               LogTools.error(e.getMessage());
            }
         }
         for (ZEDLogVideo zedLogVideo : zedLogVideos)
         {
            zedLogVideo.scrubber.scrub(yoTimestamp.getValueAsLongBits());

            int imageHeight = zedLogVideo.scrubber.getImageHeight();
            int imageWidth = zedLogVideo.scrubber.getImageWidth();
            zedLogVideo.visualizer.updateImageDimensions(imageWidth, imageHeight);

            Pointer leftColorImageSlMatPointer = zedLogVideo.scrubber.getLeftColorImageSlMatPointer();
            Mat mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, // BGRA8
                              sl_mat_get_ptr(leftColorImageSlMatPointer, SL_MEM_CPU),
                              sl_mat_get_step_bytes(leftColorImageSlMatPointer, SL_MEM_CPU));
            zedLogVideo.visualizer.setImage(mat);
            mat.close();
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
            logPosition.set(logDataReader.getCurrentLogPosition());
         }
      }

      super.renderImGuiWidgets();
   }

   public void destroy(RDXBaseUI baseUI)
   {
      for (MagewellLogVideo magewellLogVideo : magewellLogVideos)
      {
         magewellLogVideo.scrubber.getMagewellDemuxer().stop();
         magewellLogVideo.visualizer.destroy();
      }
      for (BlackmagicLogVideo blackmagicLogVideo : blackmagicLogVideos)
      {
         blackmagicLogVideo.scrubber.getDemuxer().delete();
         blackmagicLogVideo.visualizer.destroy();
      }
      for (ZEDLogVideo zedLogVideo : zedLogVideos)
      {
         zedLogVideo.scrubber.close();
         zedLogVideo.visualizer.destroy();
      }

      super.destroy(baseUI);
   }

   public ZEDSVOScrubber getFirstZEDScrubber()
   {
      if (zedLogVideos.isEmpty())
         return null;
      else
         return zedLogVideos.get(0).scrubber;
   }

   @Override
   public LogSession getSession()
   {
      return logSession;
   }
}
