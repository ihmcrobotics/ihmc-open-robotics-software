package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.OpenCVFrameConverter;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.robotDataLogger.Camera;
import us.ihmc.scs2.session.log.*;
import us.ihmc.tools.time.DurationFormatter;
import us.ihmc.yoVariables.variable.YoLong;

import java.awt.*;
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
   private final ImInt desiredLoadedIndex = new ImInt();
   private final ImFloat zedDelayCompensation = new ImFloat();
   private final Notification scrubAnyway = new Notification();
   private int lastUpdatedLogPosition = -1;
   private SCS2LogSessionWithVideo logSession;
   private YoLong yoTimestamp;
   private LogDataReader logDataReader;

   private record ZEDLogVideo(ZEDSVOScrubber scrubber, RDXOpenCVVideoVisualizer colorVis, RDXOpenCVVideoVisualizer depthVis) { }
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
         logSession = new SCS2LogSessionWithVideo(file.getParentFile(), null);
         logDataReader = logSession.getLogDataReader();
      }
      catch (IOException e)
      {
         LogTools.error("Failed to load log. {}", e.getMessage());
      }

      if (logSession != null)
      {
         startSession(logSession);

         yoTimestamp = logDataReader.getTimestamp();

         for (MagewellScrubber magewellScrubber : logSession.getMagewellScrubbers())
         {
            Camera camera = magewellScrubber.getCamera();
            RDXOpenCVVideoVisualizer visualizer = new RDXOpenCVVideoVisualizer(camera.getNameAsString(), camera.getNameAsString(), false);
            perceptionVisualizersPanel.addVisualizer(visualizer);
            MagewellLogVideo magewellLogVideo = new MagewellLogVideo(magewellScrubber, new OpenCVFrameConverter.ToMat(), visualizer);
            magewellLogVideos.add(magewellLogVideo);
         }
         for (BlackMagicScrubber blackMagicScrubber : logSession.getBlackMagicScrubbers())
         {
            Camera camera = blackMagicScrubber.getCamera();
            RDXOpenCVVideoVisualizer visualizer = new RDXOpenCVVideoVisualizer(camera.getNameAsString(), camera.getNameAsString(), false);
            perceptionVisualizersPanel.addVisualizer(visualizer);
            BlackmagicLogVideo blackmagicLogVideo = new BlackmagicLogVideo(blackMagicScrubber, visualizer);
            blackmagicLogVideos.add(blackmagicLogVideo);
         }
         for (ZEDSVOScrubber zedSVOScrubber : logSession.getZedSVOScrubbers())
         {
            // Color panel
            RDXOpenCVVideoVisualizer colorVis = new RDXOpenCVVideoVisualizer(zedSVOScrubber.getName() + " Color",
                                               zedSVOScrubber.getName() + " Color", false);
            perceptionVisualizersPanel.addVisualizer(colorVis);

            // Depth panel
            RDXOpenCVVideoVisualizer depthVis = new RDXOpenCVVideoVisualizer(zedSVOScrubber.getName() + " Depth",
                                               zedSVOScrubber.getName() + " Depth", false);
            perceptionVisualizersPanel.addVisualizer(depthVis);

            // Add once
            zedLogVideos.add(new ZEDLogVideo(zedSVOScrubber, colorVis, depthVis));
         }

         // Build depth-enabled ZED scrubbers
         File logDir = logSession.getLogDataReader().getLogDirectory();

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
      if (scrubAnyway.poll() || lastUpdatedLogPosition != currentLogPosition)
      {
         lastUpdatedLogPosition = currentLogPosition;

         for (MagewellLogVideo magewellLogVideo : magewellLogVideos)
         {
            Frame frame = magewellLogVideo.scrubber.readVideoFrame(yoTimestamp.getValueAsLongBits());
            if (frame != null)
            {
               magewellLogVideo.visualizer.updateImageDimensions(frame.imageWidth, frame.imageHeight);
               magewellLogVideo.visualizer.setImage(magewellLogVideo.converter.convertToMat(frame), opencv_imgproc.COLOR_BGR2RGBA);
               frame.close();
            }
         }
         for (BlackmagicLogVideo blackmagicLogVideo : blackmagicLogVideos)
         {
            try
            {
               YUVPicture yuvPicture = blackmagicLogVideo.scrubber.readVideoFrame(yoTimestamp.getValueAsLongBits());
               if (yuvPicture != null)
               {
                  blackmagicLogVideo.visualizer.updateImageDimensions(yuvPicture.getWidth(), yuvPicture.getHeight());
                  // TODO: Convert YUVPicture to Mat
                  yuvPicture.delete();
               }
            }
            catch (Exception e)
            {
               LogTools.error(e.getMessage());
            }
         }
         for (ZEDLogVideo zedLogVideo : zedLogVideos)
         {
            synchronized (zedLogVideo.scrubber)
            {
               zedLogVideo.scrubber.scrub(yoTimestamp.getLongValue());

               int h = zedLogVideo.scrubber.getImageHeight();
               int w = zedLogVideo.scrubber.getImageWidth();

               // color
               zedLogVideo.colorVis.updateImageDimensions(w, h);
               Pointer leftPtr = zedLogVideo.scrubber.getLeftColorImageSlMatPointer();
               if (leftPtr != null && !leftPtr.isNull())
               {
                  Mat cBGRA = new Mat(h, w, opencv_core.CV_8UC4,
                                      sl_mat_get_ptr(leftPtr, SL_MEM_CPU),
                                      sl_mat_get_step_bytes(leftPtr, SL_MEM_CPU));
                  zedLogVideo.colorVis.setImage(cBGRA, opencv_imgproc.COLOR_BGR2RGBA);
                  cBGRA.close();
               }

               // depth
               Pointer dPtr = zedLogVideo.scrubber.getDepthSlMatPointer();
               if (dPtr != null && !dPtr.isNull())
               {
                  long step = sl_mat_get_step_bytes(dPtr, SL_MEM_CPU);
                  if (step > 0)
                  {
                     Mat d16 = new Mat(h, w, opencv_core.CV_16UC1,
                                       sl_mat_get_ptr(dPtr, SL_MEM_CPU), step);

                     zedLogVideo.depthVis.updateImageDimensions(w, h);
                     Mat d8 = new Mat();
                     opencv_core.normalize(d16, d8, 0.0, 255.0,
                                           opencv_core.NORM_MINMAX, opencv_core.CV_8UC1, null);
                     opencv_imgproc.cvtColor(d8, zedLogVideo.depthVis.getRGBA8Mat(), opencv_imgproc.COLOR_GRAY2BGRA);
                     zedLogVideo.depthVis.update();
                     d8.close();
                     d16.close();
                  }
               }
            }
         }
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (isSessionThreadRunning())
      {
         File logDirectory = logDataReader.getLogDirectory();
         if (ImGuiTools.textWithUnderlineOnHover("Log directory: %s".formatted(logDirectory.getName())) && ImGui.isMouseClicked(ImGuiMouseButton.Left))
         {
            ExceptionTools.handle(() -> Desktop.getDesktop().open(logDirectory), DefaultExceptionHandler.PRINT_MESSAGE);
         }

         ImGui.pushItemWidth(ImGui.getColumnWidth());
         renderLogScrubberWidgets(labels);
         if (!zedLogVideos.isEmpty())
         {
            ImGui.text("ZED delay compensation:");
            if (ImGui.sliderFloat(labels.getHidden("ZED delay compensation"), zedDelayCompensation.getData(), -5.0f, 5.0f, "%.2f s"))
            {
               getFirstZEDScrubber().getTimestampScrubber().setDelay(Conversions.secondsToNanoseconds(zedDelayCompensation.get()));
               scrubAnyway.set();
            }
         }
         ImGui.popItemWidth();
      }

      super.renderImGuiWidgets();
   }

   public void renderLogScrubberWidgets(ImGuiUniqueLabelMap labels)
   {
      int logPosition = logDataReader.getCurrentLogPosition();
      int loadedIndex = logPosition - 1;
      boolean loadedIndexOutOfBounds = loadedIndex < 0 || loadedIndex >= logDataReader.getNumberOfEntries();
      String format;
      if (loadedIndexOutOfBounds)
      {
         format = "Not loaded.  Size: %d".formatted(logDataReader.getNumberOfEntries());
      }
      else
      {
         format = "Loaded index: %d %s  Size: %d".formatted(loadedIndex,
                                                            DurationFormatter.formatHoursMinutesSecondsMillis(logDataReader.getRelativeTimestamp(loadedIndex)),
                                                            logDataReader.getNumberOfEntries());
      }

      if (ImGui.sliderInt(this.labels.getHidden("Scrubber"), desiredLoadedIndex.getData(), 0, logDataReader.getNumberOfEntries() - 1, format))
      {
         logSession.submitLogPositionRequest(desiredLoadedIndex.get());
      }
      else
      {
         desiredLoadedIndex.set(loadedIndex);
      }
   }

   public void destroy(RDXBaseUI baseUI)
   {
      for (MagewellLogVideo magewellLogVideo : magewellLogVideos)
         magewellLogVideo.visualizer.destroy();
      for (BlackmagicLogVideo blackmagicLogVideo : blackmagicLogVideos)
         blackmagicLogVideo.visualizer.destroy();
      for (ZEDLogVideo zedLogVideo : zedLogVideos) {
         zedLogVideo.colorVis.destroy();
         zedLogVideo.depthVis.destroy();
      }


      session.shutdownSession();

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
   public SCS2LogSessionWithVideo getSession()
   {
      return logSession;
   }
}
