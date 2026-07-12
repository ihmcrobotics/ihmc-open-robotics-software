package us.ihmc.rdx.simulation.scs2;

import static us.ihmc.zed.global.zed.*;

import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import logger_msgs.Camera;
import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.OpenCVFrameConverter;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.scs2.SCS2LogSessionWithVideo;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.log.LogTools;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXImageVisualizer;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.scs2.definition.geometry.ModelFileGeometryDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.scs2.session.log.BlackMagicScrubber;
import us.ihmc.scs2.session.log.LogDataReader;
import us.ihmc.scs2.session.log.MagewellScrubber;
import us.ihmc.scs2.session.log.ZEDSVOScrubber;
import us.ihmc.tools.time.DurationFormatter;
import us.ihmc.yoVariables.variable.YoLong;

import java.awt.*;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

public class RDXSCS2LogSession extends RDXSCS2Session
{
   private final RDXPerceptionVisualizersPanel perceptionVisualizersPanel;
   private final TypedNotification<String> loadLogRequest = new TypedNotification<>();
   private boolean loadingLog = false;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt desiredLoadedIndex = new ImInt();
   private final ImFloat zedDelayCompensation = new ImFloat();
   private final Notification scrubAnyway = new Notification();
   private int lastUpdatedLogPosition = -1;
   private SCS2LogSessionWithVideo logSession;
   private YoLong yoTimestamp;
   private LogDataReader logDataReader;

   private record ZEDLogVideo(ZEDSVOScrubber scrubber, RDXImageVisualizer visualizer) { }
   private final List<ZEDLogVideo> zedLogVideos = new ArrayList<>();
   private record MagewellLogVideo(MagewellScrubber scrubber, OpenCVFrameConverter.ToMat converter, RDXImageVisualizer visualizer) { }
   private final List<MagewellLogVideo> magewellLogVideos = new ArrayList<>();
   private record BlackmagicLogVideo(BlackMagicScrubber scrubber, RDXImageVisualizer visualizer) { }
   private final List<BlackmagicLogVideo> blackmagicLogVideos = new ArrayList<>();

   public RDXSCS2LogSession(RDXBaseUI baseUI, RDXPerceptionVisualizersPanel perceptionVisualizersPanel)
   {
      super(baseUI);

      this.perceptionVisualizersPanel = perceptionVisualizersPanel;
   }

   public void startSession(String logFilePath)
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
         for (RobotDefinition robotDefinition : logSession.getRobotDefinitions())
            for (RigidBodyDefinition allRigidBody : robotDefinition.getAllRigidBodies())
               for (VisualDefinition visualDefinition : allRigidBody.getVisualDefinitions())
                  if (visualDefinition.getGeometryDefinition() instanceof ModelFileGeometryDefinition modelFileGeometryDefinition)
                  {
                     String fileName = modelFileGeometryDefinition.getFileName();
                     if (ClassLoader.getSystemResource(fileName) == null)
                     {
                        File file = new File(fileName);
                        if (!file.isAbsolute() || !file.exists())
                        {
                           // SCS 2 will unzip resources to ~/.ihmc/resources/<robotName>/, we need to repath the definition for RDX to load it
                           String robotName = logSession.getLogProperties().getModel().getNameAsString();
                           File ihmcRobotFile = new File(System.getProperty("user.home"), ".ihmc/resources/" + robotName + "/" + fileName);
                           if (ihmcRobotFile.exists())
                              modelFileGeometryDefinition.setFileName(ihmcRobotFile.getAbsolutePath());
                           else
                              LogTools.error("Could not find model file '{}' on classpath, filesystem, or in ~/.ihmc/{}", fileName, robotName);
                        }
                     }
                  }

         startSession(logSession);

         yoTimestamp = logDataReader.getTimestamp();

         for (MagewellScrubber magewellScrubber : logSession.getMagewellScrubbers())
         {
            Camera camera = magewellScrubber.getCamera();
            RDXImageVisualizer visualizer = new RDXImageVisualizer(camera.getNameAsString(), camera.getNameAsString(), false);
            visualizer.setActive(true);
            perceptionVisualizersPanel.addVisualizer(visualizer);
            MagewellLogVideo magewellLogVideo = new MagewellLogVideo(magewellScrubber, new OpenCVFrameConverter.ToMat(), visualizer);
            magewellLogVideos.add(magewellLogVideo);
         }
         for (BlackMagicScrubber blackMagicScrubber : logSession.getBlackMagicScrubbers())
         {
            Camera camera = blackMagicScrubber.getCamera();
            RDXImageVisualizer visualizer = new RDXImageVisualizer(camera.getNameAsString(), camera.getNameAsString(), false);
            visualizer.setActive(true);
            perceptionVisualizersPanel.addVisualizer(visualizer);
            BlackmagicLogVideo blackmagicLogVideo = new BlackmagicLogVideo(blackMagicScrubber, visualizer);
            blackmagicLogVideos.add(blackmagicLogVideo);
         }
         for (ZEDSVOScrubber zedSVOScrubber : logSession.getZedSVOScrubbers())
         {
            RDXImageVisualizer visualizer = new RDXImageVisualizer(zedSVOScrubber.getName(), zedSVOScrubber.getName(), false);
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
      if (loadLogRequest.poll())
         startSession(loadLogRequest.read());

      super.update();

      if (logSession == null)
         return;

      int currentLogPosition = logDataReader.getCurrentLogPosition();
      if (scrubAnyway.poll() || lastUpdatedLogPosition != currentLogPosition)
      {
         lastUpdatedLogPosition = currentLogPosition;

         for (MagewellLogVideo magewellLogVideo : magewellLogVideos)
         {
            Frame frame = magewellLogVideo.scrubber.readVideoFrame(yoTimestamp.getValueAsLongBits());
            if (frame != null)
            {
               magewellLogVideo.visualizer.setImage(magewellLogVideo.converter.convertToMat(frame), PixelFormat.BGR8);
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

               int imageHeight = zedLogVideo.scrubber.getImageHeight();
               int imageWidth = zedLogVideo.scrubber.getImageWidth();

               Pointer leftColorImageSlMatPointer = zedLogVideo.scrubber.getLeftColorImageSlMatPointer();
               Mat mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, // BGRA8
                                 sl_mat_get_ptr(leftColorImageSlMatPointer, SL_MEM_CPU),
                                 sl_mat_get_step_bytes(leftColorImageSlMatPointer, SL_MEM_CPU));
               zedLogVideo.visualizer.setImage(mat, PixelFormat.BGRA8);
               mat.close();
            }
         }
      }
   }

   @Override
   public void renderImGuiWidgets()
   {
      if (session == null)
      {
         if (loadingLog)
         {
            ImGui.text("Loading log...");
         }
         else if (ImGui.button(labels.get("Open log...")))
         {
            ThreadTools.startAsDaemon(() ->
            {
               loadingLog = true;
               java.awt.Frame frame = new java.awt.Frame();
               FileDialog fileDialog = new FileDialog(frame, "Choose a .log file", FileDialog.LOAD);
               fileDialog.setFile("*.log");
               fileDialog.setVisible(true);
               loadingLog = false;
               String directory = fileDialog.getDirectory();
               String filename = fileDialog.getFile();
               if (directory != null && filename != null)
                  loadLogRequest.set(new File(directory, filename).getAbsolutePath());
            }, "LogFileDialog");
         }

         return;
      }

      loadingLog = false;

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
      for (ZEDLogVideo zedLogVideo : zedLogVideos)
         zedLogVideo.visualizer.destroy();

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
