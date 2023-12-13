package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.extension.imguifiledialog.ImGuiFileDialog;
import imgui.flag.ImGuiDataType;
import imgui.type.ImLong;
import us.ihmc.commons.thread.Notification;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.perception.RDXBytedecoImagePanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.io.File;
import java.nio.ByteOrder;

/**
 * This demo allows to open video files like MP4, WebM, etc, play them back and scrub through them
 * frame by frame.
 */
public class RDXFFMPEGPlaybackDemo
{
   private final String logDirectory = IHMCCommonPaths.LOGS_DIRECTORY + File.separator;

   // example.webm contains licensing information at attribution.txt in same directory.
   // Used with permission from https://en.wikipedia.org/wiki/File:Schlossbergbahn.webm
   private final WorkspaceResourceFile exampleVideo = new WorkspaceResourceFile(new WorkspaceResourceDirectory(getClass()), "example.webm");

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXBytedecoImagePanel imagePanel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private FFMPEGVideoPlaybackManager video;
   private boolean videoReload;
   private final ImLong currentFrame = new ImLong();
   private final ImLong currentTimebaseUnit = new ImLong();
   private final ResettableExceptionHandlingExecutorService seekThread = MissingThreadTools.newSingleThreadExecutor("WalkPathControlPlanning", true, 1);
   private final Notification frameSeekRequested = new Notification();
   private final Notification timestampSeekRequested = new Notification();
   private volatile boolean seeking = false;
   private double numberOfFramesDouble;
   private long numberOfFrames;
   private double videoDuration;
   private double timeBase;
   private double frameRate;
   private long numberOfFramesAVStream;
   private long startTime;
   private double currentTimestamp;
   private double currentFrameDouble;
   private FFMPEGFileReader fileReader;
   private long duration;
   private int width;
   private int height;
   private double framePeriod;
   private long periodInTimeBase;

   public RDXFFMPEGPlaybackDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            FFMPEGTools.listLicenses();

            baseUI.create();

            RDXPanel panel = new RDXPanel("Image", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);

            loadVideo(exampleVideo.getFilesystemFile().toString());
         }

         @Override
         public void render()
         {
            if (imagePanel != null && !seeking)
               imagePanel.draw();

            // Don't overload the process seeking; throttle it down
            if (frameSeekRequested.poll())
            {
               seeking = true;
               seekThread.clearQueueAndExecute(() ->
               {
                  video.seekFrame(currentFrame.get());
                  seeking = false;
               });
            }
            if (timestampSeekRequested.poll())
            {
               seeking = true;
               seekThread.clearQueueAndExecute(() ->
               {
                  video.seek(currentTimebaseUnit.get());
                  seeking = false;
               });
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();

            if (videoReload)
            {
               videoReload = false;
               baseUI.getImGuiPanelManager().addPanel(imagePanel.getImagePanel());
               baseUI.getLayoutManager().reloadLayout();
            }
         }

         private void loadVideo(String file)
         {
            if (video != null)
            {
               video.close();
            }

            video = new FFMPEGVideoPlaybackManager(file);
            if (imagePanel == null)
               imagePanel = new RDXBytedecoImagePanel("Playback Video", video.getImage());
            else
               imagePanel.resize(video.getImage());
            video.seek(0);
            videoReload = true;

            numberOfFramesDouble = video.calculateNumberOfFrames();
            videoDuration = video.calculateVideoDuration();
            timeBase = video.calculateTimeBaseSeconds();
            frameRate = video.calculateAverageFramerateHz();
            framePeriod = video.calculateVideoFramePeriod();
            numberOfFrames = Math.round(numberOfFramesDouble);
            width = video.getWidth();
            height = video.getHeight();
            periodInTimeBase = Math.round(framePeriod / timeBase);

            fileReader = (FFMPEGFileReader) video.getFile();
            numberOfFramesAVStream = fileReader.getAVStream().nb_frames();
            startTime = fileReader.getStartTime();
            duration = fileReader.getDuration();
         }

         private void renderImGuiWidgets()
         {
            if (ImGui.button(labels.get("Select new file...")))
               ImGuiFileDialog.openDialog("chooseVideo", "Select video file", ".*", logDirectory, "", 1, 0, 0);

            if (ImGuiFileDialog.display("chooseVideo", 0, 400, 200, 4000, 2000))
            {
               if (ImGuiFileDialog.isOk())
                  loadVideo((String) ImGuiFileDialog.getSelection().values().toArray()[0]); // Stupid but necessary

               ImGuiFileDialog.close();
            }

            ImGui.separator();

            if (video != null && !video.isAStream())
            {
               ImGui.text("Resolution: " + width + " x " + height);
               if (!seeking)
               {
                  currentTimestamp = video.getCurrentTimestamp();
                  currentFrameDouble = currentTimestamp / video.calculateVideoFramePeriod();
                  currentFrame.set((long) Math.floor(currentFrameDouble));
                  currentTimebaseUnit.set(video.getCurrentTimestampInMillis());
               }

               ImGui.text(String.format("Framerate: %.1f Hz, Frame period %.2f ms, Video duration: %.3f s, Number of frames: %.3f",
                                        frameRate,
                                        framePeriod,
                                        videoDuration,
                                        numberOfFramesDouble));

               ImGui.text(String.format("ffmpeg: Time base: %.6f s, Start time: %d, Average frame rate: %.3f, Number of frames (if known): %d",
                                        timeBase,
                                        startTime,
                                        frameRate,
                                        numberOfFramesAVStream));

               ImGui.text(String.format("Current frame: %.3f / %.3f", currentFrameDouble, numberOfFramesDouble));
               ImGui.text(String.format("Current timestamp: %.6f s / %.6f s", currentTimestamp, videoDuration));

               // FFMPEG freaks out of you seek to the end but there isn't a great way to fix that
               if (ImGui.sliderScalar(labels.get("Frame number"), ImGuiDataType.U32, currentFrame, 0, numberOfFrames))
               { // location value changed
                  frameSeekRequested.set();
               }
               ImGui.sameLine();
               if (ImGui.button(labels.get("<")))
               {
                  if (currentFrame.get() > 0)
                  {
                     currentFrame.set(currentFrame.get() - 1);
                     frameSeekRequested.set();
                  }
               }
               ImGuiTools.previousWidgetTooltip("Go to previous frame");
               ImGui.sameLine();
               if (ImGui.button(labels.get(">")))
               {
                  if (currentFrame.get() < numberOfFrames)
                  {
                     currentFrame.set(currentFrame.get() + 1);
                     frameSeekRequested.set();
                  }
               }
               ImGuiTools.previousWidgetTooltip("Go to next frame");
               ImGui.sameLine();
               ImGui.pushItemWidth(80.0f);
               if (ImGuiTools.volatileInputLong(labels.get("Seek frame"), currentFrame))
               {
                  frameSeekRequested.set();
               }
               ImGui.popItemWidth();

               if (ImGui.sliderScalar(labels.get("Timebase unit"), ImGuiDataType.U32, currentTimebaseUnit, 0, duration))
               { // location value changed
                  timestampSeekRequested.set();
               }
               ImGui.sameLine();
               if (ImGui.button(labels.get("<")))
               {
                  if (currentTimebaseUnit.get() > 0)
                  {
                     currentTimebaseUnit.set(currentTimebaseUnit.get() - 1);
                     timestampSeekRequested.set();
                  }
               }
               ImGuiTools.previousWidgetTooltip("Go to previous timebase unit");
               ImGui.sameLine();
               if (ImGui.button(labels.get(">")))
               {
                  if (currentTimebaseUnit.get() < duration)
                  {
                     currentTimebaseUnit.set(currentTimebaseUnit.get() + 1);
                     timestampSeekRequested.set();
                  }
               }
               ImGuiTools.previousWidgetTooltip("Go to next timebase unit");
               ImGui.sameLine();
               ImGui.pushItemWidth(80.0f);
               if (ImGuiTools.volatileInputLong(labels.get("Seek timebase unit"), currentTimebaseUnit, periodInTimeBase))
               {
                  timestampSeekRequested.set();
               }
               ImGui.popItemWidth();

               if (!seeking)
               {
                  if (ImGui.button(labels.get("Play")))
                     video.play();
                  ImGui.sameLine();
                  if (ImGui.button(labels.get("Pause")))
                     video.pause();
               }
               else
               {
                  ImGui.text("Seeking...");
               }
            }
            else if (video != null && video.isAStream())
            {
               ImGui.text("Video is a stream.");
            }

            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());
         }

         @Override
         public void dispose()
         {
            video.close();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXFFMPEGPlaybackDemo();
   }
}