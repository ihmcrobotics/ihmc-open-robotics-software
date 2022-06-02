package us.ihmc.gdx.logging;

import imgui.ImGui;
import imgui.extension.imguifiledialog.ImGuiFileDialog;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import org.bytedeco.ffmpeg.ffmpeg;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.perception.GDXCVImagePanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;
import us.ihmc.tools.thread.Activator;

import java.io.File;
import java.nio.ByteOrder;

public class GDXFFMPEGPlaybackDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadNativesOnAThread(opencv_core.class, ffmpeg.class);
   private final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;

   //example.webm contains licensing information at attribution.txt in same directory. Used with permission from https://en.wikipedia.org/wiki/File:Schlossbergbahn.webm
   private final WorkspaceFile exampleVideo = new WorkspaceFile(new WorkspaceDirectory("ihmc-open-robotics-software",
                                                                                       "ihmc-high-level-behaviors/src/main/resources/us/ihmc/gdx/logging"), "example.webm");

   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/main/resources");
   private GDXCVImagePanel imagePanel;
   private GDXCVImagePanel oldImagePanel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private FFMPEGVideoPlaybackManager video;
   private boolean videoReload = false;
   private final ImInt location = new ImInt();
   private boolean isScrubbing = false;

   public GDXFFMPEGPlaybackDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            FFMPEGTools.listLicenses();

            baseUI.create();

            ImGuiPanel panel = new ImGuiPanel("Image", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
                  loadVideo(exampleVideo.getFilePath().toString());
            }

            if (imagePanel != null)
               imagePanel.draw();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();

            if (videoReload)
            {
               if (oldImagePanel != null)
                  baseUI.getImGuiPanelManager().removePanel(oldImagePanel.getVideoPanel());

               baseUI.getImGuiPanelManager().addPanel(imagePanel.getVideoPanel());
               baseUI.getPerspectiveManager().reloadPerspective();

               oldImagePanel = null;
               videoReload = false;
            }
         }

         private void loadVideo(String file)
         {
            if (video != null)
            {
               video.close();
               oldImagePanel = imagePanel;
            }

            video = new FFMPEGVideoPlaybackManager(file);

            imagePanel = new GDXCVImagePanel("Playback Video", video.getImage());

            video.play();

            videoReload = true;
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());

            ImGui.separator();

            if (ImGui.button("Select new file..."))
               ImGuiFileDialog.openDialog("chooseVideo", "Select video file", ".*", logDirectory, "", 1, 0, 0);

            if (ImGuiFileDialog.display("chooseVideo", 0, 400, 200, 4000, 2000))
            {
               if (ImGuiFileDialog.isOk())
                  loadVideo((String) ImGuiFileDialog.getSelection().values().toArray()[0]); //Stupid but necessary

               ImGuiFileDialog.close();
            }

            ImGui.separator();

            if (video != null)
               ImGui.text("Resolution: " + video.getWidth() + " x " + video.getHeight());

            if (video != null)
            {
               //FFMPEG freaks out of you seek to the end but there isn't a great way to fix that
               if (ImGui.sliderInt("##videoProgress", location.getData(), 0, (int) (video.getVideoDurationInMillis())))
               {
                  isScrubbing = true;
                  video.pause();
               }
               else
               {
                  if (isScrubbing)
                  {
                     video.seek(location.get());
                     video.play();

                     isScrubbing = false;
                  }
                  else
                     location.set((int) video.getCurrentTimestampInMillis());
               }

               if (ImGui.button("Play"))
                  video.play();
               ImGui.sameLine();
               if (ImGui.button("Pause"))
                  video.pause();
               ImGui.sameLine();
               ImGui.text(video.getCurrentTimestampInMillis() / 1000 + "s of " + video.getVideoDurationInMillis() / 1000 + 's');
            }
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
      new GDXFFMPEGPlaybackDemo();
   }
}