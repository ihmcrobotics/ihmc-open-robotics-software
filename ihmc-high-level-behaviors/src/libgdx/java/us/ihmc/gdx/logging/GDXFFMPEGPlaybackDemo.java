package us.ihmc.gdx.logging;

import imgui.ImGui;
import imgui.extension.imguifiledialog.ImGuiFileDialog;
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
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private FFMPEGVideoPlaybackManager video;
   private final ImInt seekLocation = new ImInt();

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
               {
                  video = new FFMPEGVideoPlaybackManager(exampleVideo.getFilePath().toString());

                  imagePanel = new GDXCVImagePanel("Playback Video", video.getImage());

                  video.play();

                  baseUI.getImGuiPanelManager().addPanel(imagePanel.getVideoPanel());
                  baseUI.getPerspectiveManager().reloadPerspective();
               }

               imagePanel.draw();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());

            ImGui.separator();

            ImGui.text("TODO put file dialog here");

            ImGui.separator();

            if (video != null)
               ImGui.text("Resolution: " + video.getWidth() + " x " + video.getHeight());

            if (video != null)
            {
               ImGui.progressBar(video.getCurrentTimestampInMillis() / (float) video.getVideoDurationInMillis());

               if (ImGui.button("Play"))
                  video.play();
               ImGui.sameLine();
               if (ImGui.button("Pause"))
                  video.pause();
               ImGui.sameLine();
               ImGui.text(video.getCurrentTimestampInMillis() / 1000 + "s of " + video.getVideoDurationInMillis() / 1000 + 's');

               ImGui.inputInt("##seekBox", seekLocation);
               ImGui.sameLine();
               if (ImGui.button("Seek"))
                  video.seek(seekLocation.get() * 1000);
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