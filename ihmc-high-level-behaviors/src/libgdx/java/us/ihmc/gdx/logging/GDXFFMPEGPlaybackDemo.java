package us.ihmc.gdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.ffmpeg.ffmpeg;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.perception.GDXCVImagePanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.tools.thread.Activator;

import java.io.File;
import java.nio.ByteOrder;

public class GDXFFMPEGPlaybackDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadNativesOnAThread(opencv_core.class, ffmpeg.class);
   private final File logDirectory = new File(System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator);
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
                  video = new FFMPEGVideoPlaybackManager(logDirectory.listFiles()[0].getAbsolutePath());

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
            if (video != null)
               ImGui.text("Resolution: " + video.getWidth() + " x " + video.getHeight());

            if (ImGui.button("Play"))
               video.play();

            if (ImGui.button("Pause"))
               video.pause();

            ImGui.inputInt("Seek to: ", seekLocation);
            if (ImGui.button("Seek"))
               video.seek(seekLocation.get());
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