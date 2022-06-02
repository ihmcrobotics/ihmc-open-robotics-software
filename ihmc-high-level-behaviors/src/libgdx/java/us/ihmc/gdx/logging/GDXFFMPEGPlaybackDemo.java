package us.ihmc.gdx.logging;

import imgui.ImGui;
import org.bytedeco.ffmpeg.ffmpeg;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.perception.GDXCVImagePanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.perception.BytedecoImage;
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
   private BytedecoImage image;
   private FFMPEGVideoPlaybackManager video;

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
                  video.play();

                  image = new BytedecoImage(video.getWidth(), video.getHeight(), opencv_core.CV_8UC4);
                  imagePanel = new GDXCVImagePanel("Playback Video", image);

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
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXFFMPEGPlaybackDemo();
   }
}