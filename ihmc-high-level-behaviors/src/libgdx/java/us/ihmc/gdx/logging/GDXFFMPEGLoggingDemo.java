package us.ihmc.gdx.logging;

import imgui.ImGui;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.perception.GDXCVImagePanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.time.FrequencyCalculator;

import java.nio.ByteOrder;
import java.util.Random;

public class GDXFFMPEGLoggingDemo
{
   public static final int WIDTH = 64;
   public static final int HEIGHT = 64;
   public static final int NICE_COLOR = 0xFFAA6600;
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/main/resources");
   private final Activator nativesLoadedActivator;
   private GDXCVImagePanel imagePanel;
   private final FrequencyCalculator frameReadFrequency = new FrequencyCalculator();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());

   private BytedecoImage image;

   public GDXFFMPEGLoggingDemo()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ImGuiPanel panel = new ImGuiPanel("Image", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);

            image = new BytedecoImage(WIDTH, HEIGHT, opencv_core.CV_8UC4);
            ThreadTools.startAThread(new Runnable()
            {
               final Random rand = new Random();
               final byte[] data = new byte[4];
               int index = 0;
               @Override
               public void run()
               {
                  while (true) {
                     rand.nextBytes(data);

                     if (imagePanel != null)
                     {
                        image.getBytedecoOpenCVMat().setTo(new Mat(data));
                        for (int i = 0; i < 32; i++) {
                           boolean d = ((index >> (31 - i)) & 1) == 1;
                           image.getBackingDirectByteBuffer().putInt(i * 4, d ? NICE_COLOR : 0xFF000000);
                           image.getBackingDirectByteBuffer().putInt((WIDTH + i) * 4, d ? NICE_COLOR : 0xFF000000);
                        }

                        index++;
                     }

                     ThreadTools.sleep(500);
                  }
               }
            }, "ByteDeco Image Updater");
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (imagePanel == null)
               {
                  imagePanel = new GDXCVImagePanel("Sample Image", image);

                  baseUI.getImGuiPanelManager().addPanel(imagePanel.getVideoPanel());
                  baseUI.getPerspectiveManager().reloadPerspective();
               }

               frameReadFrequency.ping();

               if (imagePanel != null)
               {
                  imagePanel.draw();
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());

            if (imagePanel != null)
            {
               ImGui.text("Frame read frequency: " + frameReadFrequency.getFrequency());
            }
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
      new GDXFFMPEGLoggingDemo();
   }
}