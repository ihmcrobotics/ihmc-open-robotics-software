package us.ihmc.rdx.logging;

import imgui.ImGui;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.perception.RDXBytedecoImagePanel;
import us.ihmc.rdx.ui.RDXBaseUI;

import java.nio.ByteOrder;
import java.util.Random;

public class RDXFFMPEGMP4LoggingDemo
{
   public static final int WIDTH = 256;
   public static final int HEIGHT = 256;
   public static final int NICE_COLOR = 0xFFAA6600;
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXBytedecoImagePanel imagePanel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private BytedecoImage image;
   private final boolean lossless = true;
   private final int framerate = 30;
   private final FFMPEGLoggerDemoHelper ffmpegLoggerDemoHelper = new FFMPEGLoggerDemoHelper(this.getClass().getSimpleName() + ".mp4",
                                                                                            avutil.AV_PIX_FMT_RGBA,
                                                                                            avutil.AV_PIX_FMT_YUV420P,
                                                                                            lossless,
                                                                                            framerate,
                                                                                            400000);
   private final Random random = new Random();
   private final byte[] data = new byte[4];
   private int index = 0;

   public RDXFFMPEGMP4LoggingDemo()
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

            ffmpegLoggerDemoHelper.create(WIDTH, HEIGHT, this::prepareSourceImage);

            image = new BytedecoImage(WIDTH, HEIGHT, opencv_core.CV_8UC4);
            imagePanel = new RDXBytedecoImagePanel("Sample Image", image);

            baseUI.getImGuiPanelManager().addPanel(imagePanel.getImagePanel());
         }

         @Override
         public void render()
         {
            imagePanel.draw();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());
            ImGui.text("Resolution: " + WIDTH + " x " + HEIGHT);

            ffmpegLoggerDemoHelper.renderImGuiBasicInfo();

            ffmpegLoggerDemoHelper.renderImGuiWidgets();
         }

         private void prepareSourceImage()
         {
            if (index % 10 == 0)
               random.nextBytes(data);

            image.getBytedecoOpenCVMat().setTo(new Mat(data));
            for (int i = 0; i < 256; i++)
            {
               boolean d = ((index >> (31 - i / 8)) & 1) == 1;

               for (int j = 0; j < 8; j++)
                  image.getBackingDirectByteBuffer().putInt((WIDTH * j + i) * 4, d ? NICE_COLOR : 0xFF000000);
            }

            index++;

            ffmpegLoggerDemoHelper.getLogger().put(image);
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
      new RDXFFMPEGMP4LoggingDemo();
   }
}