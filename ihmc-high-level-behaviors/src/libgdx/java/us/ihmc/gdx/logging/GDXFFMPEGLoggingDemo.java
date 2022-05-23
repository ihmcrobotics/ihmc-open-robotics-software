package us.ihmc.gdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.ffmpeg.avutil.AVRational;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.perception.GDXCVImagePanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.tools.thread.Activator;

import java.io.File;
import java.nio.ByteOrder;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Random;

public class GDXFFMPEGLoggingDemo
{
   public static final int WIDTH = 256;
   public static final int HEIGHT = 256;
   public static final int NICE_COLOR = 0xFFAA6600;
   private final Activator nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/main/resources");
   private GDXCVImagePanel imagePanel;
   private ImPlotFrequencyPlot loggerPutFrequencyPlot;
   private final ImInt framerate = new ImInt(30);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   private static final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;
   private String fileName;
   private BytedecoImage image;
   private volatile boolean logging = false;
   private volatile boolean finalizing = false;

   public GDXFFMPEGLoggingDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            ImGuiPanel panel = new ImGuiPanel("Image", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);

            updateFileName();
         }

         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  loggerPutFrequencyPlot = new ImPlotFrequencyPlot("FFMPEGLogger put (Hz)");

                  image = new BytedecoImage(WIDTH, HEIGHT, opencv_core.CV_8UC4);
                  imagePanel = new GDXCVImagePanel("Sample Image", image);

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

            ImGui.text("File name: " + fileName);
            ImGui.text("Resolution: " + WIDTH + " x " + HEIGHT);
            ImGui.inputInt(labels.get("framerate"), framerate, 1);

            if (nativesLoadedActivator.peek())
            {
               if (!logging)
               {
                  if (!finalizing)
                  {
                     if (ImGui.button(labels.get("Start logging")))
                     {
                        updateFileName();
                        logging = true;
                        ThreadTools.startAThread(this::loggingThread, "FFMPEGLogging");
                     }
                  }
                  else
                  {
                     ImGui.text("Finalizing...");
                  }
               }
               else
               {
                  if (ImGui.button(labels.get("Stop logging")))
                  {
                     logging = false;
                  }
               }

               loggerPutFrequencyPlot.renderImGuiWidgets();
            }
         }

         private void updateFileName()
         {
            fileName = logDirectory + dateFormat.format(new Date()) + "_FFMPEGLoggingDemo.webm";
         }

         private void loggingThread()
         {
            boolean lossless = true;
            FFMPEGLogger logger = new FFMPEGLogger(WIDTH, HEIGHT, lossless, fileName);

            AVRational msBetweenFrames = new AVRational();
            msBetweenFrames.num(1);
            msBetweenFrames.den(framerate.get());

            final Random random = new Random();
            final byte[] data = new byte[4];
            int index = 0;
            finalizing = true;

            while (logging)
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

               loggerPutFrequencyPlot.ping();
               logger.put(image);

               index++;

               // Using an AVRational helps ensure that we calculate fps the same way the logger does
               ThreadTools.sleep((int) (avutil.av_q2d(msBetweenFrames) * 1000));
            }

            ThreadTools.sleepSeconds(2.0);

            logger.close();
            finalizing = false;
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