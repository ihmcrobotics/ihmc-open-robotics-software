package us.ihmc.gdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import org.bytedeco.ffmpeg.avutil.AVRational;
import org.bytedeco.ffmpeg.global.avutil;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.perception.BytedecoImage;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.function.Supplier;

public class FFMPEGLoggerDemoHelper
{
   private int imageWidth;
   private int imageHeight;
   private String fileCoreName;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt framerate = new ImInt(30);
   private ImPlotFrequencyPlot loggerPutFrequencyPlot;
   private final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
   private final String logDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;
   private final Stopwatch expectedVideoLengthStopwatch = new Stopwatch();
   private double expectedVideoLength = 0.0;
   private String fileName;
   private volatile boolean logging = false;
   private volatile boolean finalizing = false;
   private FFMPEGLogger logger;
   private Runnable sourceImageInputRunnable;

   public FFMPEGLoggerDemoHelper(String fileCoreName)
   {
      this.fileCoreName = fileCoreName;
      updateFileName();
   }

   public void create(int imageWidth, int imageHeight, Runnable sourceImageInputRunnable)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.sourceImageInputRunnable = sourceImageInputRunnable;

      loggerPutFrequencyPlot = new ImPlotFrequencyPlot("FFMPEGLogger put (Hz)");
   }

   public void renderImGuiBasicInfo()
   {
      ImGui.text("File name: " + fileName);
      ImGui.inputInt(labels.get("framerate"), framerate, 1);
   }

   public void renderImGuiNativesLoaded()
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

      if (logger != null)
      {
         ImGui.text("Format name: " + logger.getFormatName());
         ImGui.text("Codec: " + logger.getCodecLongName());
         ImGui.text("Bit rate: " + logger.getBitRate());
         ImGui.text("Picture group size (GOP): " + logger.getPictureGroupSize());
         ImGui.text("Pixel format: planar YUV 4:2:0, 12bpp, (1 Cr & Cb sample per 2x2 Y samples)");
         ImGui.text("Global header: " + logger.getFormatWantsGlobalHeader());
         ImGui.text("Expected video length: " + FormattingTools.getFormattedDecimal2D(expectedVideoLength) + " s");
      }
   }

   private void updateFileName()
   {
      fileName = logDirectory + dateFormat.format(new Date()) + "_" + fileCoreName + ".webm";
   }

   private void loggingThread()
   {
      boolean lossless = true;
      logger = new FFMPEGLogger(imageWidth, imageHeight, lossless, framerate.get(), fileName);

      finalizing = true;

      expectedVideoLengthStopwatch.start();
      while (logging)
      {
         loggerPutFrequencyPlot.ping();
         sourceImageInputRunnable.run();

         // Using an AVRational helps ensure that we calculate fps the same way the logger does
         ThreadTools.sleep((int) (avutil.av_q2d(logger.getFramePeriod()) * 1000));
      }
      expectedVideoLength = expectedVideoLengthStopwatch.totalElapsed();

      logger.destroy();
      finalizing = false;
   }

   public FFMPEGLogger getLogger()
   {
      return logger;
   }
}
