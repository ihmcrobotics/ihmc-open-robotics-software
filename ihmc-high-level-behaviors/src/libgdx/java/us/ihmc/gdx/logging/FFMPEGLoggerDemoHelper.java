package us.ihmc.gdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.ui.tools.ImPlotFrequencyPlot;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;

public class FFMPEGLoggerDemoHelper
{
   private int imageWidth;
   private int imageHeight;
   private final String fileSuffix;
   private final int sourcePixelFormat;
   private final int encoderPixelFormat;
   private final int bitRate;
   private final boolean lossless;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImInt framerate = new ImInt();
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
   private final Throttler throttler = new Throttler();

   public FFMPEGLoggerDemoHelper(String fileSuffix, int sourcePixelFormat, int encoderPixelFormat, boolean lossless, int framerate, int bitRate)
   {
      this.fileSuffix = fileSuffix;
      this.sourcePixelFormat = sourcePixelFormat;
      this.encoderPixelFormat = encoderPixelFormat;
      this.lossless = lossless;
      this.framerate.set(framerate);
      this.bitRate = bitRate;
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
      fileName = logDirectory + dateFormat.format(new Date()) + "_" + fileSuffix;
   }

   private void loggingThread()
   {
      logger = new FFMPEGLogger(imageWidth, imageHeight, lossless, framerate.get(), bitRate, sourcePixelFormat, encoderPixelFormat, fileName);

      finalizing = true;

      expectedVideoLengthStopwatch.start();
      while (logging)
      {
         throttler.waitAndRun(UnitConversions.hertzToSeconds(framerate.get()));
         loggerPutFrequencyPlot.ping();
         sourceImageInputRunnable.run();
      }
      expectedVideoLength = expectedVideoLengthStopwatch.totalElapsed();

      logger.stop();
      finalizing = false;
   }

   public FFMPEGLogger getLogger()
   {
      return logger;
   }
}
