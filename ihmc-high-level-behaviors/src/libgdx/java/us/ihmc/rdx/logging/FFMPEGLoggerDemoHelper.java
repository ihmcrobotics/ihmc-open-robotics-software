package us.ihmc.rdx.logging;

import imgui.ImGui;
import imgui.type.ImInt;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImPlotFrequencyPlot;
import us.ihmc.log.LogTools;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

import java.io.File;
import java.lang.reflect.Constructor;
import java.text.SimpleDateFormat;
import java.util.Date;

public class FFMPEGLoggerDemoHelper
{
   private int imageWidth;
   private int imageHeight;
   private final String fileSuffix;
   private final String preferredVideoEncoder;
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
   private Class<? extends FFMPEGLogger> loggerClass = FFMPEGLogger.class;

   public FFMPEGLoggerDemoHelper(String fileSuffix, int sourcePixelFormat, int encoderPixelFormat, boolean lossless, int framerate, int bitRate)
   {
      this(fileSuffix, sourcePixelFormat, encoderPixelFormat, lossless, framerate, bitRate, null);
   }

   public FFMPEGLoggerDemoHelper(String fileSuffix,
                                 int sourcePixelFormat,
                                 int encoderPixelFormat,
                                 boolean lossless,
                                 int framerate,
                                 int bitRate,
                                 String preferredVideoEncoder)
   {
      this.fileSuffix = fileSuffix;
      this.sourcePixelFormat = sourcePixelFormat;
      this.encoderPixelFormat = encoderPixelFormat;
      this.lossless = lossless;
      this.framerate.set(framerate);
      this.bitRate = bitRate;
      this.preferredVideoEncoder = preferredVideoEncoder;
      updateFileName();
   }

   public void setLoggerClass(Class<? extends FFMPEGLogger> loggerClass)
   {
      LogTools.debug("loggerClass set to " + loggerClass.getSimpleName());
      this.loggerClass = loggerClass;
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

   public void renderImGuiWidgets()
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
      // Use custom logging class with different behavior than the default if specified.
      // This uses reflection so that it can use an alternate class from another part of the codebase which uses open-robotics-software as a dependency
      // Examples of why this may be done include using different FFMPEG libraries, or introducing a totally different pipeline for video processing
      if (loggerClass != FFMPEGLogger.class)
      {
         try
         {
            LogTools.info("Creating logger of class " + loggerClass.getSimpleName() + " using reflection");
            Constructor<? extends FFMPEGLogger> constructor = loggerClass.getConstructor(int.class,
                                                                                         int.class,
                                                                                         boolean.class,
                                                                                         int.class,
                                                                                         int.class,
                                                                                         int.class,
                                                                                         int.class,
                                                                                         String.class,
                                                                                         String.class);
            logger = constructor.newInstance(imageWidth,
                                             imageHeight,
                                             lossless,
                                             framerate.get(),
                                             bitRate,
                                             sourcePixelFormat,
                                             encoderPixelFormat,
                                             fileName,
                                             preferredVideoEncoder);
         }
         catch (ReflectiveOperationException ex)
         {
            LogTools.error("Failed to use reflection to build logging class. Defaulting to FFMPEGLogger");
            LogTools.error(ex.getStackTrace());
         }
      }

      if (logger == null)
         logger = new FFMPEGFileLogger(imageWidth,
                                       imageHeight,
                                       lossless,
                                       framerate.get(),
                                       bitRate,
                                       sourcePixelFormat,
                                       encoderPixelFormat,
                                       fileName,
                                       preferredVideoEncoder);

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
      logger = null;
      finalizing = false;
   }

   public FFMPEGLogger getLogger()
   {
      return logger;
   }
}
