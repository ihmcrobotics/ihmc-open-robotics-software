package us.ihmc.perception.streaming;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.javacpp.Pointer;
import us.ihmc.log.LogTools;
import us.ihmc.perception.ffmpeg.FFmpegHardwareVideoEncoder;
import us.ihmc.perception.ffmpeg.FFmpegInterruptCallback;
import us.ihmc.perception.ffmpeg.FFmpegSoftwareVideoEncoder;
import us.ihmc.perception.ffmpeg.FFmpegTools;
import us.ihmc.perception.ffmpeg.FFmpegVideoEncoder;

import java.net.InetSocketAddress;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.*;

// TODO: Make abstract SRTStreamer class and extend to video and audio
public class SRTVideoStreamer
{
   private static final String OUTPUT_FORMAT_NAME = "mpegts";
   private static final String PREFERRED_CODEC = "h264_nvenc"; // TODO: use other codec if CUDA not available
   private static final int OUTPUT_PIXEL_FORMAT = AV_PIX_FMT_YUV420P;
   private static final int GOP_SIZE = 5; // send 5 P frames between key frames
   private static final int MAX_B_FRAMES = 0; // don't use B frames

   private final AVDictionary encoderOptions;
   private FFmpegVideoEncoder encoder;

   private final AVOutputFormat outputFormat;
   private final FFmpegInterruptCallback interruptCallback;

   private final Thread callerConnector;
   private final Map<String, String> liveSRTOptions;
   private final String srtAddress;
   private final Set<SRTStreamWriter> callers = ConcurrentHashMap.newKeySet();

   private volatile boolean running = true;
   private boolean initialized = false;

   /**
    * Constructs a streamer, automatically detecting an address and available port
    */
   public SRTVideoStreamer()
   {
      this(StreamingTools.getHostAddress());
   }

   /**
    * Constructs a streamer, specifying the IP address and socket to output the stream on.
    * @param outputAddress IPv4 address and socket to output the stream on.
    */
   public SRTVideoStreamer(InetSocketAddress outputAddress)
   {
      srtAddress = StreamingTools.toSRTAddress(outputAddress);

      interruptCallback = new FFmpegInterruptCallback();

      liveSRTOptions = StreamingTools.getLiveSRTOptions();
      liveSRTOptions.put("mode", "listener");

      encoderOptions = new AVDictionary();
      FFmpegTools.setAVDictionary(encoderOptions, StreamingTools.getH264NVENCStreamingOptions());

      outputFormat = av_guess_format(OUTPUT_FORMAT_NAME, null, null);

      callerConnector = new Thread(this::connectToCallers, getClass().getSimpleName() + "CallerConnector");
      callerConnector.setDaemon(true);
      callerConnector.start();
   }

   /**
    * Initialize the streamer to encode images
    * @param imageWidth Width of images being streamed
    * @param imageHeight Height of images being streamed
    * @param inputAVPixelFormat Pixel format of images being provided (Must be one of AV_PIX_FMT_*)
    */
   public void initialize(int imageWidth,
                          int imageHeight,
                          int inputAVPixelFormat)
   {
      initialize(imageWidth, imageHeight, inputAVPixelFormat, -1, false, false);
   }

   /**
    * Initialize the streamer to encode images
    * @param imageWidth Width of images being streamed
    * @param imageHeight Height of images being streamed
    * @param inputAVPixelFormat Pixel format of images being provided (Must be one of AV_PIX_FMT_*)
    * @param intermediateColorConversion OpenCV color conversion to apply such that the input image matches the inputAVPixelFormat.
    *                                    To not perform a color conversion, pass in a negative value.
    * @param streamLosslessly If true, attempts to stream lossless video. The stream may not be truly lossless as color conversions may introduce error.
    * @param useHardwareAcceleration Whether to use hardware acceleration
    */
   public void initialize(int imageWidth,
                          int imageHeight,
                          int inputAVPixelFormat,
                          int intermediateColorConversion,
                          boolean streamLosslessly,
                          boolean useHardwareAcceleration)
   {
      int bitRate = 10 * imageWidth * imageHeight;

      if (streamLosslessly)
         av_dict_set(encoderOptions, "tune", "lossless", 0);

      if (useHardwareAcceleration)
         encoder = new FFmpegHardwareVideoEncoder(outputFormat,
                                                  PREFERRED_CODEC,
                                                  bitRate,
                                                  imageWidth,
                                                  imageHeight,
                                                  GOP_SIZE,
                                                  MAX_B_FRAMES,
                                                  inputAVPixelFormat);
      else
         encoder = new FFmpegSoftwareVideoEncoder(outputFormat,
                                                  PREFERRED_CODEC,
                                                  bitRate,
                                                  imageWidth,
                                                  imageHeight,
                                                  OUTPUT_PIXEL_FORMAT,
                                                  GOP_SIZE,
                                                  MAX_B_FRAMES,
                                                  inputAVPixelFormat);

      encoder.setIntermediateColorConversion(intermediateColorConversion);
      encoder.initialize(encoderOptions);
      initialized = true;
   }

   public void sendFrame(Pointer frame)
   {
      encoder.setNextFrame(frame);
      encoder.encodeNextFrame(this::writeToCallers);
   }

   private void writeToCallers(AVPacket packetToWrite)
   {
      boolean writeSucceeded;

      Iterator<SRTStreamWriter> callerIterator = callers.iterator();
      while (callerIterator.hasNext())
      {
         SRTStreamWriter callerWriter = callerIterator.next();

         if (!callerWriter.isConnected())
         {
            callerWriter.destroy();
            callerIterator.remove();
            continue;
         }

         writeSucceeded = callerWriter.write(packetToWrite);
         if (!writeSucceeded)
         {
            callerWriter.destroy();
            callerIterator.remove();
         }
      }
   }

   public int connectedCallerCount()
   {
      return callers.size();
   }

   public boolean isInitialized()
   {
      return initialized;
   }

   public boolean isUsingHardwareAcceleration()
   {
      return encoder instanceof FFmpegHardwareVideoEncoder;
   }

   public void destroy()
   {
      LogTools.debug("Shutting down {}", getClass().getSimpleName());
      running = false;

      interruptCallback.interrupt();
      try
      {
         callerConnector.join();
      }
      catch (InterruptedException e)
      {
         interruptCallback.interrupt();
      }

      Iterator<SRTStreamWriter> callerIterator = callers.iterator();
      while (callerIterator.hasNext())
      {
         callerIterator.next().destroy();
         callerIterator.remove();
      }

      av_dict_free(encoderOptions);
      encoderOptions.close();

      if (encoder != null)
         encoder.destroy();

      interruptCallback.close();
   }

   private void connectToCallers()
   {
      LogTools.debug("Starting {} thread on {}", callerConnector.getName(), srtAddress);

      while (running)
      {
         // Set the SRT options
         AVDictionary srtOptions = new AVDictionary();
         FFmpegTools.setAVDictionary(srtOptions, liveSRTOptions);

         // Wait for caller connection
         LogTools.debug("Waiting for connection on {}", srtAddress);
         AVIOContext callerSRTContext = new AVIOContext();
         int error = avio_open2(callerSRTContext, srtAddress, AVIO_FLAG_WRITE, interruptCallback, srtOptions);

         // Check whether connection succeeded
         if (error >= 0)
         {
            // Ensure options are set correctly
            FFmpegTools.checkDictionaryAfterUse(srtOptions);

            LogTools.debug("Got a connection on {}", srtAddress);
            SRTStreamWriter callerWriter = new SRTStreamWriter(encoder, callerSRTContext, outputFormat, null);
            if (callerWriter.startOutput())
               callers.add(callerWriter);
            else
               callerWriter.destroy();
         }

         av_dict_free(srtOptions);
         srtOptions.close();
      }

      LogTools.debug("{} thread listening on {} ended", callerConnector.getName(), srtAddress);
   }
}
