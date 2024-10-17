package us.ihmc.perception.streaming;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.Pointer;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.ffmpeg.FFmpegHardwareVideoEncoder;
import us.ihmc.perception.ffmpeg.FFmpegInterruptCallback;
import us.ihmc.perception.ffmpeg.FFmpegSoftwareVideoEncoder;
import us.ihmc.perception.ffmpeg.FFmpegTools;
import us.ihmc.perception.ffmpeg.FFmpegVideoEncoder;

import java.net.InetSocketAddress;
import java.time.Instant;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.*;

// TODO: Make abstract SRTStreamer class and extend to video and audio
public class SRTVideoStreamer
{
   private static final String DEFAULT_OUTPUT_FORMAT = "mpegts";
   private static final String DEFAULT_CODEC = "hevc_nvenc"; // TODO: use other codec if CUDA not available
   private static final int OUTPUT_PIXEL_FORMAT = AV_PIX_FMT_YUV444P;
   private static final int GOP_SIZE = 5; // send 5 P frames between key frames
   private static final int MAX_B_FRAMES = 0; // don't use B frames

   private FFmpegVideoEncoder encoder;

   private AVOutputFormat outputFormat;
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

   public SRTVideoStreamer(InetSocketAddress outputAddress)
   {
      srtAddress = StreamingTools.toSRTAddress(outputAddress);

      interruptCallback = new FFmpegInterruptCallback();

      liveSRTOptions = StreamingTools.getLiveSRTOptions();
      liveSRTOptions.put("mode", "listener");

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
      initialize(imageWidth,
                 imageHeight,
                 inputAVPixelFormat,
                 OUTPUT_PIXEL_FORMAT,
                 -1,
                 DEFAULT_OUTPUT_FORMAT,
                 DEFAULT_CODEC,
                 StreamingTools.getHEVCNVENCStreamingOptions(),
                 false);
   }

   /**
    * Initialize the streamer to encode images
    * @param imageWidth Width of images being streamed
    * @param imageHeight Height of images being streamed
    * @param inputAVPixelFormat Pixel format of the input being given to the encoder. Must be one of AV_PIX_FMT_*
    * @param outputAVPixelFormat Pixel format of the output generated by the encoder. Must be one of AV_PIX_FMT_*
    * @param intermediateColorConversion OpenCV color conversion to apply such that the input image matches the inputAVPixelFormat.
    *                                    To not perform a color conversion, pass in a negative value.
    * @param outputFormatName Name of the output format to use.
    * @param encoderName Name of the preferred encoder to use.
    * @param encoderOptions Options to use with the encoder. To see a list of options, use {@code ffmpeg --help encoder=[encoder-name]}
    * @param useHardwareAcceleration Whether to use hardware acceleration
    */
   public void initialize(int imageWidth,
                          int imageHeight,
                          int inputAVPixelFormat,
                          int outputAVPixelFormat,
                          int intermediateColorConversion,
                          String outputFormatName,
                          String encoderName,
                          Map<String, String> encoderOptions,
                          boolean useHardwareAcceleration)
   {
      int bitRate = 10 * imageWidth * imageHeight;

      outputFormat = av_guess_format(outputFormatName, null, null);
      FFmpegTools.checkPointer(outputFormat, "Guessing output format");

      AVDictionary encoderOptionsDictionary = new AVDictionary();
      FFmpegTools.setAVDictionary(encoderOptionsDictionary, encoderOptions);

      if (useHardwareAcceleration)
         encoder = new FFmpegHardwareVideoEncoder(outputFormat,
                                                  encoderName,
                                                  bitRate,
                                                  imageWidth,
                                                  imageHeight,
                                                  GOP_SIZE,
                                                  MAX_B_FRAMES,
                                                  inputAVPixelFormat);
      else
         encoder = new FFmpegSoftwareVideoEncoder(outputFormat,
                                                  encoderName,
                                                  bitRate,
                                                  imageWidth,
                                                  imageHeight,
                                                  outputAVPixelFormat,
                                                  GOP_SIZE,
                                                  MAX_B_FRAMES,
                                                  inputAVPixelFormat);

      encoder.setIntermediateColorConversion(intermediateColorConversion);
      encoder.initialize(encoderOptionsDictionary);

      av_dict_free(encoderOptionsDictionary);
      encoderOptionsDictionary.close();

      initialized = true;
   }

   public void sendFrame(Pointer frame, Instant frameAcquisitionTime)
   {
      encoder.setNextFrameAcquisitionTime(frameAcquisitionTime.toEpochMilli());
      encoder.setNextFrame(frame);
      encoder.encodeNextFrame(this::writeToCallers);
   }

   public void sendFrame(Pointer frame, Instant frameAcquisitionTime, BytePointer extraData)
   {
      encoder.setNextFrameAcquisitionTime(frameAcquisitionTime.toEpochMilli());
      encoder.setNextFrameSideData(extraData);
      encoder.setNextFrame(frame);
      encoder.encodeNextFrame(this::writeToCallers);
   }

   public void sendFrame(RawImage frame)
   {
      sendFrame(frame, null);
   }

   public void sendFrame(RawImage frame, BytePointer sideData)
   {
      encoder.setNextFrame(frame);
      if (sideData != null && !sideData.isNull())
         encoder.setNextFrameSideData(sideData);
      encoder.encodeNextFrame(this::writeToCallers);
   }

   private void writeToCallers(AVPacket packetToWrite)
   {
      Iterator<SRTStreamWriter> callerIterator = callers.iterator();
      while (callerIterator.hasNext())
      {
         SRTStreamWriter callerWriter = callerIterator.next();

         if (!callerWriter.write(packetToWrite))
         {
            callerWriter.destroy();
            callerIterator.remove();
         }
      }
   }

   public int connectedCallerCount()
   {
      return (int) callers.stream().filter(SRTStreamWriter::isConnected).count();
   }

   public boolean isInitialized()
   {
      return initialized;
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
            callers.add(new SRTStreamWriter(encoder, callerSRTContext, outputFormat, null));
         }

         av_dict_free(srtOptions);
         srtOptions.close();
      }

      LogTools.debug("{} thread listening on {} ended", callerConnector.getName(), srtAddress);
   }
}
