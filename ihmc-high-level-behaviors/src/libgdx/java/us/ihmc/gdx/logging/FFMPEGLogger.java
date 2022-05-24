package us.ihmc.gdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avcodec.AVCodecContext;
import org.bytedeco.ffmpeg.avcodec.AVCodecParameters;
import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVFrame;

import org.bytedeco.ffmpeg.avutil.AVRational;
import org.bytedeco.ffmpeg.global.avcodec;
import org.bytedeco.ffmpeg.global.avformat;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.ffmpeg.global.swscale;
import org.bytedeco.ffmpeg.swscale.SwsContext;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.Pointer;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;

import java.nio.file.Paths;

/**
 * Doxygen:
 * https://ffmpeg.org/doxygen/trunk/index.html
 * Wiki:
 * https://trac.ffmpeg.org/
 */
public class FFMPEGLogger
{
   private int videoWidth;
   private int videoHeight;
   private final String fileName;
   private final String formatName;
   private final int bitRate = 400000;
   private final AVRational framePeriod;
   private final int pictureGroupSize = 12;
   private final int avPixelFormat = avutil.AV_PIX_FMT_YUV420P;
   private final boolean formatWantsGlobalHeader;
   private boolean isInitialized = false;
   private final AVDictionary avDictionary;
   private final AVFormatContext avFormatContext;
   private String codecLongName;
   private AVPacket tempAVPacket;
   private AVStream avStream;
   private AVCodecContext avEncoderContext;
   private AVFrame avFrame;
   private AVFrame tempAVFrame;
   private SwsContext swsContext;
   private int nextPts;
   private AVFrame rgbTempFrame;
   private SwsContext rgbSwsContext;

   public FFMPEGLogger(int videoWidth, int videoHeight, boolean lossless, int framerate, String fileName)
   {
      this.videoWidth = videoWidth;
      this.videoHeight = videoHeight;
      this.fileName = fileName;
      formatName = "webm";

      LogTools.info("Initializing ffmpeg contexts for {} output to {}", formatName, fileName);

      int avDictFlags = 0;
      avDictionary = new AVDictionary();
      avutil.av_dict_set(avDictionary, "lossless", lossless ? "1" : "0", avDictFlags); // TODO this is maybe wrong

      // Allocate format context for an output format
      avFormatContext = new AVFormatContext();
      int returnCode = avformat.avformat_alloc_output_context2(avFormatContext, null, formatName, fileName);
      FFMPEGTools.checkError(returnCode, avFormatContext, formatName + " format request");

      // Find encoder and allocate encoder context
      AVOutputFormat outputFormat = avFormatContext.oformat();
      int codecId = outputFormat.video_codec();
      AVCodec avEncoder = avcodec.avcodec_find_encoder(codecId);
      FFMPEGTools.checkPointer(avEncoder, "Finding encoder for id: " + codecId + " name: " + formatName);
      codecLongName = avEncoder.long_name().getString().trim();
      avEncoderContext = avcodec.avcodec_alloc_context3(avEncoder);

      // Allocate a packet for later use
      tempAVPacket = avcodec.av_packet_alloc();
      FFMPEGTools.checkPointer(tempAVPacket, "Allocating a packet");

      // Create a new stream
      avStream = avformat.avformat_new_stream(avFormatContext, null);
      FFMPEGTools.checkPointer(avStream, "Adding a new stream");

      // FIXME Remove this? We were just setting the id to 0 anyway
      // Documentation says replaced by libavformat if left unset
      // int numberOfStreams = avFormatContext.nb_streams();
      // avStream.id(numberOfStreams - 1); // I don't know what this does at all, but it's in the example

      avEncoderContext.codec_id(avFormatContext.video_codec_id());
      avEncoderContext.bit_rate(bitRate); // This is what they've used in all the examples but is arbitrary other than that
      avEncoderContext.width(videoWidth);
      avEncoderContext.height(videoHeight);

      framePeriod = new AVRational();
      framePeriod.num(1);
      framePeriod.den(framerate);
      avStream.time_base(framePeriod);
      avEncoderContext.time_base(framePeriod);

      avEncoderContext.gop_size(pictureGroupSize); // Some or all of these settings may be unnecessary with lossless
      avEncoderContext.pix_fmt(avPixelFormat);

      formatWantsGlobalHeader = (outputFormat.flags() & avformat.AVFMT_GLOBALHEADER) != 0;
      if (formatWantsGlobalHeader)
      {
         avEncoderContext.flags(avEncoderContext.flags() | avcodec.AV_CODEC_FLAG_GLOBAL_HEADER);
      }
   }

   /**
    * The first time a frame is put will take longer than the others because of initialization
    */
   public boolean put(BytedecoImage image)
   {
      if (!isInitialized)
      {
         isInitialized = true;

         int returnCode = avcodec.avcodec_open2(avEncoderContext, avEncoderContext.codec(), avDictionary);
         FFMPEGTools.checkNonZeroError(returnCode, "Initializing codec context to use the codec");

         avFrame = avutil.av_frame_alloc();
         avFrame.format(avPixelFormat);
         avFrame.width(videoWidth);
         avFrame.height(videoHeight);

         tempAVFrame = avutil.av_frame_alloc();
         tempAVFrame.format(avPixelFormat);
         tempAVFrame.width(videoWidth);
         tempAVFrame.height(videoHeight);

         int bufferSizeAlignment = 0;
         returnCode = avutil.av_frame_get_buffer(avFrame, bufferSizeAlignment);
         FFMPEGTools.checkNonZeroError(returnCode, "Allocating new buffer for avFrame");

         returnCode = avutil.av_frame_get_buffer(tempAVFrame, bufferSizeAlignment);
         FFMPEGTools.checkNonZeroError(returnCode, "Allocating new buffer for tempAVFrame");

         AVCodecParameters avCodecParameters = avStream.codecpar();
         returnCode = avcodec.avcodec_parameters_from_context(avCodecParameters, avEncoderContext);
         FFMPEGTools.checkNonZeroError(returnCode, "Setting stream parameters to codec context values");

         // Dump information about the stream to a file
         int streamIndex = 0;
         int isContextOutput = 1; // Our context is output; we are streaming to file after all
         avformat.av_dump_format(avFormatContext, streamIndex, fileName, isContextOutput);

         FileTools.ensureDirectoryExists(Paths.get(fileName).getParent(), DefaultExceptionHandler.RUNTIME_EXCEPTION);

         AVIOContext avBytestreamIOContext = new AVIOContext();
         returnCode = avformat.avio_open(avBytestreamIOContext, fileName, avformat.AVIO_FLAG_WRITE);
         FFMPEGTools.checkError(returnCode, avBytestreamIOContext, "Creating and initializing the I/O context");
         avFormatContext.pb(avBytestreamIOContext);

         returnCode = avformat.avformat_write_header(avFormatContext, avDictionary);
         FFMPEGTools.checkNonZeroError(returnCode, "Allocating the stream private data and writing the stream header to the output media file");
      }

      //Encode video
      AVFrame frame = getVideoFrame(image);
      avcodec.avcodec_send_frame(avEncoderContext, frame);

      //This while loop is weird, but copied from muxing.c
      int ret = 0;
      while (ret >= 0)
      {
         ret = avcodec.avcodec_receive_packet(avEncoderContext, tempAVPacket);
         if (ret < 0)
         {
            if (ret == -11)
            {
               continue; //Resource temporarily unavailable - we just try this again
            }
            else
            {
               LogTools.error("{}: Error encoding frame. Logging will stop.", FFMPEGTools.getErrorCodeString(ret));
               destroy();
               return false;
            }
         }

         avcodec.av_packet_rescale_ts(tempAVPacket, avEncoderContext.time_base(), avStream.time_base());
         tempAVPacket.stream_index(avStream.index());

         ret = avformat.av_interleaved_write_frame(avFormatContext, tempAVPacket);
         if (ret < 0)
         {
            LogTools.error("Error writing output packet. Logging will stop.");
            destroy();
            return false;
         }
      }

      avformat.avio_flush(avFormatContext.pb());

      return ret == avutil.AVERROR_EOF();
   }

   private void fillImage(AVFrame pict, BytedecoImage image, int width, int height)
   {
      if (rgbTempFrame == null)
      {
         rgbTempFrame = avutil.av_frame_alloc();
         rgbTempFrame.format(avutil.AV_PIX_FMT_RGBA);
         rgbTempFrame.width(width);
         rgbTempFrame.height(height);

         avutil.av_frame_get_buffer(rgbTempFrame, 0);
      }

      if (avutil.av_frame_make_writable(rgbTempFrame) < 0)
      {
         LogTools.error("Could not make frame writable. Logging will stop.");
         destroy();
      }

      rgbSwsContext = swscale.sws_getContext(width, height, avutil.AV_PIX_FMT_RGBA, width, height, avutil.AV_PIX_FMT_YUV420P, swscale.SWS_BICUBIC, null, null, (DoublePointer)null);

      if (rgbSwsContext == null || rgbSwsContext.isNull())
      {
         LogTools.error("Error creating SWS Context.");
         return;
      }

      for (int y = 0; y < height; y++)
      {
         for (int x = 0; x < width; x++)
         {
            int r = image.getBackingDirectByteBuffer().get(4 * (y * width + x));
            int g = image.getBackingDirectByteBuffer().get(4 * (y * width + x) + 1);
            int b = image.getBackingDirectByteBuffer().get(4 * (y * width + x) + 2);
            int a = image.getBackingDirectByteBuffer().get(4 * (y * width + x) + 3);
            //Note: x * 4 because 4 bytes per pixel
            Pointer data = rgbTempFrame.data().get();
            data.getPointer(y * rgbTempFrame.linesize().get() + x * 4).fill(r);
            data.getPointer(y * rgbTempFrame.linesize().get() + x * 4 + 1).fill(g);
            data.getPointer(y * rgbTempFrame.linesize().get() + x * 4 + 2).fill(b);
            data.getPointer(y * rgbTempFrame.linesize().get() + x * 4 + 3).fill(a);
         }
      }

      swscale.sws_scale(rgbSwsContext, rgbTempFrame.data(), rgbTempFrame.linesize(), 0, height, pict.data(), pict.linesize());
   }

   private AVFrame getVideoFrame(BytedecoImage image)
   {
      if (avutil.av_frame_make_writable(avFrame) < 0)
      {
         LogTools.error("Could not make frame writable. Logging will stop.");
         destroy();
         return null;
      }

      if (avEncoderContext.pix_fmt() != avutil.AV_PIX_FMT_YUV420P)
      {
         if (swsContext == null || swsContext.isNull())
         {
            swsContext = swscale.sws_getContext(avEncoderContext.width(),
                                                avEncoderContext.height(),
                                                avutil.AV_PIX_FMT_YUV420P,
                                                avEncoderContext.width(),
                                                avEncoderContext.height(),
                                                avEncoderContext.pix_fmt(),
                                                swscale.SWS_BICUBIC,
                                                null,
                                                null,
                                                (DoublePointer) null);

            if (swsContext == null || swsContext.isNull())
            {
               LogTools.error("Error creating SWS Context.");
               return null;
            }
         }

         fillImage(tempAVFrame, image, avEncoderContext.width(), avEncoderContext.height());

         swscale.sws_scale(swsContext, tempAVFrame.data(), tempAVFrame.linesize(), 0, avEncoderContext.height(), avFrame.data(), avFrame.linesize());
      }
      else
      {
         fillImage(avFrame, image, avEncoderContext.width(), avEncoderContext.height());
      }

      avFrame.pts(nextPts);
      nextPts = nextPts + 1;

      return avFrame;
   }

   public void destroy()
   {
      LogTools.info("Destroying...");
      avformat.avio_close(avFormatContext.pb());
      avformat.avformat_free_context(avFormatContext);
      avcodec.avcodec_free_context(avEncoderContext);
      avutil.av_frame_free(rgbTempFrame);
      swscale.sws_freeContext(rgbSwsContext);
      avutil.av_frame_free(avFrame);
      avutil.av_frame_free(tempAVFrame);
      avutil.av_frame_free(tempAVFrame);
      avcodec.av_packet_free(tempAVPacket);
      swscale.sws_freeContext(swsContext);
   }

   public String getFormatName()
   {
      return formatName;
   }

   public String getCodecLongName()
   {
      return codecLongName;
   }

   public int getBitRate()
   {
      return bitRate;
   }

   public AVRational getFramePeriod()
   {
      return framePeriod;
   }

   public int getPictureGroupSize()
   {
      return pictureGroupSize;
   }

   public boolean getFormatWantsGlobalHeader()
   {
      return formatWantsGlobalHeader;
   }
}
