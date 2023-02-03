package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avcodec.AVCodecContext;
import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
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
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.PointerPointer;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;

/**
 * Doxygen:
 * https://ffmpeg.org/doxygen/trunk/index.html
 * Wiki:
 * https://trac.ffmpeg.org/
 */
public abstract class FFMPEGLogger
{
   protected final int sourceVideoWidth;
   protected final int sourceVideoHeight;
   protected final String fileName;
   protected final boolean encoderFormatConversionNecessary;
   private final String formatName;
   private final int bitRate;
   private final AVRational framePeriod;
   private final int pictureGroupSize = 12;
   protected final int sourceAVPixelFormat;
   protected final int encoderAVPixelFormat;
   private final boolean formatWantsGlobalHeader;
   protected final AVDictionary streamFlags;
   private final AVDictionary metadata;
   protected final AVFormatContext avFormatContext;
   protected String codecLongName;
   protected final AVPacket avPacket;
   protected AVFrame avFrameToBeEncoded;
   protected AVFrame avFrameToBeScaled;
   protected final AVStream avStream;
   protected AVCodecContext avEncoderContext;
   protected SwsContext swsContext;
   protected int presentationTimestamp = 0;

   public FFMPEGLogger(int sourceVideoWidth,
                       int sourceVideoHeight,
                       boolean lossless,
                       int framerate,
                       int bitRate,
                       int sourcePixelFormat,
                       int encoderPixelFormat,
                       String fileName,
                       String preferredVideoEncoder)
   {
      this.sourceVideoWidth = sourceVideoWidth;
      this.sourceVideoHeight = sourceVideoHeight;
      this.sourceAVPixelFormat = sourcePixelFormat;
      this.bitRate = bitRate;
      this.fileName = fileName;
      this.formatName = fileName.substring(fileName.lastIndexOf('.') + 1);
      this.encoderAVPixelFormat = encoderPixelFormat;

      // No conversion for RGBA -> RGB0
      boolean isJustAlphaToNonAlpha = sourceAVPixelFormat == avutil.AV_PIX_FMT_RGBA && encoderPixelFormat == avutil.AV_PIX_FMT_RGB0;
      encoderFormatConversionNecessary = sourceAVPixelFormat != encoderAVPixelFormat && !isJustAlphaToNonAlpha;

      avutil.av_log_set_level(avutil.AV_LOG_WARNING);

      LogTools.info("Initializing ffmpeg contexts for {} output to {}", formatName, fileName);

      int avDictFlags = 0;
      streamFlags = new AVDictionary();
      avutil.av_dict_set(streamFlags, "lossless", lossless ? "1" : "0", avDictFlags);

      // Allocate format context for an output format
      avFormatContext = new AVFormatContext();
      int returnCode = avformat.avformat_alloc_output_context2(avFormatContext, null, formatName, fileName);
      FFMPEGTools.checkError(returnCode, avFormatContext, formatName + " format request");

      // Allow for saving metadata
      metadata = new AVDictionary();
      avutil.av_dict_set(metadata, "Source Pixel Format", avutil.av_get_pix_fmt_name(sourcePixelFormat).getString(), 0);
      avFormatContext.metadata(metadata);

      // Find encoder and allocate encoder context
      AVOutputFormat outputFormat = avFormatContext.oformat();
      setupEncoder(outputFormat, preferredVideoEncoder);

      // Allocate a packet for later use
      avPacket = avcodec.av_packet_alloc();
      FFMPEGTools.checkPointer(avPacket, "Allocating a packet");

      // Create a new stream
      avStream = avformat.avformat_new_stream(avFormatContext, null);
      FFMPEGTools.checkPointer(avStream, "Adding a new stream");

      avEncoderContext.codec_id(avFormatContext.video_codec_id());
      avEncoderContext.bit_rate(bitRate); // This is what they've used in all the examples but is arbitrary other than that
      avEncoderContext.width(sourceVideoWidth);
      avEncoderContext.height(sourceVideoHeight);

      //Enable FFMPEG-decided multithreading (in testing, doubles performance)
      avEncoderContext.thread_type(AVCodecContext.FF_THREAD_SLICE);
      avEncoderContext.thread_count(0);

      framePeriod = new AVRational();
      framePeriod.num(1);
      framePeriod.den(framerate);
      avStream.time_base(framePeriod);
      avEncoderContext.time_base(framePeriod);

      avEncoderContext.gop_size(pictureGroupSize); // Some or all of these settings may be unnecessary with lossless
      avEncoderContext.pix_fmt(encoderAVPixelFormat);

      formatWantsGlobalHeader = (outputFormat.flags() & avformat.AVFMT_GLOBALHEADER) != 0;
      if (formatWantsGlobalHeader)
      {
         avEncoderContext.flags(avEncoderContext.flags() | avcodec.AV_CODEC_FLAG_GLOBAL_HEADER);
      }
   }

   protected void setupEncoder(AVOutputFormat outputFormat, String preferredVideoEncoder)
   {
      AVCodec avEncoder = null;
      if (preferredVideoEncoder != null)
      {
         avEncoder = avcodec.avcodec_find_encoder_by_name(preferredVideoEncoder);

         if (avEncoder != null && !avEncoder.isNull())
         {
            if (outputFormat.video_codec() != avEncoder.id())
               outputFormat.video_codec(avEncoder.id());
            LogTools.info("Found encoder " + preferredVideoEncoder + " - id:" + avEncoder.id());
         }
         else
            LogTools.error("Failed to find valid encoder " + preferredVideoEncoder + " - attempting to default to another");
      }

      if (preferredVideoEncoder == null || avEncoder == null)
      {
         int codecId = outputFormat.video_codec();
         avEncoder = avcodec.avcodec_find_encoder(codecId);
         FFMPEGTools.checkPointer(avEncoder, "Finding encoder for id: " + codecId + " name: " + formatName);
         LogTools.info("Found encoder " + avEncoder.name().getString() + " - id:" + avEncoder.id());
      }

      codecLongName = avEncoder.long_name().getString().trim();
      avEncoderContext = avcodec.avcodec_alloc_context3(avEncoder);
   }

   /**
    * The first time a frame is put will take longer than the others because of initialization
    */
   public abstract boolean put(BytedecoImage sourceImage);

   public void stop()
   {
      LogTools.info("Stopping...");

      int endOfFileError = avutil.AVERROR_EOF();
      int tryAgainError = avutil.AVERROR_EAGAIN();
      int returnCode;
      do
      {
         returnCode = avcodec.avcodec_send_frame(avEncoderContext, null);
         if (returnCode != 0 && returnCode != endOfFileError) // end of file is okay
         {
            LogTools.warn("Got code: {}: {}", returnCode, FFMPEGTools.getErrorCodeString(returnCode));
            FFMPEGTools.checkNonZeroError(returnCode, "Supplying null frame to encoder to signal end of stream");
         }

         // Reading encoded data packet from the encoder
         returnCode = avcodec.avcodec_receive_packet(avEncoderContext, avPacket);
         if (returnCode != 0 && returnCode != tryAgainError && returnCode != endOfFileError) // try again, end of file, are okay
         {
            FFMPEGTools.checkNonZeroError(returnCode, "Reading encoded data packet from the encoder");
         }
      }
      while (returnCode != endOfFileError);

      returnCode = avformat.av_write_trailer(avFormatContext);
      FFMPEGTools.checkNonZeroError(returnCode, "Writing stream trailer to output media file");

      avformat.avformat_flush(avFormatContext);

      avformat.avio_close(avFormatContext.pb());
      avformat.avformat_free_context(avFormatContext);

      avcodec.avcodec_close(avEncoderContext);
      avcodec.avcodec_free_context(avEncoderContext);

      avutil.av_frame_free(avFrameToBeEncoded);
      avutil.av_frame_free(avFrameToBeScaled);
      avcodec.av_packet_free(avPacket);

      swscale.sws_freeContext(swsContext);
   }

   protected void prepareFrameForWriting(BytedecoImage sourceImage)
   {
      int returnCode;
      returnCode = avutil.av_frame_make_writable(avFrameToBeEncoded);
      FFMPEGTools.checkNonZeroError(returnCode, "Ensuring frame data is writable");

      if (swsContext == null)
      {
         avFrameToBeEncoded.data(0, sourceImage.getBytedecoByteBufferPointer());
      }
      else
      {
         returnCode = avutil.av_frame_make_writable(avFrameToBeScaled);
         FFMPEGTools.checkNonZeroError(returnCode, "Ensuring frame data is writable");

         avFrameToBeScaled.data(0, sourceImage.getBytedecoByteBufferPointer());

         PointerPointer sourceSlice = avFrameToBeScaled.data();
         IntPointer sourceStride = avFrameToBeScaled.linesize();
         int sourceSliceY = 0;
         int sourceSliceHeight = avEncoderContext.height();
         PointerPointer destination = avFrameToBeEncoded.data();
         IntPointer destinationStride = avFrameToBeEncoded.linesize();
         int heightOfOutputSlice = swscale.sws_scale(swsContext, sourceSlice, sourceStride, sourceSliceY, sourceSliceHeight, destination, destinationStride);
      }
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
