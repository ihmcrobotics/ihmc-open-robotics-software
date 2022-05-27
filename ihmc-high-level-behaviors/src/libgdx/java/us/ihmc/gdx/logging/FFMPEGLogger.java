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
import org.bytedeco.ffmpeg.swscale.SwsFilter;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.PointerPointer;
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
   private final int sourceVideoWidth;
   private final int sourceVideoHeight;
   private final String fileName;
   private final String formatName;
   private final int bitRate;
   private final AVRational framePeriod;
   private final int pictureGroupSize = 12;
   private final int sourceAVPixelFormat;
   private final int encoderAVPixelFormat;
   private final boolean formatWantsGlobalHeader;
   private boolean isInitialized = false;
   private final AVDictionary avDictionary;
   private final AVFormatContext avFormatContext;
   private final String codecLongName;
   private final AVPacket avPacket;
   private final AVStream avStream;
   private final AVCodecContext avEncoderContext;
   private AVFrame avFrameToBeEncoded;
   private AVFrame avFrameToBeScaled;
   private SwsContext swsContext;
   private int presentationTimestamp = 0;

   public FFMPEGLogger(int sourceVideoWidth,
                       int sourceVideoHeight,
                       boolean lossless,
                       int framerate,
                       int bitRate,
                       int sourcePixelFormat,
                       int encoderPixelFormat,
                       String fileName)
   {
      this.sourceVideoWidth = sourceVideoWidth;
      this.sourceVideoHeight = sourceVideoHeight;
      this.sourceAVPixelFormat = sourcePixelFormat;
      this.bitRate = bitRate;
      this.fileName = fileName;
      this.formatName = fileName.substring(fileName.lastIndexOf('.') + 1);
      this.encoderAVPixelFormat = encoderPixelFormat;

      LogTools.info("Initializing ffmpeg contexts for {} output to {}", formatName, fileName);

      int avDictFlags = 0;
      avDictionary = new AVDictionary();
      avutil.av_dict_set(avDictionary, "lossless", lossless ? "1" : "0", avDictFlags);

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

   /**
    * The first time a frame is put will take longer than the others because of initialization
    */
   public boolean put(BytedecoImage sourceImage)
   {
      if (!isInitialized)
      {
         isInitialized = true;

         int returnCode = avcodec.avcodec_open2(avEncoderContext, avEncoderContext.codec(), avDictionary);
         FFMPEGTools.checkNonZeroError(returnCode, "Initializing codec context to use the codec");

         avFrameToBeEncoded = avutil.av_frame_alloc();
         avFrameToBeEncoded.format(encoderAVPixelFormat);
         avFrameToBeEncoded.width(sourceVideoWidth);
         avFrameToBeEncoded.height(sourceVideoHeight);

         int bufferSizeAlignment = 0;
         returnCode = avutil.av_frame_get_buffer(avFrameToBeEncoded, bufferSizeAlignment);
         FFMPEGTools.checkNonZeroError(returnCode, "Allocating new buffer for avFrame");

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

         if (sourceAVPixelFormat != encoderAVPixelFormat)
         {
            avFrameToBeScaled = avutil.av_frame_alloc();
            avFrameToBeScaled.format(sourceAVPixelFormat);
            avFrameToBeScaled.width(sourceVideoWidth);
            avFrameToBeScaled.height(sourceVideoHeight);

            returnCode = avutil.av_frame_get_buffer(avFrameToBeScaled, bufferSizeAlignment);
            FFMPEGTools.checkNonZeroError(returnCode, "Allocating new buffer for tempAVFrame");

            int sourceFormat = sourceAVPixelFormat;
            int sourceVideoWidth = avEncoderContext.width();
            int sourceVideoHeight = avEncoderContext.height();
            int destinationVideoWidth = avEncoderContext.width();
            int destinationVideoHeight = avEncoderContext.height();
            int destinationFormat = encoderAVPixelFormat;
            int algorithmAndOptionForScaling = swscale.SWS_BICUBIC;
            SwsFilter sourceFilter = null;
            SwsFilter destinationFilter = null;
            DoublePointer extraParameters = null;
            swsContext = swscale.sws_getContext(sourceVideoWidth,
                                                sourceVideoHeight,
                                                sourceFormat,
                                                destinationVideoWidth,
                                                destinationVideoHeight,
                                                destinationFormat,
                                                algorithmAndOptionForScaling,
                                                sourceFilter,
                                                destinationFilter,
                                                extraParameters);
            FFMPEGTools.checkPointer(swsContext, "Allocating SWS context");
         }
      }

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

      // Presentation timestamp in time_base units (time when frame should be shown to user).
      avFrameToBeEncoded.pts(presentationTimestamp++);

      // Try again is returned quite often, citing "Resource temporarily unavailable"
      // Documentation says that the user must try to send input, so we put the send frame in here
      int tryAgainError = avutil.AVERROR_EAGAIN();
      do
      {
         returnCode = avcodec.avcodec_send_frame(avEncoderContext, avFrameToBeEncoded);
         FFMPEGTools.checkNonZeroError(returnCode, "Supplying frame to encoder");

         returnCode = avcodec.avcodec_receive_packet(avEncoderContext, avPacket);
      }
      while (returnCode == tryAgainError);
      FFMPEGTools.checkNonZeroError(returnCode, "Reading encoded data from the encoder");

      // Convert valid timing fields (timestamps / durations) in a packet from one timebase to another
      AVRational sourceTimebase = avEncoderContext.time_base();
      AVRational destinationTimebase = avStream.time_base();
      avcodec.av_packet_rescale_ts(avPacket, sourceTimebase, destinationTimebase);

      // Decompression timestamp in AVStream->time_base units; the time at which the packet is decompressed
      avPacket.stream_index(avStream.index());

      returnCode = avformat.av_interleaved_write_frame(avFormatContext, avPacket);
      FFMPEGTools.checkNonZeroError(returnCode, "Writing packet to output media file ensuring correct interleaving");

      return returnCode == 0;
   }

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

      avformat.avio_close(avFormatContext.pb());
      avformat.avformat_free_context(avFormatContext);

      avcodec.avcodec_close(avEncoderContext);
      avcodec.avcodec_free_context(avEncoderContext);

      avutil.av_frame_free(avFrameToBeEncoded);
      avutil.av_frame_free(avFrameToBeScaled);
      avutil.av_frame_free(avFrameToBeScaled);
      avcodec.av_packet_free(avPacket);

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
