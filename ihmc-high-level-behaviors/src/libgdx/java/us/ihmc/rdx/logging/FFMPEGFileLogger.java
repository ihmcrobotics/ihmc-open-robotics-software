package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVCodecParameters;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avutil.AVRational;
import org.bytedeco.ffmpeg.global.avcodec;
import org.bytedeco.ffmpeg.global.avformat;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.ffmpeg.global.swscale;
import org.bytedeco.ffmpeg.swscale.SwsFilter;
import org.bytedeco.javacpp.DoublePointer;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.perception.BytedecoImage;

import java.nio.file.Paths;

public class FFMPEGFileLogger extends FFMPEGLogger
{
   private boolean isInitialized = false;

   protected void initialize()
   {
      isInitialized = true;

      int returnCode = avcodec.avcodec_open2(avEncoderContext, avEncoderContext.codec(), streamFlags);
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

      returnCode = avformat.avformat_write_header(avFormatContext, streamFlags);
      FFMPEGTools.checkNonZeroError(returnCode, "Allocating the stream private data and writing the stream header to the output media file");

      if (encoderFormatConversionNecessary)
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

   @Override
   public boolean put(BytedecoImage sourceImage)
   {
      if (!isInitialized)
         initialize();

      this.prepareFrameForWriting(sourceImage);

      // Presentation timestamp in time_base units (time when frame should be shown to user).
      avFrameToBeEncoded.pts(presentationTimestamp++);

      int returnCode;

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

   public FFMPEGFileLogger(int sourceVideoWidth,
                           int sourceVideoHeight,
                           boolean lossless,
                           int framerate,
                           int bitRate,
                           int sourcePixelFormat,
                           int encoderPixelFormat,
                           String fileName,
                           String preferredVideoEncoder)
   {
      super(sourceVideoWidth, sourceVideoHeight, lossless, framerate, bitRate, sourcePixelFormat, encoderPixelFormat, fileName, preferredVideoEncoder);
   }
}
