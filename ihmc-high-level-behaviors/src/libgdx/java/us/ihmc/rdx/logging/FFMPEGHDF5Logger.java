package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVCodecParameters;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avutil.AVRational;
import org.bytedeco.ffmpeg.global.avcodec;
import org.bytedeco.ffmpeg.global.avformat;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.ffmpeg.global.swscale;
import org.bytedeco.ffmpeg.swscale.SwsFilter;
import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.PointerPointer;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.logging.HDF5Manager;
import us.ihmc.perception.logging.HDF5Tools;

import java.nio.file.Paths;

public class FFMPEGHDF5Logger extends FFMPEGLogger
{
   public static final String NAMESPACE = "FFMPEGVideo"; //this will need to be changed later to be user-set
   private HDF5Manager hdf5Manager;
   private Group framesGroup;

   @Override
   public void stop()
   {
      framesGroup.close();
      super.stop();
   }

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

      FileTools.ensureDirectoryExists(Paths.get(fileName).getParent(), DefaultExceptionHandler.RUNTIME_EXCEPTION);

      AVIOContext avBytestreamIOContext = new AVIOContext(); //do this instead to handle custom wrtie? https://stackoverflow.com/questions/42400811/looking-for-javacpp-ffmpeg-customio-example
      returnCode = avformat.avio_open(avBytestreamIOContext, fileName, avformat.AVIO_FLAG_WRITE);
      FFMPEGTools.checkError(returnCode, avBytestreamIOContext, "Creating and initializing the I/O context");
      avFormatContext.pb(avBytestreamIOContext);

      returnCode = avformat.avformat_write_header(avFormatContext, streamFlags); //The current setup works, but creates a (mostly) empty file with the header information. Hopefully setrting a custom AVIOContext can fix this problem, but failing that
      //creating a temp file and reading it back should work. Not ideal, but considering we can write frames directly it won't cause a performance hit. The same thing will probably have to happen when final data is written to the file at close.
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

      framesGroup = hdf5Manager.getGroup(NAMESPACE);
   }

   @Override
   public boolean put(BytedecoImage sourceImage)
   {
      if (!isInitialized)
         initialize();

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
      avFrameToBeEncoded.pts(presentationTimestamp);

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

      HDF5Tools.storeBytesFromPointer(framesGroup, presentationTimestamp++, avPacket.data(), avPacket.size());

      returnCode = avformat.av_interleaved_write_frame(avFormatContext, avPacket);
      FFMPEGTools.checkNonZeroError(returnCode, "Writing packet to output media file ensuring correct interleaving");

      return returnCode == 0;
   }

   public FFMPEGHDF5Logger(int sourceVideoWidth,
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

      fileName += ".hdf5"; //this is going to look like something.mp4.hdf5 for now and should be improved later
      hdf5Manager = new HDF5Manager(fileName, hdf5.H5F_ACC_TRUNC);
      hdf5Manager.getFile().openFile(fileName, hdf5.H5F_ACC_RDWR);
   }
}
