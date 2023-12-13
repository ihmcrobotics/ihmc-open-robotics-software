package us.ihmc.rdx.logging;

import com.google.common.primitives.Longs;
import org.apache.commons.io.FileUtils;
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
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.DoublePointer;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.logging.HDF5Manager;
import us.ihmc.perception.logging.HDF5Tools;

import java.io.File;
import java.io.IOException;
import java.nio.file.Paths;

/**
 * FIXME: Not working yet. Needs some love.
 */
public class FFMPEGHDF5Logger extends FFMPEGLogger
{
   public static final String NAMESPACE_ROOT = "ffmpeg"; //this will need to be changed later to be user-set
   private final HDF5Manager hdf5Manager;
   private Group ptsGroup;
   private Group dtsGroup;
   private Group dataGroup;
   private Group flagsGroup;
   private Group sideDataGroup;
   private Group durationGroup;
   private Group posGroup;
   private final String tempFileName;
   private final HDF5Tools hdf5Tools = new HDF5Tools();

   @Override
   public void stop()
   {
      super.stop();

      try {
         Group headerGroup = hdf5Manager.createOrGetGroup(NAMESPACE_ROOT + "/header");

         byte[] headerData = FileUtils.readFileToByteArray(new File(tempFileName));
         hdf5Tools.storeByteArray(headerGroup, 0, headerData, headerData.length);

         headerGroup.close();
      }
      catch (IOException ex) {
         LogTools.warn("Encountered problem recording header data - video may be corrupt.");
         LogTools.warn(ex);
      }

      ptsGroup.close();
      dtsGroup.close();
      dataGroup.close();
      flagsGroup.close();
      sideDataGroup.close();
      durationGroup.close();
      posGroup.close();

      hdf5Manager.getFile().close();
   }

   private boolean isInitialized = false;
   protected void initialize()
   {
      isInitialized = true;

      ptsGroup = hdf5Manager.createOrGetGroup(NAMESPACE_ROOT + "/pts");
      dtsGroup = hdf5Manager.createOrGetGroup(NAMESPACE_ROOT + "/dts");
      dataGroup = hdf5Manager.createOrGetGroup(NAMESPACE_ROOT + "/data");
      flagsGroup = hdf5Manager.createOrGetGroup(NAMESPACE_ROOT + "/flags");
      sideDataGroup = hdf5Manager.createOrGetGroup(NAMESPACE_ROOT + "/side_data");
      durationGroup = hdf5Manager.createOrGetGroup(NAMESPACE_ROOT + "/duration");
      posGroup = hdf5Manager.createOrGetGroup(NAMESPACE_ROOT + "/pos");

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

      AVIOContext avBytestreamIOContext = new AVIOContext(); //Possible more robust solution: https://stackoverflow.com/questions/42400811/looking-for-javacpp-ffmpeg-customio-example (use for playback probably)
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
      avFrameToBeEncoded.pts(presentationTimestamp);

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

      // Write packet data to HDF groups instead of calling av_interleave_frame()
      hdf5Tools.storeByteArray(ptsGroup, presentationTimestamp++, Longs.toByteArray(avPacket.pts()), Long.BYTES);
      hdf5Tools.storeByteArray(dtsGroup, presentationTimestamp++, Longs.toByteArray(avPacket.dts()), Long.BYTES);

      // TODO this copy operation is necessary for data to be properly logged but there might be a faster way to do this.
      //  There doesn't seem to be a major speed impact though
      try (BytePointer data = new BytePointer(avPacket.size())) {
         BytePointer.memcpy(data, avPacket.data(), avPacket.size());
         hdf5Tools.storeBytes(dataGroup, presentationTimestamp++, data);
      }

      hdf5Tools.storeByteArray(flagsGroup, presentationTimestamp++, Longs.toByteArray(avPacket.flags()), Integer.BYTES); // Not a long but that's okay

      if (avPacket.side_data() != null) {
         try (BytePointer data = new BytePointer(avPacket.side_data().size())) {
            BytePointer.memcpy(data, avPacket.side_data().data(), avPacket.side_data().size());
            hdf5Tools.storeBytes(dataGroup, presentationTimestamp++, data);
         }
      }

      hdf5Tools.storeByteArray(durationGroup, presentationTimestamp++, Longs.toByteArray(avPacket.duration()), Long.BYTES);
      hdf5Tools.storeByteArray(posGroup, presentationTimestamp++, Longs.toByteArray(avPacket.pos()), Long.BYTES);

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
      // We write some information (such as the header) to a temp file because of the way FFMPEG works - we don't have direct control over this write
      super(sourceVideoWidth, sourceVideoHeight, lossless, framerate, bitRate, sourcePixelFormat, encoderPixelFormat,
            System.getProperty("java.io.tmpdir") + System.getProperty("file.separator") + fileName.replaceAll("\\W+", "") + ".h264", preferredVideoEncoder);

      this.tempFileName = System.getProperty("java.io.tmpdir") + System.getProperty("file.separator") + fileName.replaceAll("\\W+", "") + ".h264";

      hdf5Manager = new HDF5Manager(fileName, hdf5.H5F_ACC_TRUNC);
      hdf5Manager.getFile().openFile(fileName, hdf5.H5F_ACC_RDWR);
   }
}
