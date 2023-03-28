package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avcodec.AVCodecContext;
import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVFrame;
import org.bytedeco.ffmpeg.avutil.AVRational;
import org.bytedeco.ffmpeg.global.avcodec;
import org.bytedeco.ffmpeg.global.avformat;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.ffmpeg.global.swscale;
import org.bytedeco.ffmpeg.swscale.SwsContext;
import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.PointerPointer;
import us.ihmc.log.LogTools;
import us.ihmc.perception.logging.HDF5Manager;
import us.ihmc.perception.logging.HDF5Tools;
import us.ihmc.perception.logging.PerceptionLoggerConstants;

import java.nio.ByteBuffer;

/**
 * FIXME: Not working yet. Needs some love.
 */
public class FFMPEGHDF5FileReader implements IFFMPEGFileReader
{
   private final AVFormatContext avFormatContext;
   private int streamIndex;
   private AVCodecContext decoderContext;
   private final AVFrame videoFrame;
   private final AVFrame rgbFrame;
   private final AVPacket packet;
   private SwsContext swsContext;
   private boolean isClosed;
   private final AVRational timeBase;
   private final AVRational framerate;
   private final int width;
   private final int height;
   private final long duration;
   private final long startTime;

   private HDF5Manager hdf5Manager;
   private final HDF5Tools hdf5Tools = new HDF5Tools();
   private Group framesGroup;

   BytePointer packetData = new BytePointer(0);

   /**
    * Due to the current intermediate design of the FFMPEGHDF5Logger, there are two files which
    * the log needs to read from, one of which contains all the metadata required for ffmpeg.
    */
   public FFMPEGHDF5FileReader(String hdf5File)
   {
      avutil.av_log_set_level(avutil.AV_LOG_WARNING);

      String file = hdf5File.replaceFirst(PerceptionLoggerConstants.HDF5_FILE_EXTENSION, ""); // BAD

      LogTools.info("Initializing ffmpeg contexts for playback from {}", file);
      avFormatContext = avformat.avformat_alloc_context();
      FFMPEGTools.checkNonZeroError(avformat.avformat_open_input(avFormatContext, file, null, null), "Initializing format context");

      FFMPEGTools.checkNonZeroError(avformat.avformat_find_stream_info(avFormatContext, (AVDictionary) null), "Finding stream information");

      openCodecContext();

      AVStream stream = avFormatContext.streams(streamIndex);

      LogTools.debug("FILE PROPERTIES: Width: {}\tHeight: {}\tFormat:{}",
                     decoderContext.width(),
                     decoderContext.height(),
                     avutil.av_get_pix_fmt_name(decoderContext.pix_fmt()).getString());

      width = decoderContext.width();
      height = decoderContext.height();

      // Extremely strange but working method for determining duration. Webms have weird durations sometimes so we get it from avFormatContext
      // It just so happens that avFormatContext.duration() is exactly 1000 times too large, so we make it smaller.
      if ((stream.duration() == 0 || stream.duration() == 0x8000000000000000L))
         duration = avFormatContext.duration() / 1000;
      else
         duration = stream.duration();

      startTime = stream.start_time();
      timeBase = stream.time_base();
      framerate = stream.avg_frame_rate(); //TODO Simple workaround fix for framerate. Does not work with variable framerate streams, but should be fine for IHMC webms

      avformat.av_dump_format(avFormatContext, 0, file, 0);

      videoFrame = avutil.av_frame_alloc();
      packet = avcodec.av_packet_alloc();

      // Because videoFrame's buffers move around in memory, we build rgbFrame regardless of if the format is already RGBA
      rgbFrame = avutil.av_frame_alloc();
      rgbFrame.width(width);
      rgbFrame.height(height);
      rgbFrame.format(avutil.AV_PIX_FMT_RGBA);

      FFMPEGTools.checkNonZeroError(avutil.av_frame_get_buffer(rgbFrame, 0), "Allocating buffer for rgbFrame");

      if (decoderContext.pix_fmt() != avutil.AV_PIX_FMT_RGBA)
      {
         swsContext = swscale.sws_getContext(width,
                                             height,
                                             decoderContext.pix_fmt(),
                                             width,
                                             height,
                                             avutil.AV_PIX_FMT_RGBA,
                                             swscale.SWS_BICUBIC,
                                             null,
                                             null,
                                             (DoublePointer) null);
         FFMPEGTools.checkPointer(swsContext, "Allocating SWS context");
      }

      hdf5Manager = new HDF5Manager(hdf5File, hdf5.H5F_ACC_RDONLY);
      hdf5Manager.getFile().openFile(hdf5File, hdf5.H5F_ACC_RDONLY);

      framesGroup = hdf5Manager.createOrGetGroup(FFMPEGHDF5Logger.NAMESPACE_ROOT);
   }

   // Adapted from demuxing_decoding.c. Currently assumes video stream, but could be adapted for audio use, too
   private void openCodecContext()
   {
      streamIndex = avformat.av_find_best_stream(avFormatContext, avutil.AVMEDIA_TYPE_VIDEO, -1, -1, (AVCodec) null, 0);
      FFMPEGTools.checkNegativeError(streamIndex, "Finding video stream");

      AVStream stream = avFormatContext.streams(streamIndex);
      AVCodec decoder = avcodec.avcodec_find_decoder(stream.codecpar().codec_id());
      FFMPEGTools.checkPointer(decoder, "Finding codec");

      decoderContext = avcodec.avcodec_alloc_context3(decoder);
      FFMPEGTools.checkPointer(decoderContext, "Allocating decoder context");

      FFMPEGTools.checkNonZeroError(avcodec.avcodec_parameters_to_context(decoderContext, stream.codecpar()), "Copying codec parameters to decoder context");
      FFMPEGTools.checkNonZeroError(avcodec.avcodec_open2(decoderContext, decoder, (AVDictionary) null), "Opening codec");
   }

   @Override
   public long seek(long timestamp)
   {
      return 0; // Not yet implemented
   }

   private boolean loadNextFrame()
   {
      int returnCode;
      do
      {

         avformat.av_read_frame(avFormatContext, packet);
         // get packet from HDF5

         hdf5Tools.loadBytes(framesGroup, streamIndex, packetData);
         BytePointer.memcpy(packet.data(), packetData, packetData.capacity());
      }
      while (packet.stream_index() != streamIndex);

      FFMPEGTools.checkNonZeroError(avcodec.avcodec_send_packet(decoderContext, packet), "Sending packet for decoding");

      // Note: Video packets always contain exactly one frame. For audio, etc. care must be taken to ensure all frames are read
      do
      {
         returnCode = avcodec.avcodec_receive_frame(decoderContext, videoFrame);
      }
      while (returnCode == avutil.AVERROR_EAGAIN() || returnCode == avutil.AVERROR_EOF());
      FFMPEGTools.checkNegativeError(returnCode, "Decoding frame from packet");

      avcodec.av_packet_unref(packet); // This is NOT freeing the packet, which is done later

      return true;
   }

   /**
    * Gets next frame, and stores in native memory (access with {@link #getFrameDataBuffer()})
    * @param load Whether or not to laod the next frame, or to use the existing loaded one
    * @return -1 when end of file reached (AVERROR_EOF), timestamp in time base units otherwise
    */
   @Override
   public long getNextFrame(boolean load)
   {
      if (load && !loadNextFrame()) // Do not call loadNextFrame if load is false
         return -1; // EOF

      FFMPEGTools.checkNonZeroError(avutil.av_frame_make_writable(rgbFrame), "Ensuring frame data is writable");

      if (swsContext == null)
      {
         avutil.av_frame_copy(rgbFrame, videoFrame);
      }
      else
      {
         PointerPointer sourceSlice = videoFrame.data();
         IntPointer sourceStride = videoFrame.linesize();
         int sourceSliceY = 0;
         int sourceSliceHeight = height;
         PointerPointer destination = rgbFrame.data();
         IntPointer destinationStride = rgbFrame.linesize();
         swscale.sws_scale(swsContext, sourceSlice, sourceStride, sourceSliceY, sourceSliceHeight, destination, destinationStride);
      }

      long approxTimestamp = videoFrame.best_effort_timestamp();

      avutil.av_frame_unref(videoFrame);

      return approxTimestamp;
   }

   @Override
   public void close()
   {
      if (isClosed)
         return;

      if (swsContext != null)
         swscale.sws_freeContext(swsContext);

      avcodec.avcodec_free_context(decoderContext);
      avformat.avformat_close_input(avFormatContext);

      avcodec.av_packet_free(packet);
      avutil.av_frame_free(videoFrame);

      if (rgbFrame != null)
         avutil.av_frame_free(rgbFrame);

      isClosed = true;
   }

   @Override
   public ByteBuffer getFrameDataBuffer()
   {
      return rgbFrame.data(0).asByteBuffer();
   }

   @Override
   public AVRational getTimeBase()
   {
      return timeBase;
   }

   @Override
   public AVRational getAverageFramerate()
   {
      return framerate;
   }

   @Override
   public int getWidth()
   {
      return width;
   }

   @Override
   public int getHeight()
   {
      return height;
   }

   @Override
   public long getDuration()
   {
      return duration;
   }

   @Override
   public long getStartTime()
   {
      return startTime;
   }
}
