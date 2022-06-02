package us.ihmc.gdx.logging;

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
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.PointerPointer;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;

public class FFMPEGFileReader
{
   private AVFormatContext avFormatContext;
   private int streamIndex;
   private AVCodecContext decoderContext;
   private AVFrame videoFrame;
   private AVFrame rgbFrame;
   private AVPacket packet;
   private SwsContext swsContext;
   private boolean isClosed = false;
   private final AVRational timeBase;
   private final AVRational framerate;
   private final int width;
   private final int height;
   private final long duration;
   private final long startTime;

   public FFMPEGFileReader(String file) {
      LogTools.info("Initializing ffmpeg contexts for playback from {}", file);
      avFormatContext = avformat.avformat_alloc_context();
      FFMPEGTools.checkNonZeroError(avformat.avformat_open_input(avFormatContext, file, null, null), "Initializing format context");

      FFMPEGTools.checkNonZeroError(avformat.avformat_find_stream_info(avFormatContext, (AVDictionary) null), "Finding stream information");

      openCodecContext();

      AVStream stream = avFormatContext.streams(streamIndex);

      LogTools.debug("FILE PROPERTIES: Width: {}\tHeight: {}\tFormat:{}",
                     decoderContext.width(), decoderContext.height(), avutil.av_get_pix_fmt_name(decoderContext.pix_fmt()).getString());

      width = decoderContext.width();
      height = decoderContext.height();
      duration = avFormatContext.duration();
      startTime = avFormatContext.start_time();

      timeBase = stream.time_base();
      framerate = stream.avg_frame_rate(); //TODO Simple workaround fix for framerate. Does not work with variable framerate streams, but should be fine for IHMC webms

      avformat.av_dump_format(avFormatContext, 0, file, 0);

      videoFrame = avutil.av_frame_alloc();
      packet = avcodec.av_packet_alloc();

      if (decoderContext.pix_fmt() != avutil.AV_PIX_FMT_RGBA)
      {
         rgbFrame = avutil.av_frame_alloc();
         rgbFrame.width(width);
         rgbFrame.height(height);
         rgbFrame.format(avutil.AV_PIX_FMT_RGBA);

         FFMPEGTools.checkNonZeroError(avutil.av_frame_get_buffer(rgbFrame, 0), "Allocating buffer for rgbFrame");

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
   }

   //Adapted from demuxing_decoding.c. Currently assumes video stream, but could be adapted for audio use, too
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

   public void seek(long timestamp)
   {
      FFMPEGTools.checkNegativeError(avformat.av_seek_frame(avFormatContext, streamIndex, timestamp, 0), "Seeking frame via timestamp", false);
   }

   /***
    * @param image BytedecoImage of proper resolution/format to store data from frame into
    * @return -1 when end of file reached (AVERROR_EOF), timestamp in time base units otherwise
    */
   public long getNextFrame(BytedecoImage image) {
      int returnCode;
      do
      {
         returnCode = avformat.av_read_frame(avFormatContext, packet);
         if (returnCode == avutil.AVERROR_EOF())
            return -1;
         FFMPEGTools.checkNegativeError(returnCode, "Getting next frame from stream");
      }
      while (packet.stream_index() != streamIndex);

      FFMPEGTools.checkNonZeroError(avcodec.avcodec_send_packet(decoderContext, packet), "Sending packet for decoding");

      //Note: video packets always contain exactly one frame. For audio, etc. care must be taken to ensure all frames are read
      do
      {
         returnCode = avcodec.avcodec_receive_frame(decoderContext, videoFrame);
      }
      while (returnCode == avutil.AVERROR_EAGAIN() || returnCode == avutil.AVERROR_EOF());
      FFMPEGTools.checkNegativeError(returnCode, "Decoding frame from packet");

      if (swsContext != null)
      {
         FFMPEGTools.checkNonZeroError(avutil.av_frame_make_writable(rgbFrame), "Ensuring frame data is writable");

         PointerPointer sourceSlice = videoFrame.data();
         IntPointer sourceStride = videoFrame.linesize();
         int sourceSliceY = 0;
         int sourceSliceHeight = height;
         PointerPointer destination = rgbFrame.data();
         IntPointer destinationStride = rgbFrame.linesize();
         swscale.sws_scale(swsContext, sourceSlice, sourceStride, sourceSliceY, sourceSliceHeight, destination, destinationStride);

         for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
               int data = rgbFrame.data(0).getInt(4 * (i * width + j));
               image.getBackingDirectByteBuffer().putInt(4 * (i * width + j), data);
            }
         }
         //image.getBackingDirectByteBuffer().put(rgbFrame.data(0).asByteBuffer());
      }
      else
      {
         image.getBackingDirectByteBuffer().put(videoFrame.data(0).asByteBuffer());
      }

      long approxTimestamp = videoFrame.best_effort_timestamp();

      avutil.av_frame_unref(videoFrame);

      avcodec.av_packet_unref(packet); //This is NOT freeing the packet, which is done later

      return approxTimestamp;
   }

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

   public AVRational getTimeBase()
   {
      return timeBase;
   }

   public AVRational getFramerate()
   {
      return framerate;
   }

   public int getWidth()
   {
      return width;
   }

   public int getHeight()
   {
      return height;
   }

   public long getDuration()
   {
      return duration;
   }

   public long getStartTime()
   {
      return startTime;
   }
}
