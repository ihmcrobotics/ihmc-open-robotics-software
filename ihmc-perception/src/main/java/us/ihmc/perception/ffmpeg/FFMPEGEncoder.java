package us.ihmc.perception.ffmpeg;

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
import us.ihmc.log.LogTools;

import java.util.function.Consumer;

import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avformat.av_interleaved_write_frame;
import static org.bytedeco.ffmpeg.global.avformat.avformat_new_stream;
import static org.bytedeco.ffmpeg.global.avutil.*;
import static org.bytedeco.ffmpeg.presets.avutil.AVERROR_EAGAIN;

// TODO: Add a FFMPEGAudioEncoder
public abstract class FFMPEGEncoder
{
   protected final AVFormatContext outputContext;

   protected final AVStream stream;
   protected final AVCodec encoder;
   protected final AVCodecContext encoderContext;

   protected long nextPresentationTimestamp = 0L;
   protected final AVFrame nextFrame;
   protected final AVPacket encodedPacket;

   protected AVRational timeBase;
   protected final int bitRate;

   protected int error;

   public FFMPEGEncoder(AVFormatContext outputContext, String preferredEncoderName, int bitRate)
   {
      this.outputContext = outputContext;
      this.bitRate = bitRate;

      encoder = findEncoder(preferredEncoderName, outputContext.oformat());
      FFMPEGTools.checkPointer(encoder, "Finding encoder");

      encodedPacket = av_packet_alloc();
      FFMPEGTools.checkPointer(encodedPacket, "Allocating next packet");

      stream = avformat_new_stream(outputContext, encoder);
      FFMPEGTools.checkPointer(stream, "Creating new stream");

      encoderContext = avcodec_alloc_context3(encoder);
      FFMPEGTools.checkPointer(encoderContext, "Allocating codec context");

      nextFrame = av_frame_alloc();
      FFMPEGTools.checkPointer(nextFrame, "Allocating input frame");

      if ((outputContext.oformat().flags() & avformat.AVFMT_GLOBALHEADER) != 0)
         encoderContext.flags(encoderContext.flags() | avcodec.AV_CODEC_FLAG_GLOBAL_HEADER);
   }

   public void initialize()
   {
      initialize(null);
   }

   public abstract void initialize(AVDictionary codecOptions);

   protected void assignNextFrame(Consumer<AVFrame> nextFrameAssignment)
   {
      nextFrameAssignment.accept(nextFrame);
      nextFrame.pts(nextPresentationTimestamp++);
   }

   public boolean encodeAndWriteNextFrame()
   {
      return encodeNextFrame(packet ->
      {
         error = av_interleaved_write_frame(outputContext, packet);
         FFMPEGTools.checkNegativeError(error, "Writing packet");
      });
   }

   public boolean encodeNextFrame(Consumer<AVPacket> packetConsumer)
   {
      int error;
      error = avcodec_send_frame(encoderContext, nextFrame);
      FFMPEGTools.checkNegativeError(error, "Sending frame");

      while (error >= 0)
      {
         error = avcodec_receive_packet(encoderContext, encodedPacket);
         if (error == AVERROR_EAGAIN() || error == AVERROR_EOF())
            break;
         else
            FFMPEGTools.checkNegativeError(error, "Receiving packet");

         av_packet_rescale_ts(encodedPacket, encoderContext.time_base(), stream.time_base());
         encodedPacket.stream_index(stream.index());

         if (packetConsumer != null)
            packetConsumer.accept(encodedPacket);

         av_packet_unref(encodedPacket);
      }

      return error == AVERROR_EAGAIN();
   }

   public void destroy()
   {
      nextFrame.close();
      encodeNextFrame(null);

      avcodec_close(encoderContext);
      avcodec_free_context(encoderContext);
      av_frame_free(nextFrame);
      av_packet_free(encodedPacket);

      encoder.close();
      encoderContext.close();
      stream.close();
      timeBase.close();
   }

   private AVCodec findEncoder(String preferredEncoderName, AVOutputFormat outputFormat)
   {
      AVCodec encoder;

      if (preferredEncoderName != null)
      {
         encoder = avcodec_find_encoder_by_name(preferredEncoderName);
         if (encoder != null && !encoder.isNull())
            return encoder;

         LogTools.error("Failed to find valid encoder " + preferredEncoderName + " - attempting to default to another");
      }

      encoder = avcodec_find_encoder(outputFormat.video_codec());
      return encoder;
   }
}
