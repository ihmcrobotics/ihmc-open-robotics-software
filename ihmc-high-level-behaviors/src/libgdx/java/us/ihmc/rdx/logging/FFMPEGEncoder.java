package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVCodecContext;
import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVFrame;

import java.util.function.Consumer;

import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avutil.AVERROR_EOF;
import static org.bytedeco.ffmpeg.presets.avutil.AVERROR_EAGAIN;

public class FFMPEGEncoder
{
   public static boolean encodeVideoFrame(FFMPEGVideoOutputStream outputStream, Consumer<AVPacket> packetConsumer)
   {
      return encodeFrame(outputStream.getCodecContext(), outputStream.getStream(), outputStream.getNextFrame(), outputStream.getNextPacket(), packetConsumer);
   }

   public static boolean encodeFrame(AVCodecContext codecContext, AVStream stream, AVFrame frame, AVPacket packet, Consumer<AVPacket> packetConsumer)
   {
      int error;
      error = avcodec_send_frame(codecContext, frame);
      FFMPEGTools.checkNegativeError(error, "Sending frame");

      while (error >= 0)
      {
         error = avcodec_receive_packet(codecContext, packet);
         if (error == AVERROR_EAGAIN() || error == AVERROR_EOF())
            break;
         else
            FFMPEGTools.checkNegativeError(error, "Receiving packet");

         av_packet_rescale_ts(packet, codecContext.time_base(), stream.time_base());
         packet.stream_index(stream.index());

         packetConsumer.accept(packet);

         av_packet_unref(packet);
      }

      return error == AVERROR_EAGAIN();
   }

}
