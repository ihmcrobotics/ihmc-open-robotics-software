package us.ihmc.perception.ffmpeg;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avcodec.AVCodecContext;
import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVFrame;

import java.util.function.Consumer;
import java.util.function.Function;

import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avformat.av_find_best_stream;
import static org.bytedeco.ffmpeg.global.avutil.*;

public class FFmpegDecoder
{
   protected final AVFormatContext inputContext;
   protected final AVStream streamToDecode;
   protected final AVCodec decoder;
   protected final AVCodecContext decoderContext;

   protected final AVPacket nextPacket;
   protected final AVFrame decodedFrame;

   private Function<AVPacket, Integer> packetProvider;
   private boolean sendAPacket = true;

   protected int error;

   public FFmpegDecoder(AVFormatContext inputContext, int wantedStreamIndex, int relatedStreamIndex, int avMediaType)
   {
      this.inputContext = inputContext;

      // Find the stream to decode, and decoder to use
      decoder = new AVCodec();
      int streamIndex = av_find_best_stream(inputContext, avMediaType, wantedStreamIndex, relatedStreamIndex, decoder, 0);
      FFmpegTools.checkNegativeError(streamIndex, "Finding best stream index");
      FFmpegTools.checkPointer(decoder, "Finding stream decoder");
      streamToDecode = inputContext.streams(streamIndex);

      // Create decoder context
      decoderContext = avcodec_alloc_context3(decoder);
      FFmpegTools.checkPointer(decoderContext, "Allocating codec context");

      // Copy stream parameters to codec context
      error = avcodec_parameters_to_context(decoderContext, streamToDecode.codecpar());
      FFmpegTools.checkNegativeError(error, "Copying parameters from stream to codec");

      // Allocate space for packet and frame
      nextPacket = av_packet_alloc();
      FFmpegTools.checkPointer(nextPacket, "Allocating encoded packet");
      decodedFrame = av_frame_alloc();
      FFmpegTools.checkPointer(decodedFrame, "Allocating decoded frame");
   }

   public void initialize(AVDictionary codecOptions, Function<AVPacket, Integer> packetProvider)
   {
      this.packetProvider = packetProvider;

      AVDictionary optionsCopy = new AVDictionary();
      av_dict_copy(optionsCopy, codecOptions, 0);
      error = avcodec_open2(decoderContext, decoder, optionsCopy);
      FFmpegTools.checkNegativeError(error, "Opening codec");
      FFmpegTools.checkDictionaryAfterUse(optionsCopy);
      av_dict_free(optionsCopy);
   }

   protected boolean decodeNextFrame(Consumer<AVFrame> frameConsumer)
   {
      do
      {
         if (sendAPacket)
         {
            // Send a packet
            error = sendNextPacket();

            // Handle the errors
            if (error == AVERROR_EOF())
               return false;
            FFmpegTools.checkNegativeError(error, "Sending packet");
         }

         // Try to receive decoded frame
         error = avcodec_receive_frame(decoderContext, decodedFrame);

         // Handle errors
         if (error == AVERROR_EOF()) // No more frames to receive, don't throw exceptions
            return false;
         else if (error != AVERROR_EAGAIN())// Bad error. Throw exceptions
            FFmpegTools.checkNegativeError(error, "Receiving decoded frame");

         // Go again if libav tells us to
         sendAPacket = error == AVERROR_EAGAIN();
      } while (sendAPacket);

      // Do whatever with the frame
      frameConsumer.accept(decodedFrame);
      return true;
   }

   private int sendNextPacket()
   {
      // Get a packet from the stream we're decoding
      do
      {
         error = packetProvider.apply(nextPacket);
         if (error < 0)
            return error;
      }
      while (nextPacket.stream_index() != streamToDecode.index());

      // Send the packet to the decoder
      error = avcodec_send_packet(decoderContext, nextPacket);
      return error;
   }

   public void destroy()
   {
      // Flush the decoder
      avcodec_flush_buffers(decoderContext);

      // Free everything
      avcodec_close(decoderContext);
      avcodec_free_context(decoderContext);
      av_frame_free(decodedFrame);
      av_packet_free(nextPacket);

      // Close everything
      decoder.close();
      decoderContext.close();
      decodedFrame.close();
      nextPacket.close();
   }
}
