package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avcodec.AVCodecContext;
import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVFrame;

import java.util.function.Consumer;

import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avformat.av_find_best_stream;
import static org.bytedeco.ffmpeg.global.avutil.*;

public class FFMPEGDecoder
{
   protected final AVFormatContext inputContext;
   protected final AVStream streamToDecode;
   protected final AVCodec decoder;
   protected final AVCodecContext decoderContext;

   protected final AVPacket nextPacket;
   protected final AVFrame decodedFrame;

   protected int error;

   private Consumer<AVPacket> packetProvider;

   public FFMPEGDecoder(AVFormatContext intputContext, int wantedStreamIndex, int relatedStreamIndex, int avMediaType)
   {
      this.inputContext = intputContext;

      // Find the stream to decode, and decoder to use
      decoder = new AVCodec();
      int streamIndex = av_find_best_stream(inputContext, avMediaType, wantedStreamIndex, relatedStreamIndex, decoder, 0);
      FFMPEGTools.checkNegativeError(streamIndex, "Finding best stream index");
      FFMPEGTools.checkPointer(decoder, "Finding stream decoder");
      streamToDecode = intputContext.streams(streamIndex);

      // Create decoder context
      decoderContext = avcodec_alloc_context3(decoder);
      FFMPEGTools.checkPointer(decoderContext, "Allocating codec context");

      // Copy stream parameters to codec context
      error = avcodec_parameters_to_context(decoderContext, streamToDecode.codecpar());
      FFMPEGTools.checkNegativeError(error, "Copying parameters from stream to codec");

      // Allocate space for packet and frame
      nextPacket = av_packet_alloc();
      FFMPEGTools.checkPointer(nextPacket, "Allocating encoded packet");
      decodedFrame = av_frame_alloc();
      FFMPEGTools.checkPointer(decodedFrame, "Allocating decoded frame");
   }

   public void initialize(AVDictionary codecOptions, Consumer<AVPacket> packetProvider)
   {
      this.packetProvider = packetProvider;

      AVDictionary optionsCopy = new AVDictionary();
      av_dict_copy(optionsCopy, codecOptions, 0);
      error = avcodec_open2(decoderContext, decoder, optionsCopy);
      av_dict_free(optionsCopy);
      FFMPEGTools.checkNegativeError(error, "Opening codec");
   }

   protected boolean decodePacket(Consumer<AVFrame> frameConsumer)
   {
      // Get the next packet
      packetProvider.accept(nextPacket);

      // Don't decode packets from another stream
      if (nextPacket.stream_index() != streamToDecode.index())
         return false;

      // Send the packet to be decoded
      error = avcodec_send_packet(decoderContext, nextPacket);
      FFMPEGTools.checkNegativeError(error, "Sending packet for decoding");

      while (error >= 0)
      {
         // Try to receive decoded frame
         error = avcodec_receive_frame(decoderContext, decodedFrame);

         // Handle any errors
         if (error == AVERROR_EAGAIN() || error == AVERROR_EOF()) // No errors, but no frame received
            break;
         else // bad errors
            FFMPEGTools.checkNegativeError(error, "Receiving decoded frame");

         // Do whatever with the frame
         frameConsumer.accept(decodedFrame);

         // Unreference the frame
         av_frame_unref(decodedFrame);
      }

      return error == AVERROR_EAGAIN();
   }

   public void destroy()
   {
      // TODO:
   }
}
