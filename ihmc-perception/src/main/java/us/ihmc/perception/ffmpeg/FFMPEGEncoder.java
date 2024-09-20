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
import org.bytedeco.javacpp.Pointer;
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
   protected final AVCodec encoder;
   protected final AVCodecContext encoderContext;

   protected final AVFrame frameToEncode;
   protected final AVPacket encodedPacket;

   protected AVRational timeBase;
   protected final int bitRate;

   protected int error;

   public FFMPEGEncoder(AVOutputFormat outputFormat, String preferredEncoderName, int bitRate)
   {
      this.bitRate = bitRate;

      encoder = findEncoder(preferredEncoderName, outputFormat);
      FFMPEGTools.checkPointer(encoder, "Finding encoder");

      encodedPacket = av_packet_alloc();
      FFMPEGTools.checkPointer(encodedPacket, "Allocating next packet");

      encoderContext = avcodec_alloc_context3(encoder);
      FFMPEGTools.checkPointer(encoderContext, "Allocating codec context");

      frameToEncode = av_frame_alloc();
      FFMPEGTools.checkPointer(frameToEncode, "Allocating input frame");

      if ((outputFormat.flags() & avformat.AVFMT_GLOBALHEADER) != 0)
         encoderContext.flags(encoderContext.flags() | avcodec.AV_CODEC_FLAG_GLOBAL_HEADER);
   }

   /**
    * Initialize the encoder with default options
    */
   public void initialize()
   {
      initialize(null);
   }

   /**
    * Initialize the encoder with provided options
    * @param codecOptions Options for the encoder.
    */
   public void initialize(AVDictionary codecOptions)
   {
      AVDictionary optionsCopy = new AVDictionary();
      av_dict_copy(optionsCopy, codecOptions, 0);
      error = avcodec_open2(encoderContext, encoder, optionsCopy);
      FFMPEGTools.checkNegativeError(error, "Opening codec");
      FFMPEGTools.checkDictionaryAfterUse(optionsCopy);
      av_dict_free(optionsCopy);
   }

   /**
    * Get a new stream to go with the provided output context.
    * @param outputContext Output context of the stream to be created.
    * @return An AVStream from the encoder with the provided output context.
    */
   public AVStream newStream(AVFormatContext outputContext)
   {
      AVStream stream = avformat_new_stream(outputContext, encoder);
      FFMPEGTools.checkPointer(stream, "Creating new stream");

      return stream;
   }

   /**
    * Assign the frame to be encoded with the provided data.
    * @param data Pointer to an object containing information about the frame to encode.
    */
   public abstract void setNextFrame(Pointer data);

   public boolean encodeAndWriteNextFrame(AVFormatContext outputContext, AVStream stream)
   {
      return encodeNextFrame(packet ->
      {
         av_packet_rescale_ts(encodedPacket, encoderContext.time_base(), stream.time_base());
         encodedPacket.stream_index(stream.index());

         error = av_interleaved_write_frame(outputContext, packet);
         FFMPEGTools.checkNegativeError(error, "Writing packet");
      });
   }

   /**
    * Encode the frame assigned using {@link #setNextFrame(Pointer)}.
    * @param packetConsumer Method to use the encoded packets generated from the frame.
    * @return true if the encoder can keep going, false if EOF was reached.
    */
   public boolean encodeNextFrame(Consumer<AVPacket> packetConsumer)
   {
      int error;
      error = avcodec_send_frame(encoderContext, frameToEncode);
      FFMPEGTools.checkNegativeError(error, "Sending frame");

      while (error >= 0)
      {
         error = avcodec_receive_packet(encoderContext, encodedPacket);
         if (error == AVERROR_EAGAIN() || error == AVERROR_EOF())
            break;
         else
            FFMPEGTools.checkNegativeError(error, "Receiving packet");

         if (packetConsumer != null)
            packetConsumer.accept(encodedPacket);

         av_packet_unref(encodedPacket);
      }

      return error != AVERROR_EOF();
   }

   public AVRational getTimeBase()
   {
      return encoderContext.time_base();
   }

   public void destroy()
   {
      frameToEncode.close();
      encodeNextFrame(null);

      avcodec_close(encoderContext);
      avcodec_free_context(encoderContext);
      av_frame_free(frameToEncode);
      av_packet_free(encodedPacket);

      encoder.close();
      encoderContext.close();
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
