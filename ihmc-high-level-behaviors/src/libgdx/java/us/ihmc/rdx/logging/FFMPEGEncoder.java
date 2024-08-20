package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avcodec.AVCodecContext;
import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avutil.AVFrame;
import org.bytedeco.ffmpeg.avutil.AVRational;
import org.bytedeco.ffmpeg.global.avcodec;
import org.bytedeco.ffmpeg.global.avutil;
import us.ihmc.perception.RawImage;

import java.util.ArrayList;
import java.util.List;

public class FFMPEGEncoder
{

   private final AVCodec codec;
   private final AVCodecContext codecContext;

   private final AVFrame frameToEncode;
   private final AVPacket packet;

   private int error = 0;
   private boolean initialized = false;
   private long sequenceNumber = 0L;

   private final List<Byte> encodedData = new ArrayList<>();

   public FFMPEGEncoder(String codecName, float targetFrameRate, int bitRate, int groupSize, int maxBFrames, int sourcePixelFormat, int encoderPixelFormat)
   {
      codec = avcodec.avcodec_find_encoder_by_name(codecName);
      FFMPEGTools.checkPointer(codec, "Finding codec by name");

      AVRational inputFrameRate = avutil.av_d2q(targetFrameRate, 4096);
      AVRational inputTimeBase = avutil.av_inv_q(inputFrameRate);

      codecContext = avcodec.avcodec_alloc_context3(codec);
      FFMPEGTools.checkPointer(codecContext, "Allocating codec context");
      codecContext.codec_id(codec.id());
      codecContext.bit_rate(bitRate);
      codecContext.gop_size(groupSize);
      codecContext.max_b_frames(maxBFrames);
      codecContext.pix_fmt(encoderPixelFormat);
      codecContext.framerate(inputFrameRate);
      codecContext.time_base(inputTimeBase);

      packet = avcodec.av_packet_alloc();
      FFMPEGTools.checkPointer(packet, "Allocating packet");

      frameToEncode = avutil.av_frame_alloc();
      FFMPEGTools.checkPointer(frameToEncode, "Allocating frame");
      frameToEncode.format(sourcePixelFormat);

      inputFrameRate.close();
      inputTimeBase.close();
   }

   private void initialize(RawImage firstImage)
   {
      int width = firstImage.getImageWidth();
      int height = firstImage.getImageHeight();

      codecContext.width(width);
      codecContext.height(height);

      frameToEncode.width(width);
      frameToEncode.height(height);

      error = avutil.av_frame_get_buffer(frameToEncode, 0);
      FFMPEGTools.checkNegativeError(error, "Getting frame buffer");

      initialized = true;
   }

   public void setNextFrame(RawImage image)
   {
      if (!initialized)
         initialize(image);

      error = avutil.av_frame_make_writable(frameToEncode);
      FFMPEGTools.checkNegativeError(error, "Making frame writable");

      frameToEncode.data(0, image.getCpuImageMat().data());
      frameToEncode.pts(sequenceNumber++);

      error = avcodec.avcodec_send_frame(codecContext, frameToEncode);
      FFMPEGTools.checkNegativeError(error, "Sending frame to encoder");
      while (error >= 0)
      {
         error = avcodec.avcodec_receive_packet(codecContext, packet);

         int packetSize = packet.size();
         for (int i = 0; i < packetSize; ++i)
         {
            encodedData.add(packet.data().get(i));
         }
         avcodec.av_packet_unref(packet);
      }
   }

   public List<Byte> getEncodedData()
   {
      return encodedData;
   }

   public void destroy()
   {
      // TODO: Destroy everything
   }

   private AVRational toRational(float value)
   {
      return new AVRational().num(Math.round(value)).den(1);
   }
}
