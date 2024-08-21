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
import org.bytedeco.ffmpeg.swscale.SwsContext;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.perception.RawImage;

import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avformat.avformat_new_stream;
import static org.bytedeco.ffmpeg.global.avutil.*;
import static org.bytedeco.ffmpeg.global.swscale.*;

public class FFMPEGVideoOutputStream
{
   private static final int SCALE_METHOD = SWS_BICUBIC;

   private final AVStream stream;
   private final AVCodec codec;
   private final AVCodecContext codecContext;

   private long nextPresentationTimestamp = 0L;
   private final AVFrame inputFrame;
   private final AVFrame nextFrame;
   private final AVPacket nextPacket;

   private final AVRational frameRate;
   private final AVRational timeBase;

   private SwsContext swsContext;

   private int error;

   public FFMPEGVideoOutputStream(int inputPixelFormat,
                                  AVFormatContext formatContext,
                                  int codecID,
                                  int outputWidth,
                                  int outputHeight,
                                  int bitRate,
                                  double outputFrameRate,
                                  int outputPixelFormat,
                                  AVDictionary codecOptions)
   {
      frameRate = avutil.av_d2q(outputFrameRate, 4096);
      timeBase = avutil.av_inv_q(frameRate);

      stream = avformat_new_stream(formatContext, null);
      FFMPEGTools.checkPointer(stream, "Creating new stream");
      stream.id(formatContext.nb_streams() - 1);
      stream.time_base(timeBase);

      codec = avcodec_find_encoder(codecID);
      FFMPEGTools.checkPointer(codec, "Finding encoder");

      codecContext = avcodec_alloc_context3(codec);
      FFMPEGTools.checkPointer(codecContext, "Allocating codec context");

      codecContext.codec_id(codecID);
      codecContext.bit_rate(bitRate);
      codecContext.width(outputWidth);
      codecContext.height(outputHeight);
      codecContext.time_base(timeBase);
      codecContext.pix_fmt(outputPixelFormat);

      if ((formatContext.oformat().flags() & avformat.AVFMT_GLOBALHEADER) != 0)
         codecContext.flags(codecContext.flags() | avcodec.AV_CODEC_FLAG_GLOBAL_HEADER);

      inputFrame = av_frame_alloc();
      FFMPEGTools.checkPointer(inputFrame, "Allocating input frame");
      inputFrame.format(inputPixelFormat);
      inputFrame.width(-1);
      inputFrame.height(-1);
      // input frame width & height initialized upon reception of first image

      nextFrame = av_frame_alloc();
      FFMPEGTools.checkPointer(inputFrame, "Allocating next frame");
      nextFrame.format(outputPixelFormat);
      nextFrame.width(outputWidth);
      nextFrame.height(outputHeight);
      error = av_frame_get_buffer(nextFrame, 0);
      FFMPEGTools.checkNegativeError(error, "Getting next frame buffer");

      nextPacket = av_packet_alloc();
      FFMPEGTools.checkPointer(nextPacket, "Allocating next packet");

      AVDictionary options = new AVDictionary();
      av_dict_copy(options, codecOptions, 0);
      error = avcodec_open2(codecContext, codec, options);
      av_dict_free(options);
      FFMPEGTools.checkNegativeError(error, "Opening codec");
   }

   public void addFrame(Mat frame)
   {
      if (inputFrame.width() < 0 && inputFrame.height() < 0)
         initializeInputFrame(frame);

      if (swsContext != null)
      {
         Mat frameCopy = frame.clone();
         inputFrame.data(0, frameCopy.data());
         error = sws_scale(swsContext, inputFrame.data(), inputFrame.linesize(), 0, nextFrame.height(), nextFrame.data(), nextFrame.linesize());
         frameCopy.close();
         FFMPEGTools.checkNegativeError(error, "Scaling frame");
      }
      else
      {
         nextFrame.data(0, frame.clone().data());
      }

      nextFrame.pts(nextPresentationTimestamp++);
   }

   private void initializeInputFrame(Mat firstFrame)
   {
      int width = firstFrame.cols();
      int height = firstFrame.rows();

      inputFrame.width(width);
      inputFrame.height(height);
      error = av_frame_get_buffer(inputFrame, 0);
      FFMPEGTools.checkNegativeError(error, "Getting input frame buffer");

      if (inputFrame.format() != nextFrame.format())
         initializeSwsContext();
   }

   private void initializeSwsContext()
   {
      swsContext = sws_getContext(inputFrame.width(),
                                  inputFrame.height(),
                                  inputFrame.format(),
                                  codecContext.width(),
                                  codecContext.height(),
                                  codecContext.pix_fmt(),
                                  SCALE_METHOD,
                                  null,
                                  null,
                                  (DoublePointer) null);
      FFMPEGTools.checkPointer(swsContext, "Initializing swsContext");
   }

   public AVCodecContext getCodecContext()
   {
      return codecContext;
   }

   public AVStream getStream()
   {
      return stream;
   }

   public AVFrame getNextFrame()
   {
      return nextFrame;
   }

   public AVPacket getNextPacket()
   {
      return nextPacket;
   }

   public void destroy()
   {
      avcodec_free_context(codecContext);
      av_frame_free(inputFrame);
      av_frame_free(nextFrame);
      av_packet_free(nextPacket);

      codec.close();
      codecContext.close();
      inputFrame.close();
      nextFrame.close();
      stream.close();
      frameRate.close();
      timeBase.close();

      if (swsContext != null)
      {
         sws_freeContext(swsContext);
         swsContext.close();
      }
   }
}
