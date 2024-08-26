package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVCodec;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVFrame;
import org.bytedeco.ffmpeg.global.avcodec;
import org.bytedeco.ffmpeg.global.avformat;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.ffmpeg.swscale.SwsContext;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.log.LogTools;

import static org.bytedeco.ffmpeg.global.avcodec.*;
import static org.bytedeco.ffmpeg.global.avutil.*;
import static org.bytedeco.ffmpeg.global.swscale.*;

public class FFMPEGVideoEncoder extends FFMPEGEncoder
{
   private static final int SCALE_METHOD = SWS_BICUBIC;

   private final int outputWidth;
   private final int outputHeight;
   private final int outputPixelFormat;
   private final double outputFrameRate;

   private final int groupOfPicturesSize;
   private final int maxBFrames;

   private final int inputPixelFormat;

   private final AVFrame inputFrame;

   private SwsContext swsContext;

   public FFMPEGVideoEncoder(AVFormatContext formatContext,
                             String preferredCodecName,
                             int bitRate,
                             int outputWidth,
                             int outputHeight,
                             int outputPixelFormat,
                             double outputFrameRate,
                             int groupOfPicturesSize,
                             int maxBFrames,
                             int inputPixelFormat)
   {
      super(formatContext, preferredCodecName, bitRate);

      this.outputWidth = outputWidth;
      this.outputHeight = outputHeight;
      this.outputPixelFormat = outputPixelFormat;
      this.outputFrameRate = outputFrameRate;
      this.groupOfPicturesSize = groupOfPicturesSize;
      this.maxBFrames = maxBFrames;
      this.inputPixelFormat = inputPixelFormat;

      inputFrame = av_frame_alloc();
      FFMPEGTools.checkPointer(inputFrame, "Allocating input frame");
   }

   @Override
   public void initialize(AVDictionary codecOptions)
   {
      timeBase = avutil.av_inv_q(avutil.av_d2q(outputFrameRate, 4096));
      stream.time_base(timeBase);

      if ((formatContext.oformat().flags() & avformat.AVFMT_GLOBALHEADER) != 0)
         encoderContext.flags(encoderContext.flags() | avcodec.AV_CODEC_FLAG_GLOBAL_HEADER);

      inputFrame.format(inputPixelFormat);
      // input frame width & height initialized upon reception of first image
      inputFrame.width(-1);
      inputFrame.height(-1);

      nextFrame.format(outputPixelFormat);
      nextFrame.width(outputWidth);
      nextFrame.height(outputHeight);
      error = av_frame_get_buffer(nextFrame, 0);
      FFMPEGTools.checkNegativeError(error, "Getting next frame buffer");

      encoderContext.bit_rate(bitRate);
      encoderContext.codec_id(encoder.id());
      encoderContext.width(outputWidth);
      encoderContext.height(outputHeight);
      encoderContext.time_base(timeBase);
      encoderContext.pix_fmt(outputPixelFormat);
      encoderContext.gop_size(groupOfPicturesSize);
      encoderContext.max_b_frames(maxBFrames);

      error = avcodec_parameters_from_context(stream.codecpar(), encoderContext);
      FFMPEGTools.checkNegativeError(error, "Copying parameters from codec to stream");

      AVDictionary options = new AVDictionary();
      av_dict_copy(options, codecOptions, 0);
      error = avcodec_open2(encoderContext, encoder, options);
      av_dict_free(options);
      FFMPEGTools.checkNegativeError(error, "Opening codec");
   }

   public void setNextFrame(Mat frame)
   {
      assignNextFrame(nextFrame ->
      {
         if (inputFrame.width() < 0 && inputFrame.height() < 0)
            initializeInputFrame(frame);

         if (swsContext != null)
         {
            inputFrame.data(0, frame.data());
            error = sws_scale_frame(swsContext, nextFrame, inputFrame);
            FFMPEGTools.checkNegativeError(error, "Scaling frame");
         }
         else
         {
            nextFrame.data(0, frame.data());
         }
      });
   }

   private void initializeInputFrame(Mat firstFrame)
   {
      int width = firstFrame.cols();
      int height = firstFrame.rows();

      inputFrame.width(width);
      inputFrame.height(height);
      error = av_frame_get_buffer(inputFrame, 0);
      FFMPEGTools.checkNegativeError(error, "Getting input frame buffer");

      if (inputPixelFormat != outputPixelFormat)
         initializeSwsContext();
   }

   private void initializeSwsContext()
   {
      swsContext = sws_getContext(inputFrame.width(),
                                  inputFrame.height(),
                                  inputFrame.format(),
                                  encoderContext.width(),
                                  encoderContext.height(),
                                  encoderContext.pix_fmt(),
                                  SCALE_METHOD,
                                  null,
                                  null,
                                  (DoublePointer) null);
      FFMPEGTools.checkPointer(swsContext, "Initializing swsContext");
   }

   @Override
   public void destroy()
   {
      super.destroy();

      av_frame_free(inputFrame);
      inputFrame.close();

      if (swsContext != null)
      {
         sws_freeContext(swsContext);
         swsContext.close();
      }
   }

   @Override
   protected AVCodec findEncoder(String preferredEncoderName, AVOutputFormat outputFormat)
   {
      AVCodec encoder = null;

      if (preferredEncoderName != null)
      {
         encoder = avcodec_find_encoder_by_name(preferredEncoderName);

         if (encoder != null && !encoder.isNull())
         {
            if (outputFormat.video_codec() != encoder.id())
               outputFormat.video_codec(encoder.id()); // FIXME: This doesn't work
            LogTools.info("Found encoder " + preferredEncoderName + " - id:" + encoder.id());
         }
         else
            LogTools.error("Failed to find valid encoder " + preferredEncoderName + " - attempting to default to another");
      }

      if (preferredEncoderName == null || encoder == null || encoder.isNull())
      {
         encoder = avcodec_find_encoder(outputFormat.video_codec());
      }

      return encoder;
   }
}
