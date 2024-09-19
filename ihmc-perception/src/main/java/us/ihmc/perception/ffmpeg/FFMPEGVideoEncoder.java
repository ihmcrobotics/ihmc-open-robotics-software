package us.ihmc.perception.ffmpeg;

import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avformat.AVStream;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVFrame;
import org.bytedeco.ffmpeg.swscale.SwsContext;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.Conversions;

import static org.bytedeco.ffmpeg.global.avcodec.avcodec_open2;
import static org.bytedeco.ffmpeg.global.avcodec.avcodec_parameters_from_context;
import static org.bytedeco.ffmpeg.global.avutil.*;
import static org.bytedeco.ffmpeg.global.swscale.*;

public class FFMPEGVideoEncoder extends FFMPEGEncoder
{
   private static final int SCALE_METHOD = SWS_BICUBIC;

   private final int outputWidth;
   private final int outputHeight;
   private final int outputPixelFormat;

   private final int groupOfPicturesSize;
   private final int maxBFrames;

   private final int inputPixelFormat;

   private final AVFrame inputFrame;

   private SwsContext swsContext;

   private long firstFrameTime = -1L;

   public FFMPEGVideoEncoder(AVOutputFormat outputFormat,
                             String preferredCodecName,
                             int bitRate,
                             int outputWidth,
                             int outputHeight,
                             int outputPixelFormat,
                             int groupOfPicturesSize,
                             int maxBFrames,
                             int inputPixelFormat)
   {
      super(outputFormat, preferredCodecName, bitRate);

      this.outputWidth = outputWidth;
      this.outputHeight = outputHeight;
      this.outputPixelFormat = outputPixelFormat;
      this.groupOfPicturesSize = groupOfPicturesSize;
      this.maxBFrames = maxBFrames;
      this.inputPixelFormat = inputPixelFormat;

      inputFrame = av_frame_alloc();
      FFMPEGTools.checkPointer(inputFrame, "Allocating input frame");
   }

   @Override
   public void initialize(AVDictionary codecOptions)
   {
      timeBase = av_make_q(1, (int) Conversions.secondsToNanoseconds(1.0));

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

      AVDictionary optionsCopy = new AVDictionary();
      av_dict_copy(optionsCopy, codecOptions, 0);
      error = avcodec_open2(encoderContext, encoder, optionsCopy);
      FFMPEGTools.checkNegativeError(error, "Opening codec");
      FFMPEGTools.checkDictionaryAfterUse(optionsCopy);
      av_dict_free(optionsCopy);
   }

   @Override
   public AVStream newStream(AVFormatContext outputContext)
   {
      AVStream stream = super.newStream(outputContext);

      error = avcodec_parameters_from_context(stream.codecpar(), encoderContext);
      FFMPEGTools.checkNegativeError(error, "Copying parameters from codec to stream");

      return stream;
   }

   public void setNextFrame(Mat frame)
   {
      long currentTime = System.nanoTime();

      if (inputFrame.width() < 0 && inputFrame.height() < 0)
      {
         initializeInputFrame(frame);
         firstFrameTime = currentTime;
      }

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

      long timeElapsed = currentTime - firstFrameTime;
      nextFrame.pts(timeElapsed);
   }

   private void initializeInputFrame(Mat firstFrame)
   {
      int inputWidth = firstFrame.cols();
      int inputHeight = firstFrame.rows();

      inputFrame.width(inputWidth);
      inputFrame.height(inputHeight);
      error = av_frame_get_buffer(inputFrame, 0);
      FFMPEGTools.checkNegativeError(error, "Getting input frame buffer");

      if (inputWidth != outputWidth || inputHeight != outputHeight || inputPixelFormat != outputPixelFormat)
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
}
