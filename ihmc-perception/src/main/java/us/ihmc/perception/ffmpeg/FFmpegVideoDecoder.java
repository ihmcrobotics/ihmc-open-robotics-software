package us.ihmc.perception.ffmpeg;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVFrame;
import org.bytedeco.ffmpeg.avutil.AVFrameSideData;
import org.bytedeco.ffmpeg.avutil.AVRational;
import org.bytedeco.ffmpeg.swscale.SwsContext;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.Conversions;

import java.util.function.Function;

import static org.bytedeco.ffmpeg.global.avutil.*;
import static org.bytedeco.ffmpeg.global.swscale.*;

public class FFmpegVideoDecoder extends FFmpegDecoder
{
   private static final int SCALE_METHOD = SWS_BICUBIC;

   private int inputWidth = -1;
   private int inputHeight = -1;
   private int inputPixelFormat = -1;

   private SwsContext swsContext;

   private int outputWidth;
   private int outputHeight;
   private int outputPixelFormat;
   private final AVFrame outputFrame;
   private Mat outputImage;
   private BytePointer lastFrameSideData;

   private final AVRational millisecondTimebase;
   private long lastFrameTimestamp = -1L;

   public FFmpegVideoDecoder(AVFormatContext inputContext, int outputPixelFormat)
   {
      this(inputContext, -1, -1, -1, -1, outputPixelFormat);
   }

   public FFmpegVideoDecoder(AVFormatContext inputContext,
                             int wantedStreamIndex,
                             int relatedStreamIndex,
                             int outputWidth,
                             int outputHeight,
                             int outputPixelFormat)
   {
      super(inputContext, wantedStreamIndex, relatedStreamIndex, AVMEDIA_TYPE_VIDEO);

      this.outputWidth = outputWidth;
      this.outputHeight = outputHeight;
      this.outputPixelFormat = outputPixelFormat;

      millisecondTimebase = av_make_q(1, (int) Conversions.secondsToMilliseconds(1.0));

      outputFrame = av_frame_alloc();
      FFmpegTools.checkPointer(outputFrame, "Allocating output frame");

      lastFrameSideData = new BytePointer();
   }

   @Override
   public void initialize(AVDictionary codecOptions, Function<AVPacket, Integer> packetProvider)
   {
      super.initialize(codecOptions, packetProvider);

      inputWidth = decoderContext.width();
      inputHeight = decoderContext.height();
      inputPixelFormat = decoderContext.pix_fmt();

      if (outputWidth <= 0)
         outputWidth = inputWidth;
      if (outputHeight <= 0)
         outputHeight = inputHeight;
      if (outputPixelFormat < 0)
         outputPixelFormat = inputPixelFormat;

      if (inputWidth != outputWidth || inputHeight != outputHeight || inputPixelFormat != outputPixelFormat)
         initializeSwsContext();
   }

   private void initializeSwsContext()
   {
      swsContext = sws_getContext(inputWidth,
                                  inputHeight,
                                  inputPixelFormat,
                                  outputWidth,
                                  outputHeight,
                                  outputPixelFormat,
                                  SCALE_METHOD,
                                  null,
                                  null,
                                  (DoublePointer) null);
      FFmpegTools.checkPointer(swsContext, "Initializing swsContext");
   }

   public Mat getNextFrame()
   {
      if (outputImage != null)
         outputImage.close();

      MutableBoolean gotAFrame = new MutableBoolean();
      gotAFrame.setValue(decodeNextFrame(decodedFrame ->
      {  // Ensure the pixel format was identified
         if (decodedFrame.format() < 0 || decodedFrame.format() > AV_PIX_FMT_NB)
         {
            gotAFrame.setFalse();
            return;
         }

         lastFrameTimestamp = av_rescale_q(decodedFrame.best_effort_timestamp(), streamToDecode.time_base(), millisecondTimebase);
         extractSideData(decodedFrame);

         if (swsContext != null)
         {
            error = sws_scale_frame(swsContext, outputFrame, decodedFrame);
            FFmpegTools.checkNegativeError(error, "Scaling decoded frame to output frame");
            outputImage = FFmpegTools.avFrameToMat(outputFrame);
         }
         else
         {
            error = av_frame_ref(outputFrame, decodedFrame);
            FFmpegTools.checkNegativeError(error, "Copying decoded frame to output frame");
            outputImage = FFmpegTools.avFrameToMat(outputFrame);
            av_frame_unref(outputFrame);
         }
      }));

      if (!gotAFrame.getValue())
         return null;

      return outputImage;
   }

   /**
    * @return The timestamp of the last frame gotten through {@link #getNextFrame()}
    */
   public long getLastFrameTimestamp()
   {
      return lastFrameTimestamp;
   }

   public BytePointer getLastFrameSideData()
   {
      return lastFrameSideData;
   }

   @Override
   public void destroy()
   {
      super.destroy();

      av_frame_free(outputFrame);
      outputFrame.close();

      if (outputImage != null)
         outputImage.close();

      if (swsContext != null)
      {
         sws_freeContext(swsContext);
         swsContext.close();
      }
   }

   private void extractSideData(AVFrame decodedFrame)
   {
      lastFrameSideData.close();
      AVFrameSideData sideData;
      for (int i = 0; i < decodedFrame.nb_side_data(); ++i)
      {
         sideData = decodedFrame.side_data(i);
         if (!sideData.isNull() && sideData.type() == AV_FRAME_DATA_SEI_UNREGISTERED)
         {
            lastFrameSideData = new BytePointer(sideData.data().position(16).limit(sideData.size()));
            break;
         }
      }
   }
}
