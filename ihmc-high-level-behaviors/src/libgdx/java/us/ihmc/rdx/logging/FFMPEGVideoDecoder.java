package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avcodec.AVPacket;
import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.avutil.AVFrame;
import org.bytedeco.ffmpeg.swscale.SwsContext;
import org.bytedeco.javacpp.DoublePointer;
import org.bytedeco.opencv.opencv_core.Mat;

import java.util.function.Consumer;

import static org.bytedeco.ffmpeg.global.avutil.*;
import static org.bytedeco.ffmpeg.global.swscale.*;

public class FFMPEGVideoDecoder extends FFMPEGDecoder
{
   private static final int SCALE_METHOD = SWS_BICUBIC;

   private int inputWidth = -1;
   private int inputHeight = -1;
   private int inputPixelFormat = -1;

   private SwsContext swsContext;

   private final int outputWidth;
   private final int outputHeight;
   private final int outputPixelFormat;
   private final AVFrame outputFrame;
   private Mat outputImage;

   public FFMPEGVideoDecoder(AVFormatContext intputContext,
                             int wantedStreamIndex,
                             int relatedStreamIndex,
                             int avMediaType,
                             int outputWidth,
                             int outputHeight,
                             int outputPixelFormat)
   {
      super(intputContext, wantedStreamIndex, relatedStreamIndex, avMediaType);

      this.outputWidth = outputWidth;
      this.outputHeight = outputHeight;
      this.outputPixelFormat = outputPixelFormat;

      outputFrame = av_frame_alloc();
      FFMPEGTools.checkPointer(outputFrame, "Allocating output frame");
   }

   @Override
   public void initialize(AVDictionary codecOptions, Consumer<AVPacket> packetProvider)
   {
      super.initialize(codecOptions, packetProvider);

      inputWidth = decoderContext.width();
      inputHeight = decoderContext.height();
      inputPixelFormat = decoderContext.pix_fmt();

      int openCVType = FFMPEGTools.avPixelFormatToOpenCVType(outputPixelFormat);
      outputImage = new Mat(inputHeight, inputWidth, openCVType);

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
      FFMPEGTools.checkPointer(swsContext, "Initializing swsContext");
   }

   public Mat getNextFrame()
   {
      boolean keepGoing;
      do
      {
         if (outputImage != null)
            outputImage.close();

         keepGoing = decodePacket(decodedFrame ->
         {
            if (swsContext != null)
            {
               error = sws_scale_frame(swsContext, outputFrame, decodedFrame);
               FFMPEGTools.checkNegativeError(error, "Scaling decoded frame to output frame");
               outputImage = FFMPEGTools.avFrameToMat(outputFrame);
            }
            else
            {
               error = av_frame_ref(outputFrame, decodedFrame);
               FFMPEGTools.checkNegativeError(error, "Copying decoded frame to output frame");
               outputImage = FFMPEGTools.avFrameToMat(outputFrame);
               av_frame_unref(outputFrame);
            }
         });
      } while (keepGoing);

      return outputImage.clone();
   }

   @Override
   public void destroy()
   {
      super.destroy();

      // TODO:
   }
}
