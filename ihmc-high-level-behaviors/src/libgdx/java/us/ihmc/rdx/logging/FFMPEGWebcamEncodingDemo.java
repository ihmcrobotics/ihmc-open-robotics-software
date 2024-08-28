package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;

import java.io.File;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;

import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.*;

public class FFMPEGWebcamEncodingDemo
{
   private static final String RESULT_FILE_DIRECTORY = System.getProperty("user.home") + File.separator + "Videos" + File.separator;
   private static final String RESULT_FILE_NAME = "FFMPEGWebcamEncodingDemo_" + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date()) + ".h264";
   private static final String RESULT_FILE_PATH = RESULT_FILE_DIRECTORY + RESULT_FILE_NAME;

   public FFMPEGWebcamEncodingDemo()
   {
      int error;

      VideoCapture videoCapture = new VideoCapture(-1);

      int imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
      int imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
      double reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);

      AVFormatContext outputContext = new AVFormatContext();
      error = avformat_alloc_output_context2(outputContext, null, null, RESULT_FILE_NAME);
      FFMPEGTools.checkPointer(outputContext, "Allocating output context");
      FFMPEGTools.checkNegativeError(error, "Allocating output context");

      FFMPEGVideoEncoder videoEncoder = new FFMPEGVideoEncoder(outputContext,
                                                               "h264_nvenc",
                                                               400000,
                                                               imageWidth,
                                                               imageHeight,
                                                               AV_PIX_FMT_YUV420P,
                                                               reportedFPS,
                                                               10,
                                                               2,
                                                               AV_PIX_FMT_BGR24);

      videoEncoder.initialize();

      FileTools.ensureDirectoryExists(Paths.get(RESULT_FILE_PATH).getParent(), DefaultExceptionHandler.RUNTIME_EXCEPTION);

      AVIOContext avBytestreamIOContext = new AVIOContext();
      error = avio_open(avBytestreamIOContext, RESULT_FILE_PATH, AVIO_FLAG_WRITE);
      FFMPEGTools.checkNegativeError(error, "Opening output file");
      outputContext.pb(avBytestreamIOContext);

      AVDictionary streamFlags = new AVDictionary();
      avutil.av_dict_set(streamFlags, "lossless", "0", 0);
      error = avformat_write_header(outputContext, streamFlags);
      FFMPEGTools.checkNegativeError(error, "Writing header");

      boolean notDone = true;
      for (int i = 0; i < 180 && notDone; ++i)
      {
         System.out.println("Grabbing frame " + i);
         Mat frame = new Mat();
         videoCapture.read(frame);

         System.out.println("Encoding frame " + i);
         videoEncoder.setNextFrame(frame);
         notDone = videoEncoder.encodeAndWriteNextFrame();

         frame.close();
      }

      av_write_trailer(outputContext);

      videoEncoder.destroy();
      avio_closep(outputContext.pb());

      avformat_free_context(outputContext);

      av_dict_free(streamFlags);
      streamFlags.close();

      videoCapture.close();
   }

   public static void main(String[] args)
   {
      new FFMPEGWebcamEncodingDemo();
   }
}
