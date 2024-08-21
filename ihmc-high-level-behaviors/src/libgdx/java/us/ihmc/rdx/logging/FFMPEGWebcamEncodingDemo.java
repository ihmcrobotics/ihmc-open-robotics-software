package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avformat.AVOutputFormat;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.global.avformat;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;

import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.av_dict_free;

public class FFMPEGWebcamEncodingDemo
{
   private static final String RESULT_FILE_DIRECTORY = System.getProperty("user.home") + File.separator + "Videos" + File.separator;

   private final VideoCapture videoCapture;

   private final AVFormatContext outputContext;
   private final AVOutputFormat outputFormat;
   private final FFMPEGVideoOutputStream outputStream;

   private int error;

   public FFMPEGWebcamEncodingDemo()
   {
      String resultFileName = "FFMPEGWebcamEncodingDemo_" + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date()) + ".mpeg";
      String resultFilePath = RESULT_FILE_DIRECTORY + resultFileName;
      String formatName = resultFileName.substring(resultFileName.lastIndexOf('.') + 1);

      videoCapture = new VideoCapture(-1);

      int imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
      int imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
      double reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);

      outputContext = new AVFormatContext();
      error = avformat.avformat_alloc_output_context2(outputContext, null, formatName, resultFileName);
      FFMPEGTools.checkPointer(outputContext, "Allocating output context");
      FFMPEGTools.checkNegativeError(error, "Allocating output context");

      outputFormat = outputContext.oformat();

      outputStream = new FFMPEGVideoOutputStream(avutil.AV_PIX_FMT_BGR24,
                                                 outputContext,
                                                 outputFormat.video_codec(),
                                                 imageWidth,
                                                 imageHeight,
                                                 400000,
                                                 reportedFPS,
                                                 avutil.AV_PIX_FMT_YUV420P,
                                                 null);

      FileTools.ensureDirectoryExists(Paths.get(resultFilePath).getParent(), DefaultExceptionHandler.RUNTIME_EXCEPTION);

      AVIOContext avBytestreamIOContext = new AVIOContext();
      error = avio_open(avBytestreamIOContext, resultFilePath, AVIO_FLAG_WRITE);
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
         outputStream.addFrame(frame);
         notDone = FFMPEGEncoder.encodeVideoFrame(outputStream, packet ->
         {
            error = av_interleaved_write_frame(outputContext, packet);
            FFMPEGTools.checkNegativeError(error, "Writing packet");
         });
      }

      av_write_trailer(outputContext);

      outputStream.destroy();
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
