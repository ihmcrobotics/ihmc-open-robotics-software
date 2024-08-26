package us.ihmc.rdx.logging;

import org.bytedeco.ffmpeg.avformat.AVFormatContext;
import org.bytedeco.ffmpeg.avformat.AVIOContext;
import org.bytedeco.ffmpeg.avutil.AVDictionary;
import org.bytedeco.ffmpeg.global.avutil;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import us.ihmc.commons.thread.ThreadTools;

import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static org.bytedeco.ffmpeg.global.avformat.*;
import static org.bytedeco.ffmpeg.global.avutil.*;

public class FFMPEGWebcamStreamingDemo
{
   private final VideoCapture videoCapture;
   private final Mat image = new Mat();
   private long sequenceNumber = 0L;
   private final Lock newImageLock = new ReentrantLock();
   private final Condition newImageAvailable = newImageLock.newCondition();

   public FFMPEGWebcamStreamingDemo()
   {
      int error;

      videoCapture = new VideoCapture(-1);

      ThreadTools.startAThread(this::captureImage, "ImageCapture");

      int imageWidth = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
      int imageHeight = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);
      double reportedFPS = videoCapture.get(opencv_videoio.CAP_PROP_FPS);

      AVDictionary ioOptions = new AVDictionary();
      error = av_dict_set(ioOptions, "listen", "2", 0);
      FFMPEGTools.checkNegativeError(error, "Setting ioOption listen");

      AVIOContext serverContext = new AVIOContext();
      error = avio_open2(serverContext, "http://127.0.0.1:8080", AVIO_FLAG_WRITE, null, ioOptions);
      FFMPEGTools.checkNegativeError(error, "Opening IO");
      FFMPEGTools.checkPointer(serverContext, "Opening IO");

      for (int i = 0; true; i++)
      {
         System.out.println("Waiting for client...");
         AVIOContext clientContext = new AVIOContext();
         error = avio_accept(serverContext, clientContext);
         FFMPEGTools.checkNegativeError(error, "Accepting client");
         FFMPEGTools.checkPointer(clientContext, "Accepting client");
         System.out.println("Got a client!");

         ThreadTools.startAThread(new ClientHandler(clientContext, imageWidth, imageHeight, reportedFPS), "ClientThread" + i);
      }
   }

   public void captureImage()
   {
      newImageLock.lock();
      try
      {
         videoCapture.read(image);
         sequenceNumber++;
         newImageAvailable.signalAll();
      }
      finally
      {
         newImageLock.unlock();
      }
   }

   private class ClientHandler implements Runnable
   {
      private final AVFormatContext outputContext;
      private final FFMPEGVideoEncoder videoEncoder;
      private long lastSequenceNumber = 0L;
      private int error;

      private ClientHandler(AVIOContext clientContext, int imageWidth, int imageHeight, double outputFrameRate)
      {
         outputContext = new AVFormatContext();
         error = avformat_alloc_output_context2(outputContext, null, "h264", null);
         FFMPEGTools.checkError(error, outputContext, "Allocating output context");
         outputContext.pb(clientContext);

         videoEncoder = new FFMPEGVideoEncoder(outputContext,
                                               "h264_nvenc",
                                               400000,
                                               imageWidth,
                                               imageHeight,
                                               AV_PIX_FMT_YUV420P,
                                               outputFrameRate,
                                               10,
                                               2,
                                               AV_PIX_FMT_BGR24);
      }

      @Override
      public void run()
      {
         AVDictionary streamFlags = new AVDictionary();
         avutil.av_dict_set(streamFlags, "lossless", "0", 0);
         error = avformat_write_header(outputContext, streamFlags);

         boolean notDone = true;
         while (notDone)
         {
            newImageLock.lock();
            try
            {
               while (lastSequenceNumber >= sequenceNumber)
                  newImageAvailable.await();

               lastSequenceNumber = sequenceNumber;
               videoEncoder.setNextFrame(image);
               notDone = videoEncoder.encodeAndWriteNextFrame();
            }
            catch (InterruptedException e)
            {
               throw new RuntimeException(e);
            }
            finally
            {
               newImageLock.unlock();
            }
         }

         av_write_trailer(outputContext);

         av_write_trailer(outputContext);

         videoEncoder.destroy();
         avio_closep(outputContext.pb());

         avformat_free_context(outputContext);

         av_dict_free(streamFlags);
         streamFlags.close();
      }
   }

   public static void main(String[] args)
   {
      new FFMPEGWebcamStreamingDemo();
   }
}
