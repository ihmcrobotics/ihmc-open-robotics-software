package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.global.opencv_videoio;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import org.bytedeco.opencv.opencv_videoio.VideoWriter;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;

import javax.annotation.Nullable;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

class ValveIndexCameraReaderThread extends Thread
{
   private final ConcurrentLinkedQueue<Mat> leftRects = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<Mat> rightRects = new ConcurrentLinkedQueue<>();
   private int width;
   private int height;
   private volatile boolean running = true;

   @Override
   public void run()
   {
      VideoCapture videoCapture = new VideoCapture(0);

      width = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_WIDTH);
      height = (int) videoCapture.get(opencv_videoio.CAP_PROP_FRAME_HEIGHT);

      videoCapture.set(opencv_videoio.CAP_PROP_FOURCC, VideoWriter.fourcc((byte) 'M', (byte) 'J', (byte) 'P', (byte) 'G'));
      videoCapture.set(opencv_videoio.CAP_PROP_FPS, 60.0);

      LogTools.info("Index camera resolution " + width + "x" + height);
      LogTools.info("fps: {}", videoCapture.get(opencv_videoio.CAP_PROP_FPS));
      LogTools.info("Backend: {}", videoCapture.getBackendName().getString());
      LogTools.info("Format: {}", videoCapture.get(opencv_videoio.CAP_PROP_FORMAT));
      LogTools.info("Is open: {}", videoCapture.isOpened());

      videoCapture.set(opencv_videoio.CAP_PROP_BUFFERSIZE, 2);

      while (running)
      {
         boolean readFrame = false;

         Mat bgrImage = new Mat(height, width, opencv_core.CV_8UC3);

         if (videoCapture.read(bgrImage))
         {
            readFrame = true;
         }
         else
         {
            LogTools.error("Failed to read frame");
         }

         if (readFrame)
         {
            // Left
            Mat bgrLeftCameraRect = bgrImage.apply(new Rect(0, 0, width / 2, height));
            Mat rgbLeftCameraRect = new Mat();
            opencv_imgproc.cvtColor(bgrLeftCameraRect, rgbLeftCameraRect, opencv_imgproc.COLOR_BGR2RGBA);
            bgrLeftCameraRect.close();
            leftRects.add(rgbLeftCameraRect);

            // Right
            Mat bgrRightCameraRect = bgrImage.apply(new Rect(width / 2, 0, width - width / 2, height));
            Mat rgbRightCameraRect = new Mat();
            opencv_imgproc.cvtColor(bgrRightCameraRect, rgbRightCameraRect, opencv_imgproc.COLOR_BGR2RGBA);
            bgrRightCameraRect.close();
            rightRects.add(rgbRightCameraRect);
         }

         bgrImage.close();
      }

      LogTools.info("Closing video capture device...");
      videoCapture.close();
      LogTools.info("Closed video capture device");
   }

   @Nullable
   public Queue<Mat> getRects(RobotSide side)
   {
      if (side == RobotSide.LEFT)
         return leftRects;
      else
         return rightRects;
   }

   public int getWidth()
   {
      return width;
   }

   public int getHeight()
   {
      return height;
   }

   public void stopRunning()
   {
      running = false;
   }
}
