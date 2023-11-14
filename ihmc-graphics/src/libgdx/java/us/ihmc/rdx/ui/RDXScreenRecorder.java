package us.ihmc.rdx.ui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.utils.ScreenUtils;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_videoio.VideoWriter;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.IHMCCommonPaths;

import java.nio.ByteBuffer;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.atomic.AtomicReference;

public class RDXScreenRecorder
{
   private VideoWriter videoWriter;
   private String fileLocation;
   private final AtomicReference<Pixmap> lastFrame = new AtomicReference<>();
   private ScreenFrameWriteThread screenFrameWriteThread;

   private volatile boolean recording;

   public boolean isRecording()
   {
      return recording;
   }

   public void setRecording(boolean recording)
   {
      this.recording = recording;
   }

   public void writeFrame()
   {
      if (recording && videoWriter == null)
      {
         SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
         String logFileName = dateFormat.format(new Date()) + "_" + "UIRecording.avi";
         FileTools.ensureDirectoryExists(Paths.get(IHMCCommonPaths.RECORDINGS_DIRECTORY.toString()), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

         fileLocation = IHMCCommonPaths.RECORDINGS_DIRECTORY.resolve(logFileName).toString();

         int fourcc = VideoWriter.fourcc((byte) 'M', (byte) 'J', (byte) 'P', (byte) 'G');
         int fps = 30;

         videoWriter = new VideoWriter(fileLocation, fourcc, fps, new Size(Gdx.graphics.getWidth(), Gdx.graphics.getHeight()), true);

         LogTools.warn("Opening AVI for UI Recording: {} - [{}]", fileLocation, videoWriter.isOpened());

         if (videoWriter.isOpened())
         {
            screenFrameWriteThread = new ScreenFrameWriteThread();
            screenFrameWriteThread.start();

            RDXBaseUI.getInstance().getPrimary3DPanel().getNotificationManager().pushNotification("Started recording to: " + fileLocation);
         }
      }
      else if (recording && videoWriter.isOpened())
      {
         Pixmap pixmap = getFrame(Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
         lastFrame.set(pixmap);

         synchronized (lastFrame)
         {
            lastFrame.notify();
         }
      }
      else
      {
         // Stop the recording
         if (videoWriter != null)
         {
            LogTools.warn("Closing AVI for UI Recording: " + fileLocation);
            try
            {
               screenFrameWriteThread.join();
            }
            catch (InterruptedException e)
            {
               LogTools.error(e);
            }
            screenFrameWriteThread = null;

            lastFrame.set(null);

            videoWriter.release(); // Release the video writer
            videoWriter = null;

            RDXBaseUI.getInstance().getPrimary3DPanel().getNotificationManager().pushNotification("Stopped recording");
         }
      }
   }

   private class ScreenFrameWriteThread extends Thread
   {
      @Override
      public void run()
      {
         while (recording)
         {
            synchronized (lastFrame)
            {
               try
               {
                  lastFrame.wait();
               }
               catch (InterruptedException e)
               {
                  LogTools.error(e);
               }

               // Check again after waiting
               if (!recording)
                  break;

               Pixmap pixmap = lastFrame.get();

               ByteBuffer buffer = ByteBuffer.allocateDirect(pixmap.getPixels().limit());
               BytePointer bytePointer = new BytePointer(buffer);
               buffer.put(pixmap.getPixels());

               Mat mat = new Mat(pixmap.getHeight(), pixmap.getWidth(), org.bytedeco.opencv.global.opencv_core.CV_8UC4, bytePointer);
               videoWriter.write(mat);
               mat.close();

               pixmap.dispose();
            }
         }
      }
   }

   private static Pixmap getFrame(int w, int h)
   {
      final Pixmap pixmap = ScreenUtils.getFrameBufferPixmap(0, 0, w, h);
      return pixmap;
   }
}
