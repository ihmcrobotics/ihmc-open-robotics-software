package us.ihmc.rdx.ui;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.files.FileHandle;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.PixmapIO;
import com.badlogic.gdx.utils.ScreenUtils;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_videoio.VideoWriter;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.nio.ByteBuffer;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.Executors;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKey;

public class RDXScreenRecorder
{
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   private VideoWriter videoWriter = null;

   String fileLocation = null;

   private int counter = 1;

   private boolean recording = false;

   public void record(boolean enabled)
   {
      if (enabled)
      {
         if (!recording)
         {
            SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
            String logFileName = dateFormat.format(new Date()) + "_" + "UIRecording.avi";
            FileTools.ensureDirectoryExists(Paths.get(IHMCCommonPaths.RECORDINGS_DIRECTORY.toString()), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

            fileLocation = IHMCCommonPaths.RECORDINGS_DIRECTORY.resolve(logFileName).toString();

            int fourcc = VideoWriter.fourcc((byte) 'M', (byte) 'J', (byte) 'P', (byte) 'G');
            int fps = 30;

            videoWriter = new VideoWriter(fileLocation, fourcc, fps, new Size(Gdx.graphics.getWidth(), Gdx.graphics.getHeight()), true);

            LogTools.warn("Opening MP4 for UI Recording: {} - [{}]", fileLocation, videoWriter.isOpened());

            if (videoWriter.isOpened())
            {
               recording = true;
            }
            else {
               // Print an error message and check for any additional error details
               LogTools.error("Failed to open VideoWriter.");
            }
         }
         else if (recording)
         {
            if (videoWriter != null && videoWriter.isOpened())
            {
               Pixmap pixmap = getScreenshot(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
               ByteBuffer buffer = ByteBuffer.allocateDirect(pixmap.getPixels().limit());
               BytePointer bytePointer = new BytePointer(buffer);
               buffer.put(pixmap.getPixels()).flip();
               Mat mat = new Mat(pixmap.getHeight(), pixmap.getWidth(), org.bytedeco.opencv.global.opencv_core.CV_8UC4, bytePointer);

               videoWriter.write(mat); // Write the frame to the video

               pixmap.dispose();
            }
         }
      }

      if (recording && !enabled)
      {
         recording = false;
         if (videoWriter != null && videoWriter.isOpened())
         {
            LogTools.warn("Closing MP4 for UI Recording: " + fileLocation);
            videoWriter.release(); // Release the video writer
         }
      }
   }

   public void saveScreenshot(String path)
   {
      try
      {
         FileHandle fh;
         do
         {
            fh = new FileHandle(path + "/screenshot" + counter++ + ".png");
         }
         while (fh.exists());
         Pixmap pixmap = getScreenshot(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
         PixmapIO.writePNG(fh, pixmap);
         pixmap.dispose();
      }
      catch (Exception e)
      {
      }
   }

   public void showScreenCaptureImage()
   {
      Pixmap pixmap = getScreenshot(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
      ByteBuffer buffer = ByteBuffer.allocateDirect(pixmap.getPixels().limit());
      BytePointer bytePointer = new BytePointer(buffer);
      buffer.put(pixmap.getPixels()).flip();
      Mat mat = new Mat(pixmap.getHeight(), pixmap.getWidth(), org.bytedeco.opencv.global.opencv_core.CV_8UC4, bytePointer);

      imshow("Screen Capture", mat);
      waitKey(1);

      pixmap.dispose();
   }

   private Pixmap getScreenshot(int x, int y, int w, int h)
   {
      final Pixmap pixmap = ScreenUtils.getFrameBufferPixmap(x, y, w, h);
      return pixmap;
   }
}
