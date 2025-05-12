package us.ihmc.avatar.logProcessor.leRobot;

import org.bytedeco.javacv.FFmpegFrameGrabber;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.OpenCVFrameConverter;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;

import java.nio.file.Path;

public class LeRobotDatasetVideoReader
{
   private final FFmpegFrameGrabber grabber;
   private final OpenCVFrameConverter.ToMat frameConverter = new OpenCVFrameConverter.ToMat();
   private Frame currentFrame;
   private Mat currentMat;
   
   private long currentTimestamp = -1;
   private boolean hasMoreFrames = true;

   public LeRobotDatasetVideoReader(Path mp4Path)
   {
      // Create the grabber for the mp4 file
      grabber = new FFmpegFrameGrabber(mp4Path.toFile());
      
      // Start the grabber
      ExceptionTools.handle(() -> grabber.start(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }
   
   /**
    * Reads the next frame from the video
    * @return Mat containing the next frame, or null if no more frames are available
    */
   public Mat readFrame()
   {
      try
      {
         // Grab the next frame
         currentFrame = grabber.grabImage();
         
         if (currentFrame != null)
         {
            // Convert the frame to a Mat
            if (currentMat != null)
            {
               currentMat.close();
            }

            // Update the timestamp
            currentTimestamp = grabber.getTimestamp();
            
            return currentMat;
         }
         else
         {
            hasMoreFrames = false;
            return null;
         }
      }
      catch (Exception e)
      {
         throw new RuntimeException("Error grabbing next frame", e);
      }
   }
   
   /**
    * Gets the timestamp of the current frame in microseconds
    * @return current timestamp in microseconds
    */
   public long getCurrentTimestamp()
   {
      return currentTimestamp;
   }
   
   /**
    * Checks if there are more frames available in the video
    * @return true if more frames are available, false otherwise
    */
   public boolean hasMoreFrames()
   {
      return hasMoreFrames;
   }
   
   /**
    * Gets the width of the video frames
    * @return video width in pixels
    */
   public int getImageWidth()
   {
      return grabber.getImageWidth();
   }
   
   /**
    * Gets the height of the video frames
    * @return video height in pixels
    */
   public int getImageHeight()
   {
      return grabber.getImageHeight();
   }
   
   /**
    * Gets the framerate of the video
    * @return framerate in frames per second
    */
   public double getFrameRate()
   {
      return grabber.getFrameRate();
   }
   
   /**
    * Gets the total number of frames in the video
    * @return total frames
    */
   public int getLengthInFrames()
   {
      return grabber.getLengthInFrames();
   }
   
   /**
    * Gets the total duration of the video in microseconds
    * @return duration in microseconds
    */
   public long getLengthInTime()
   {
      return grabber.getLengthInTime();
   }
   
   /**
    * Closes and releases all resources
    */
   public void close()
   {
      if (currentMat != null)
      {
         currentMat.close();
         currentMat = null;
      }
      
      if (currentFrame != null)
      {
         currentFrame.close();
         currentFrame = null;
      }
      
      ExceptionTools.handle(grabber::stop, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      ExceptionTools.handle(grabber::release, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      frameConverter.close();
   }
}
