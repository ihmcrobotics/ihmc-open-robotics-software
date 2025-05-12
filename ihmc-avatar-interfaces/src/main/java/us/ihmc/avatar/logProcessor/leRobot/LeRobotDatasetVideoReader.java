package us.ihmc.avatar.logProcessor.leRobot;

import org.bytedeco.javacpp.Pointer;
import org.bytedeco.javacv.FFmpegFrameGrabber;
import org.bytedeco.javacv.Frame;
import org.bytedeco.javacv.OpenCVFrameConverter;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.io.File;
import java.nio.file.Path;

public class LeRobotDatasetVideoReader
{
   private final RobotSide side;
   private final FFmpegFrameGrabber grabber;
   private final OpenCVFrameConverter.ToMat frameConverter = new OpenCVFrameConverter.ToMat();
   private Frame currentFrame;
   private Mat currentMat;
   
   private long lastTimestamp = -1;

   public LeRobotDatasetVideoReader(RobotSide side, Path mp4Path)
   {
      this.side = side;
      
      // Create the grabber for the mp4 file
      grabber = new FFmpegFrameGrabber(mp4Path.toFile());
      
      // Set properties that correspond to those used in the writer
      grabber.setFormat("mp4");
      grabber.setFrameRate(LeRobotDataset.ZED_FPS);
      
      // Start the grabber
      ExceptionTools.handle(() -> grabber.start(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
   }
   
   /**
    * Reads the frame at the specified timestamp
    * @param videoTimestampMicros timestamp in microseconds
    * @return Mat containing the frame at the specified timestamp
    */
   public Mat readFrame(long videoTimestampMicros)
   {
      if (videoTimestampMicros != lastTimestamp)
      {
         // Only seek if we're moving to a different timestamp
         ExceptionTools.handle(() -> grabber.setTimestamp(videoTimestampMicros), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
         
         // Grab the frame at this timestamp
         try
         {
            currentFrame = grabber.grabImage();
            
            if (currentFrame != null)
            {
               // Convert the frame to a Mat
               if (currentMat != null)
               {
                  currentMat.close();
               }
               
               currentMat = frameConverter.convert(currentFrame);
               
               // Convert RGBA to BGRA (reverse of what writer does)
               opencv_imgproc.cvtColor(currentMat, currentMat, opencv_imgproc.COLOR_RGBA2BGRA);
               
               lastTimestamp = videoTimestampMicros;
            }
         }
         catch (Exception e)
         {
            throw new RuntimeException("Error grabbing frame at timestamp " + videoTimestampMicros, e);
         }
      }
      
      return currentMat;
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
      
      ExceptionTools.handle(() -> grabber.stop(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      ExceptionTools.handle(() -> grabber.release(), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);
      frameConverter.close();
   }
}
