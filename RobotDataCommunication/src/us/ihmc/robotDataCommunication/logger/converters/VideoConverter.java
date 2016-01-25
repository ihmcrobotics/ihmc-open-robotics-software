package us.ihmc.robotDataCommunication.logger.converters;

import java.io.File;
import java.io.IOException;

import org.jcodec.containers.mp4.MP4Packet;

import us.ihmc.codecs.builder.MP4H264MovieBuilder;
import us.ihmc.codecs.builder.MP4MJPEGMovieBuilder;
import us.ihmc.codecs.demuxer.MP4VideoDemuxer;
import us.ihmc.codecs.generated.EUsageType;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.robotDataCommunication.logger.util.CustomProgressMonitor;

public class VideoConverter
{
   
   /**
    * 
    * @param source
    * @param target
    * @param startPTS
    * @param endPTS
    * @param monitor
    * @return frame rate of the new video file
    * 
    * @throws IOException
    */
   public static int crop(File source, File target, long startPTS, long endPTS, CustomProgressMonitor monitor) throws IOException
   {
      MP4MJPEGMovieBuilder builder = null;
      MP4VideoDemuxer demuxer = new MP4VideoDemuxer(source);
      
      int frameRate = getFrameRate(demuxer);
      
      
      long endFrame = getFrame(endPTS, demuxer);
      long startFrame = getFrame(startPTS, demuxer);
      long numberOfFrames = endFrame - startFrame;
      
      MP4Packet frame;
      while((frame = demuxer.getNextPacket()) != null && demuxer.getCurrentFrame() <= endFrame)
      {
         if(builder == null)
         {
            builder = new MP4MJPEGMovieBuilder(target, demuxer.getWidth(), demuxer.getHeight(), frameRate, 1);
         }
        // frame.toYUV420();
         builder.encodeFrame(frame.getData());
         
         if(monitor != null)
         {
            double progress = ((double)(demuxer.getCurrentFrame() - startFrame) / (double) numberOfFrames) * 100.0;
            monitor.setProgress((int) progress);
         }
      }
      
      builder.close();
      demuxer.delete();
      
      return frameRate;
      
   }
   /**
    * 
    * @param source
    * @param target
    * @param startPTS
    * @param endPTS
    * @param bitrate
    * @param monitor
    * @return frame rate of the new video file
    * 
    * @throws IOException
    */
   public static int convert(File source, File target, long startPTS, long endPTS, int bitrate, CustomProgressMonitor monitor) throws IOException
   {
      MP4H264MovieBuilder builder = null;
      MP4VideoDemuxer demuxer = new MP4VideoDemuxer(source);
      
      int frameRate = getFrameRate(demuxer);
      
      
      long endFrame = getFrame(endPTS, demuxer);
      long startFrame = getFrame(startPTS, demuxer);
      long numberOfFrames = endFrame - startFrame;
      
      YUVPicture frame;
      while((frame = demuxer.getNextFrame()) != null && demuxer.getCurrentFrame() <= endFrame)
      {
         if(builder == null)
         {
            builder = new MP4H264MovieBuilder(target, frame.getWidth(), frame.getHeight(), frameRate, bitrate, EUsageType.CAMERA_VIDEO_REAL_TIME);
         }
        // frame.toYUV420();
         builder.encodeFrame(frame);
         frame.delete();
         
         if(monitor != null)
         {
            double progress = ((double)(demuxer.getCurrentFrame() - startFrame) / (double) numberOfFrames) * 100.0;
            monitor.setProgress((int) progress);
         }
      }
      
      builder.close();
      demuxer.delete();
      
      return frameRate;
      
   }

   private static long getFrame(long endPTS, MP4VideoDemuxer demuxer) throws IOException
   {
      demuxer.seekToPTS(endPTS);
      long endFrame = demuxer.getCurrentFrame();
      return endFrame;
   }

   private static int getFrameRate(MP4VideoDemuxer demuxer) throws IOException
   {
      demuxer.seek(0.0);
      long start = demuxer.getCurrentFrame();
      demuxer.seek(1.0);
      long end = demuxer.getCurrentFrame();
      
      int frameRate = (int) (end - start);
      System.out.println("Video framerate is " + frameRate);
      return frameRate;
   }
}
