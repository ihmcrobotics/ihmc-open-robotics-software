package us.ihmc.robotDataCommunication.logger;

import gnu.trove.list.array.TLongArrayList;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.HashMap;

import javax.swing.JFrame;

import us.ihmc.robotDataCommunication.logger.util.FFMpeg;
import us.ihmc.robotDataCommunication.logger.util.PipedCommandExecutor;

import com.xuggle.mediatool.IMediaReader;
import com.xuggle.mediatool.IMediaViewer;
import com.xuggle.mediatool.IMediaViewer.Mode;
import com.xuggle.mediatool.ToolFactory;
import com.xuggle.xuggler.ICodec.Type;
import com.xuggle.xuggler.IError;
import com.xuggle.xuggler.IRational;
import com.xuggle.xuggler.IStream;

public class VideoDataPlayer
{
   private final String name;
   private final boolean hasTimebase;
   private final boolean interlaced;
   
   private long[] robotTimestamps;
   private long[] videoTimestamps;
   
   private long bmdTimeBaseNum;
   private long bmdTimeBaseDen;
   private long ffTimeBaseNum;
   private long ffTimeBaseDen;
   
   
   private final int videoStream;
   private final IMediaReader reader;
   private final HideableMediaFrame viewer;
   
   
   private int currentlyShowingIndex = 0;
   private long currentlyShowingRobottimestamp = 0;
   private long upcomingRobottimestamp = 0;
   
   private final File videoFile;
   
   public VideoDataPlayer(String name, File dataDirectory, LogProperties logProperties)
   {
      this.name = name;
      this.interlaced = logProperties.getInterlaced(name);
      this.hasTimebase = logProperties.hasTimebase();
      
      if(!hasTimebase)
      {
         System.err.println("Video data is using timestamps instead of frame numbers. Falling back to seeking based on timestamp.");
      }
      videoFile = new File(dataDirectory, logProperties.getVideoFile(name));
      File timestampFile = new File(dataDirectory, logProperties.getTimestampFile(name));

      parseTimestampData(timestampFile);
      
      reader = ToolFactory.makeReader(videoFile.getAbsolutePath());
      viewer = new HideableMediaFrame();
      reader.addListener(viewer.getViewer());
      reader.open();
      int videoStream = -1;
      for(int i = 0; i < reader.getContainer().getNumStreams(); i++)
      {
         IStream stream = reader.getContainer().getStream(i);
         if(stream.getStreamCoder().getCodecType() == Type.CODEC_TYPE_VIDEO)
         {
            videoStream = i;
            break;
         }
      }
      if(videoStream == -1)
      {
         throw new RuntimeException("Cannot find video stream in " + videoFile.getAbsolutePath());
      }
      this.videoStream = videoStream;
      
      
      IRational timeBase = reader.getContainer().getStream(videoStream).getTimeBase();
      ffTimeBaseNum = timeBase.getNumerator();
      ffTimeBaseDen = timeBase.getDenominator();
      
      
      
      
      
   }
   

   public synchronized void showVideoFrame(long timestamp)
   {
      if(timestamp >= currentlyShowingRobottimestamp && timestamp < upcomingRobottimestamp)
      {
         return;
      }
      
      long videoTimestamp = getVideoTimestamp(timestamp);
      
      if(currentlyShowingIndex + 1 < robotTimestamps.length)
      {
         upcomingRobottimestamp = robotTimestamps[currentlyShowingIndex + 1];
      }
      else
      {
         upcomingRobottimestamp = currentlyShowingRobottimestamp;
      }
      
      int retval = reader.getContainer().seekKeyFrame(videoStream, videoTimestamp, 0);
      
      if(retval != 0)
      {
         System.err.println(IError.make(retval).getDescription());
      }
      reader.readPacket();
   }

   public void setVisible(boolean visible)
   {
      viewer.setVisible(visible);
   }
   
   private long getVideoTimestamp(long timestamp)
   {
      currentlyShowingIndex = Arrays.binarySearch(robotTimestamps, timestamp);
      
      if(currentlyShowingIndex < 0)
      {
         int nextIndex = -currentlyShowingIndex + 1;
         if ((nextIndex < robotTimestamps.length) && (Math.abs(robotTimestamps[-currentlyShowingIndex] - timestamp) > Math.abs(robotTimestamps[nextIndex])))
         {
            currentlyShowingIndex = nextIndex;
         }
         else
         {
            currentlyShowingIndex = - currentlyShowingIndex;
         }
      }
      
      if (currentlyShowingIndex < 0) currentlyShowingIndex = 0;
      if (currentlyShowingIndex >= robotTimestamps.length) currentlyShowingIndex = robotTimestamps.length - 1;
      currentlyShowingRobottimestamp = robotTimestamps[currentlyShowingIndex];
      
      
      long videoTimestamp = videoTimestamps[currentlyShowingIndex];
      
      if(hasTimebase)
      {
         videoTimestamp = (videoTimestamp * bmdTimeBaseNum * ffTimeBaseDen) / (bmdTimeBaseDen * ffTimeBaseNum);
      }
      
      return videoTimestamp;
   }
   
   private void parseTimestampData(File timestampFile)
   {
      BufferedReader reader = null;
      try
      {
         reader = new BufferedReader(new FileReader(timestampFile));
         
                  
         String line;
         
         if((line = reader.readLine()) != null)
         {
            bmdTimeBaseNum = Long.valueOf(line);
         }
         else
         {
            throw new RuntimeException("Cannot read numerator");
         }
         
         if((line = reader.readLine()) != null)
         {
            bmdTimeBaseDen = Long.valueOf(line);
         }
         else
         {
            throw new RuntimeException("Cannot read denumerator");
         }

         TLongArrayList robotTimestamps = new TLongArrayList();
         TLongArrayList videoTimestamps = new TLongArrayList();
         while ((line = reader.readLine()) != null)
         {
            String[] stamps = line.split("\\s");
            long robotStamp = Long.valueOf(stamps[0]);
            long videoStamp = Long.valueOf(stamps[1]);
            
            if(interlaced)
            {
               videoStamp /= 2;
            }

            robotTimestamps.add(robotStamp);
            videoTimestamps.add(videoStamp);

         }
         
         this.robotTimestamps = robotTimestamps.toArray();
         this.videoTimestamps = videoTimestamps.toArray();

      }
      catch (FileNotFoundException e)
      {
         throw new RuntimeException(e);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
      finally
      {
         try
         {
            if(reader != null)
            {
               reader.close();
            }
         }
         catch (IOException e)
         {
         }
      }
   }
   
   public void exportVideo(File selectedFile, long startTimestamp, long endTimestamp, PrintStream output)
   {
      
      long startVideoTimestamp = getVideoTimestamp(startTimestamp);
      long endVideoTimestamp = getVideoTimestamp(endTimestamp);
      
      double timebase = reader.getContainer().getStream(videoStream).getTimeBase().getDouble();
      
      double startTime = startVideoTimestamp * timebase;
      double endTime = endVideoTimestamp * timebase;
      
      FFMpeg ffMpeg  = new FFMpeg();
      
      ffMpeg.setStarttime(startTime);
      ffMpeg.setEndtime(endTime);
      ffMpeg.setInputFile("\""+videoFile.getAbsolutePath()+"\"");
      ffMpeg.setVideoCodec("h264");
      ffMpeg.setAudioCodec("aac");
      ffMpeg.enableExperimentalCodecs(true);
      ffMpeg.setOutputFile("\""+selectedFile.getAbsolutePath()+"\"");
      
      PipedCommandExecutor executor = new PipedCommandExecutor(ffMpeg);
      try
      {
         executor.execute(output, output);
         executor.waitFor();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      
   }

   
   public void cropVideo(File outputFile, File timestampFile, long startTimestamp, long endTimestamp, PrintStream output) throws IOException
   {
      
      PrintWriter timestampWriter = new PrintWriter(timestampFile);
      timestampWriter.println(bmdTimeBaseNum);
      timestampWriter.println(bmdTimeBaseDen);
      
      long videoOffset = Long.MIN_VALUE;
      
      for(int i = 0; i < robotTimestamps.length; i++)
      {
         long robotTimestamp = robotTimestamps[i];
         long videoTimestamp = videoTimestamps[i];
         
         if(robotTimestamp >= startTimestamp && robotTimestamp <= endTimestamp)
         {
            if(videoOffset == Long.MIN_VALUE)
            {
               videoOffset = videoTimestamp;
            }
            
            timestampWriter.print(robotTimestamp);
            timestampWriter.print(" ");
            timestampWriter.println(videoTimestamp - videoOffset);
         }
         else if(robotTimestamp > endTimestamp)
         {
            break;
         }
      }
      
      timestampWriter.close();
      
      
      
      long startVideoTimestamp = getVideoTimestamp(startTimestamp);
      long endVideoTimestamp = getVideoTimestamp(endTimestamp);
      double timebase = reader.getContainer().getStream(videoStream).getTimeBase().getDouble();
      
      double startTime = startVideoTimestamp * timebase;
      double endTime = endVideoTimestamp * timebase;
      
      FFMpeg ffMpeg  = new FFMpeg();
      
      ffMpeg.setStarttime(startTime);
      ffMpeg.setEndtime(endTime);
      ffMpeg.setInputFile("'"+videoFile.getAbsolutePath()+"'");
      ffMpeg.setVideoCodec("copy");
      ffMpeg.setAudioCodec("copy");
      ffMpeg.enableExperimentalCodecs(true);
      ffMpeg.setOutputFile("'"+outputFile.getAbsolutePath()+"'");
      
      PipedCommandExecutor executor = new PipedCommandExecutor(ffMpeg);
      try
      {
         executor.execute(output, output);
         executor.waitFor();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
   
   private class HideableMediaFrame 
   {
      private final IMediaViewer mediaViewer;
      private final HashMap<Integer, JFrame> mFrames;
      
      @SuppressWarnings("unchecked")
      public HideableMediaFrame()
      {
         mediaViewer = ToolFactory.makeViewer(Mode.FAST_VIDEO_ONLY, false);
         Field frames;
         try
         {
            frames = mediaViewer.getClass().getDeclaredField("mFrames");
            frames.setAccessible(true);
            mFrames = (HashMap<Integer, JFrame>) frames.get(mediaViewer);
         }
         catch (NoSuchFieldException | SecurityException | IllegalArgumentException | IllegalAccessException e)
         {
            throw new RuntimeException(e);
         }
         
      }
      
      public IMediaViewer getViewer()
      {
         return mediaViewer;
      }
      
      public void setVisible(boolean visible)
      {
         for(JFrame frame : mFrames.values())
         {
            frame.setVisible(visible);
         }
      }
      
   }

   public String getName()
   {
      return name;
   }
}
