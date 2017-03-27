package us.ihmc.simulationConstructionSetTools.bambooTools.videoWall;

import java.awt.image.BufferedImage;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import javax.swing.ImageIcon;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

import us.ihmc.codecs.demuxer.MP4VideoDemuxer;
import us.ihmc.codecs.generated.FilterModeEnum;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.commons.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class VideoPlayer
{
   private static final double FRAME_RATE = 15.0; //TODO: This should be 30.0 according to the file information. Something is inconsistent...
   private static final boolean LOOP_FOREVER = true;
   private static int videoPlayerThreadIndex = 0;
   private static int demuxerThreadIndex = 0;
   
   private final JLabel jLabel;
   private final JPanel jPanel;
   private YUVPictureConverter converter = new YUVPictureConverter();
   private BufferedImage bufferedImage;
   private boolean readyForUpdate;
   private Path currentPathToVideo;
   private MP4VideoDemuxer demuxer;
   private ImageIcon imageIcon;
   private boolean timedOut;

   public VideoPlayer()
   {
      jLabel = new JLabel();
      jPanel = new JPanel();
      
      jPanel.add(jLabel);
      readyForUpdate = true;
      
      startPlaying();
   }
   
   public void startPlaying()
   {
      PrintTools.info(garbageCollectAndPrintUsedMemoryInMB(false) + "startPlaying() ");
      
      ExecutorService executor = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName() + videoPlayerThreadIndex++));
      executor.execute(new Runnable()
      {
         @Override
         public void run()
         {
            while (true)
            {
               if (currentPathToVideo == null)
               {
                  ThreadTools.sleep(10);
               }
               else
               {
                  boolean demuxerCreatedSuccessfully = tryToMakeDemuxer(currentPathToVideo);
                  if (demuxerCreatedSuccessfully)
                  {
                     playVideo(demuxer);
                  }
                  else
                  {
                     currentPathToVideo = null;
                  }
                  
                  if (demuxer != null)
                  {
                     demuxer.delete();
                  }
               }
               
               readyForUpdate = true;
            }
         }
      });
      executor.shutdown();
   }
   
   private boolean tryToMakeDemuxer(final Path path)
   {
      PrintTools.debug(this, garbageCollectAndPrintUsedMemoryInMB(false) + " tryToMakeDemuxer " + path.getFileName().toString());
      
      timedOut = true;
      ThreadTools.executeWithTimeout(MP4VideoDemuxer.class.getSimpleName() + demuxerThreadIndex++, new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               demuxer = new MP4VideoDemuxer(path.toFile());
               timedOut = false;
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         }
      }, 1000, TimeUnit.MILLISECONDS);
      if (timedOut)
         PrintTools.error("Demuxer.init() timed out!");
      return timedOut;
   }

   public void playVideo(MP4VideoDemuxer demuxer)
   {
      try
      {
         long sleepTimeMillisPerFrame = (long) (1000.0 / FRAME_RATE);
         long previousTimeMillis = System.currentTimeMillis();

         demuxer.seekToFrame(0);
         YUVPicture picture;
         while (true)
         {
            picture = demuxer.getNextFrame();
            
            if (picture == null)
            {
               if (LOOP_FOREVER)
               {
                  demuxer.seekToFrame(0);
                  return;
               }
               else
               {
                  break;
               }
            }
            
            long currentTimeMillis = System.currentTimeMillis();

            doTheNextFrame(picture);

            long timeDelta = currentTimeMillis - previousTimeMillis;
            long timeToSleep = sleepTimeMillisPerFrame - timeDelta;
            if (timeToSleep < 1)
               timeToSleep = 1;
            if (timeToSleep > sleepTimeMillisPerFrame)
               timeToSleep = sleepTimeMillisPerFrame;

            ThreadTools.sleep(timeToSleep);
            previousTimeMillis = currentTimeMillis;
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void doTheNextFrame(YUVPicture picture)
   {
      int width = jPanel.getWidth();
      int height = jPanel.getHeight();
      
      if (width < 1 || height < 1)
         return;
      
      picture.scale(width, height, FilterModeEnum.kFilterBilinear);
      
      bufferedImage = converter.toBufferedImage(picture, bufferedImage);
      
      SwingUtilities.invokeLater(new Runnable()
      {
         @Override
         public void run()
         {
            imageIcon = new ImageIcon(bufferedImage);
            jLabel.setIcon(imageIcon);
         }
      });
      picture.delete();
   }
   
   public void updateVideoPath(Path pathToVideo)
   {
      readyForUpdate = false;
      currentPathToVideo = pathToVideo;
   }
   
   public boolean readyToUpdateVideoPath()
   {
      return readyForUpdate;
   }

   public JPanel getjPanel()
   {
      return jPanel;
   }
   
   private String garbageCollectAndPrintUsedMemoryInMB(boolean doGarbageCollect)
   {
      Runtime runtime = Runtime.getRuntime();

      if (doGarbageCollect)
      {
         System.gc();
         try
         {
            Thread.sleep(100);
         }
         catch (InterruptedException e)
         {
            e.printStackTrace();
         }
         System.gc();
      }

      long freeMemory = runtime.freeMemory();
      long totalMemory = runtime.totalMemory();
      long usedMemory = totalMemory - freeMemory;

      int usedMemoryMB = (int) (usedMemory / 1000000);

      return "USED MEMORY: " + usedMemoryMB + " MB: ";
   }
   
   static int i = 0;
   
   public static void main(String[] args)
   {
      final ArrayList<MP4VideoDemuxer> demuxers = new ArrayList<>();

      for (i = 0; i < 1000; i++)
      {
         ThreadTools.executeWithTimeout("demuxer" + i, new Runnable()
         {
            @Override
            public void run()
            {
               MP4VideoDemuxer demuxer;
               try
               {
                  System.out.println("i = " + i);
                  demuxer = new MP4VideoDemuxer(Paths.get("20150920_0009_Atlas_DRCPushRecoveryMultiStepTest.testMultiStepForwardAndContinueWalking.mp4").toFile());
                  
                  for (int j = 0; j < 5; j++)
                  {
                  demuxer.getNextFrame();
                  }
                  demuxer.delete();
                  demuxers.add(demuxer);
               }
               catch (IOException e)
               {
                  e.printStackTrace();
               }
            }
         }, 500, TimeUnit.MILLISECONDS);
      }
      
      System.gc();

      ThreadTools.sleepSeconds(10.0);
   }
}