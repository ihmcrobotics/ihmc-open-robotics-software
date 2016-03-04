package us.ihmc.imageProcessing.utilities;

import java.awt.BorderLayout;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.io.IOException;

import javax.swing.JFrame;

import us.ihmc.codecs.demuxer.MP4VideoDemuxer;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.YUVPictureConverter;

/**
 * User: Matt
 * Date: 3/4/13
 */
public class VideoPlayer
{
   private String filename;
   private VideoListener videoListener;
   private boolean LOOP_CONTINUOUSLY = false;
   private MP4VideoDemuxer demuxer;
   private final YUVPictureConverter converter = new YUVPictureConverter();
   
   public VideoPlayer(String filename, VideoListener videoListener, boolean loopContinuously)
   {
      this.filename = filename;
      this.videoListener = videoListener;
      this.LOOP_CONTINUOUSLY = loopContinuously;
   }

   public void start()
   {
      do
      {
         openVideoFile(filename);
         streamBufferedImagesFromVideo();
      }
      while (LOOP_CONTINUOUSLY);
   }

   private void openVideoFile(String filename)
   {
      try
      {
         demuxer = new MP4VideoDemuxer(new File(filename));
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void streamBufferedImagesFromVideo()
   {
      YUVPicture frame;
      try
      {
         while ((frame = demuxer.getNextFrame()) != null)
         {
            videoListener.updateImage(converter.toBufferedImage(frame));
            frame.delete();
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }

   }

   public void close()
   {
      demuxer.delete();
   }

   public static void main(String[] args)
   {
      ImageViewer imageViewer = new ImageViewer();
      final VideoPlayer videoPlayer = new VideoPlayer("./media/videos/run1.mov", imageViewer, true);

      JFrame jFrame = new JFrame("Video Player Test");
      jFrame.addWindowListener(new WindowAdapter()
      {
         public void windowClosing(WindowEvent e)
         {
            videoPlayer.close();
            System.exit(0);
         }
      });

      jFrame.getContentPane().add(imageViewer, BorderLayout.CENTER);
      jFrame.pack();
      jFrame.setVisible(true);

      videoPlayer.start();
   }
}