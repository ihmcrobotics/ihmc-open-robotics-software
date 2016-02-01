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
import us.ihmc.imageProcessing.utilities.stereo.StereoImageViewer;
import us.ihmc.imageProcessing.utilities.stereo.StereoVideoListener;

/**
 * User: Matt
 * Date: 3/4/13
 */
public class StereoVideoPlayer
{
   private MP4VideoDemuxer leftEyeDemuxer;
   private MP4VideoDemuxer rightEyeDemuxer;
   private String leftEyeFilename;
   private String rightEyeFilename;

   private StereoVideoListener videoListener;
   private boolean LOOP_CONTINUOUSLY = false;
   private final YUVPictureConverter converter = new YUVPictureConverter();

   public StereoVideoPlayer(String leftEyeFilename, String rightEyeFilename, StereoVideoListener videoListener, boolean loopContinuously)
   {
      this.leftEyeFilename = leftEyeFilename;
      this.rightEyeFilename = rightEyeFilename;

      this.videoListener = videoListener;
      this.LOOP_CONTINUOUSLY = loopContinuously;
   }

   public void start()
   {
      System.out.println("starting video player");

      do
      {
         System.out.println("opeing left eye video");
         openleftEyeVideoFile();
         System.out.println("opeing right eye video");
         openRightEyeVideoFile();
         streamBufferedImagesFromVideo();
      }
      while (LOOP_CONTINUOUSLY);
   }

   private void openleftEyeVideoFile()
   {
      try
      {
         leftEyeDemuxer = new MP4VideoDemuxer(new File(leftEyeFilename));
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void openRightEyeVideoFile()
   {
      try
      {
         rightEyeDemuxer = new MP4VideoDemuxer(new File(rightEyeFilename));
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void streamBufferedImagesFromVideo()
   {
      // Now, we start walking through the container looking at each packet.

      YUVPicture leftEye;
      YUVPicture rightEye;
      try
      {
         while ((leftEye = leftEyeDemuxer.getNextFrame()) != null && (rightEye = rightEyeDemuxer.getNextFrame()) != null)
         {
            videoListener.updateImage(converter.toBufferedImage(leftEye), converter.toBufferedImage(rightEye));
            leftEye.delete();
            rightEye.delete();
            //TODO: Insert eait loop
         }
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

   }

   public void close()
   {
      leftEyeDemuxer.delete();
      rightEyeDemuxer.delete();
   }

   public static void main(String[] args)
   {
      StereoImageViewer imageViewer = new StereoImageViewer();
      final StereoVideoPlayer videoPlayer = new StereoVideoPlayer("./media/videos/run1.mov", "./media/videos/run1Video2.mov", imageViewer, true);

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
