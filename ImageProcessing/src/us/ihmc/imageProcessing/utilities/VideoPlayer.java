package us.ihmc.imageProcessing.utilities;

import com.xuggle.xuggler.*;
import us.ihmc.utilities.camera.ImageViewer;
import us.ihmc.utilities.camera.VideoListener;

import javax.swing.*;
import java.awt.*;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;

/**
 * User: Matt
 * Date: 3/4/13
 */
public class VideoPlayer
{
   private IContainer container;
   private int videoStreamId = -1;
   private IStreamCoder videoCoder;
   private IVideoResampler resampler;
   private String filename;
   private VideoListener videoListener;
   private boolean LOOP_CONTINUOUSLY = false;

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
      } while (LOOP_CONTINUOUSLY);
   }

   private void openVideoFile(String filename)
   {
      // Let's make sure that we can actually convert video pixel formats.
      if (!IVideoResampler.isSupported(IVideoResampler.Feature.FEATURE_COLORSPACECONVERSION))
      {
         throw new RuntimeException("you must install the GPL version of Xuggler (with IVideoResampler support) for this demo to work");
      }

      // Create a Xuggler container object
      container = IContainer.make();

      // Open up the container
      if (container.open(filename, IContainer.Type.READ, null) < 0)
      {
         throw new IllegalArgumentException("could not open file: " + filename);
      }

      // query how many streams the call to open found
      int numStreams = container.getNumStreams();

      // and iterate through the streams to find the first video stream
      for (int i = 0; i < numStreams; i++)
      {
         // Find the stream object
         IStream stream = container.getStream(i);
         // Get the pre-configured decoder that can decode this stream;
         IStreamCoder coder = stream.getStreamCoder();

         if (coder.getCodecType() == ICodec.Type.CODEC_TYPE_VIDEO)
         {
            videoStreamId = i;
            videoCoder = coder;
            break;
         }
      }
      if (videoStreamId == -1)
      {
         throw new RuntimeException("could not find video stream in container: " + filename);
      }

      // Now we have found the video stream in this file.  Let's open up our decoder so it can do work.
      if (videoCoder.open() < 0)
      {
         throw new RuntimeException("could not open video decoder for container: " + filename);
      }

      if (videoCoder.getPixelType() != IPixelFormat.Type.BGR24)
      {
         // if this stream is not in BGR24, we're going to need to convert it.  The VideoResampler does that for us.
         resampler = IVideoResampler.make(videoCoder.getWidth(), videoCoder.getHeight(), IPixelFormat.Type.BGR24, videoCoder.getWidth(), videoCoder.getHeight(), videoCoder.getPixelType());
         if (resampler == null)
         {
            throw new RuntimeException("could not create color space resampler for: " + filename);
         }
      }
   }

   private void streamBufferedImagesFromVideo()
   {
      // Now, we start walking through the container looking at each packet.
      IPacket packet = IPacket.make();
      long firstTimestampInStream = Global.NO_PTS;
      long systemClockStartTime = 0;
      while (container != null && container.readNextPacket(packet) >= 0)
      {
         // Now we have a packet, let's see if it belongs to our video stream
         if (packet.getStreamIndex() == videoStreamId)
         {
            // We allocate a new picture to get the data out of Xuggler
            IVideoPicture picture = IVideoPicture.make(videoCoder.getPixelType(), videoCoder.getWidth(), videoCoder.getHeight());

            int offset = 0;
            while (offset < packet.getSize())
            {
               // Now, we decode the video, checking for any errors.
               int bytesDecoded = videoCoder.decodeVideo(picture, packet, offset);
               if (bytesDecoded < 0)
               {
                  throw new RuntimeException("got error decoding video in: " + filename);
               }
               offset += bytesDecoded;

               /*
                * Some decoders will consume data in a packet, but will not be able to construct
                * a full video picture yet.  Therefore you should always check if you
                * got a complete picture from the decoder
               */
               if (picture.isComplete())
               {
                  IVideoPicture newPic = picture;
                  /*
                  * If the resampler is not null, that means we didn't get the
                  * video in BGR24 format and
                  * need to convert it into BGR24 format.
                  */
                  if (resampler != null)
                  {
                     // we must resample
                     newPic = IVideoPicture.make(resampler.getOutputPixelFormat(), picture.getWidth(), picture.getHeight());
                     if (resampler.resample(newPic, picture) < 0)
                     {
                        throw new RuntimeException("could not resample video from: " + filename);
                     }
                  }
                  if (newPic.getPixelType() != IPixelFormat.Type.BGR24)
                  {
                     throw new RuntimeException("could not decode video as BGR 24 bit data in: " + filename);
                  }

                  /**
                   * We could just display the images as quickly as we decode them,
                   * but it turns out we can decode a lot faster than you think.
                   *
                   * So instead, the following code does a poor-man's version of
                   * trying to match up the frame-rate requested for each
                   * IVideoPicture with the system clock time on your computer.
                   *
                   * Remember that all Xuggler IAudioSamples and IVideoPicture objects
                   * always give timestamps in Microseconds, relative to the first
                   * decoded item. If instead you used the packet timestamps, they can
                   * be in different units depending on your IContainer, and IStream
                   * and things can get hairy quickly.
                   */
                  if (firstTimestampInStream == Global.NO_PTS)
                  {
                     // This is our first time through
                     firstTimestampInStream = picture.getTimeStamp();
                     // get the starting clock time so we can hold up frames until the right time.
                     systemClockStartTime = System.currentTimeMillis();
                  }
                  else
                  {
                     long systemClockCurrentTime = System.currentTimeMillis();
                     long millisecondsClockTimeSinceStartofVideo = systemClockCurrentTime - systemClockStartTime;
                     // compute how long for this frame since the first frame in the stream.
                     // remember that IVideoPicture and IAudioSamples timestamps are always in MICROSECONDS,
                     // so we divide by 1000 to get milliseconds.
                     long millisecondsStreamTimeSinceStartOfVideo = (picture.getTimeStamp() - firstTimestampInStream) / 1000;
                     final long millisecondsTolerance = 50; // and we give ourselves 50 ms of tolerance
                     final long millisecondsToSleep = (millisecondsStreamTimeSinceStartOfVideo - (millisecondsClockTimeSinceStartofVideo + millisecondsTolerance));
                     if (millisecondsToSleep > 0)
                     {
                        try
                        {
                           Thread.sleep(millisecondsToSleep);
                        }
                        catch (InterruptedException e)
                        {
                           // we might get this when the user closes the dialog box, so just return from the method.
                           return;
                        }
                     }
                  }

                  // And finally, convert the BGR24 to an Java buffered image
                  BufferedImage javaImage = Utils.videoPictureToImage(newPic);

                  videoListener.updateImage(javaImage);
               }
            }
         }
         else
         {
            // This packet isn't part of our video stream, so we just silently drop it.
         }
      }
   }

   public void close()
   {
      /*
     * Technically since we're exiting anyway, these will be cleaned up by
     * the garbage collector... but because we're nice people and want
     * to be invited places for Christmas, we're going to show how to clean up.
     */
      if (videoCoder != null)
      {
         videoCoder.close();
         videoCoder = null;
      }
      if (container != null)
      {
         container.close();
         container = null;
      }
   }

   public static void main(String[] args)
   {
      ImageViewer imageViewer = new ImageViewer();
      final VideoPlayer videoPlayer = new VideoPlayer("./media/videos/LaneDetectionVideo.mp4", imageViewer, true);

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