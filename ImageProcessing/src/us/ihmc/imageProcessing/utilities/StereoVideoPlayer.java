package us.ihmc.imageProcessing.utilities;

import com.xuggle.xuggler.*;

import us.ihmc.utilities.camera.ImageViewer;
import us.ihmc.utilities.camera.StereoImageViewer;
import us.ihmc.utilities.camera.StereoVideoListener;
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
public class StereoVideoPlayer
{
   private IContainer leftEyeContainer;
   private IContainer rightEyeContainer;

   private int leftEyeVideoStreamId = -1;
   private int rightEyeVideoStreamId = -1;

   private IStreamCoder leftEyeVideoCoder;
   private IStreamCoder rightEyeVideoCoder;

   private IVideoResampler leftEyeResampler;
   private IVideoResampler rightEyeResampler;

   private String leftEyeFilename;
   private String rightEyeFilename;

   private StereoVideoListener videoListener;
   private boolean LOOP_CONTINUOUSLY = false;

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
         System.out.println("left eye video open. ID: " + leftEyeVideoStreamId);
         System.out.println("opeing right eye video");
         openRightEyeVideoFile();
         System.out.println("right eye video open. ID: " + rightEyeVideoStreamId);
         streamBufferedImagesFromVideo();
      }
      while (LOOP_CONTINUOUSLY);
   }

   private void openleftEyeVideoFile()
   {
      // Let's make sure that we can actually convert video pixel formats.
      if (!IVideoResampler.isSupported(IVideoResampler.Feature.FEATURE_COLORSPACECONVERSION))
      {
         throw new RuntimeException("you must install the GPL version of Xuggler (with IVideoResampler support) for this demo to work");
      }

      // Create a Xuggler container object
      leftEyeContainer = IContainer.make();

      // Open up the container
      if (leftEyeContainer.open(leftEyeFilename, IContainer.Type.READ, null) < 0)
      {
         throw new IllegalArgumentException("could not open file: " + leftEyeFilename);
      }

      // query how many streams the call to open found
      int numStreams = leftEyeContainer.getNumStreams();

      // and iterate through the streams to find the first video stream
      for (int i = 0; i < numStreams; i++)
      {
         // Find the stream object
         IStream stream = leftEyeContainer.getStream(i);

         // Get the pre-configured decoder that can decode this stream;
         IStreamCoder coder = stream.getStreamCoder();

         if (coder.getCodecType() == ICodec.Type.CODEC_TYPE_VIDEO)
         {
            leftEyeVideoStreamId = i;
            leftEyeVideoCoder = coder;

            break;
         }
      }

      if (leftEyeVideoStreamId == -1)
      {
         throw new RuntimeException("could not find video stream in container: " + leftEyeFilename);
      }

      // Now we have found the video stream in this file.  Let's open up our decoder so it can do work.
      if (leftEyeVideoCoder.open() < 0)
      {
         throw new RuntimeException("could not open video decoder for container: " + leftEyeFilename);
      }

      if (leftEyeVideoCoder.getPixelType() != IPixelFormat.Type.BGR24)
      {
         // if this stream is not in BGR24, we're going to need to convert it.  The VideoResampler does that for us.
         leftEyeResampler = IVideoResampler.make(leftEyeVideoCoder.getWidth(), leftEyeVideoCoder.getHeight(), IPixelFormat.Type.BGR24,
                 leftEyeVideoCoder.getWidth(), leftEyeVideoCoder.getHeight(), leftEyeVideoCoder.getPixelType());

         if (leftEyeResampler == null)
         {
            throw new RuntimeException("could not create color space resampler for: " + leftEyeFilename);
         }
      }
   }

   private void openRightEyeVideoFile()
   {
      // Let's make sure that we can actually convert video pixel formats.
      if (!IVideoResampler.isSupported(IVideoResampler.Feature.FEATURE_COLORSPACECONVERSION))
      {
         throw new RuntimeException("you must install the GPL version of Xuggler (with IVideoResampler support) for this demo to work");
      }

      // Create a Xuggler container object
      rightEyeContainer = IContainer.make();

      // Open up the container
      if (rightEyeContainer.open(rightEyeFilename, IContainer.Type.READ, null) < 0)
      {
         throw new IllegalArgumentException("could not open file: " + rightEyeFilename);
      }

      // query how many streams the call to open found
      int numStreams = rightEyeContainer.getNumStreams();

      // and iterate through the streams to find the first video stream
      for (int i = 0; i < numStreams; i++)
      {
         // Find the stream object
         IStream stream = rightEyeContainer.getStream(i);

         // Get the pre-configured decoder that can decode this stream;
         IStreamCoder coder = stream.getStreamCoder();

         if (coder.getCodecType() == ICodec.Type.CODEC_TYPE_VIDEO)
         {
            rightEyeVideoStreamId = i;
            rightEyeVideoCoder = coder;

            break;
         }
      }

      if (rightEyeVideoStreamId == -1)
      {
         throw new RuntimeException("could not find video stream in container: " + rightEyeFilename);
      }

      // Now we have found the video stream in this file.  Let's open up our decoder so it can do work.
      if (rightEyeVideoCoder.open() < 0)
      {
         throw new RuntimeException("could not open video decoder for container: " + rightEyeFilename);
      }

      if (rightEyeVideoCoder.getPixelType() != IPixelFormat.Type.BGR24)
      {
         // if this stream is not in BGR24, we're going to need to convert it.  The VideoResampler does that for us.
         rightEyeResampler = IVideoResampler.make(rightEyeVideoCoder.getWidth(), rightEyeVideoCoder.getHeight(), IPixelFormat.Type.BGR24,
                 rightEyeVideoCoder.getWidth(), rightEyeVideoCoder.getHeight(), rightEyeVideoCoder.getPixelType());

         if (rightEyeResampler == null)
         {
            throw new RuntimeException("could not create color space resampler for: " + rightEyeFilename);
         }
      }
   }

   private void streamBufferedImagesFromVideo()
   {
      // Now, we start walking through the container looking at each packet.
      IPacket leftEyePacket = IPacket.make();
      IPacket rightEyePacket = IPacket.make();


      long firstTimestampInStream = Global.NO_PTS;
      long systemClockStartTime = 0;


      while ((leftEyeContainer != null) && (leftEyeContainer.readNextPacket(leftEyePacket) >= 0) && (rightEyeContainer != null)
             && (rightEyeContainer.readNextPacket(rightEyePacket) >= 0))
      {
         IVideoPicture leftEyePicture = getNextFrame(leftEyePacket, leftEyeVideoStreamId, leftEyeVideoCoder, leftEyeResampler);
         IVideoPicture rightEyePicture = getNextFrame(rightEyePacket, rightEyeVideoStreamId, rightEyeVideoCoder, rightEyeResampler);
         if ((leftEyePicture != null) && (rightEyePicture != null))
         {
            BufferedImage leftEye = Utils.videoPictureToImage(leftEyePicture);
            BufferedImage rightEye = Utils.videoPictureToImage(rightEyePicture);

          if (firstTimestampInStream == Global.NO_PTS)
          {
             // This is our first time through
             firstTimestampInStream = leftEyePicture.getTimeStamp();

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
             long millisecondsStreamTimeSinceStartOfVideo = (leftEyePicture.getTimeStamp() - firstTimestampInStream) / 1000;
             final long millisecondsTolerance = 50;    // and we give ourselves 50 ms of tolerance
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


          

            videoListener.updateImage(leftEye, rightEye);
         }
      }

   }

   private IVideoPicture getNextFrame(IPacket packed, int streamId, IStreamCoder streamCoder, IVideoResampler videoResampler)
   {
      IVideoPicture newPic = null;

      // Now we have a packet, let's see if it belongs to our video stream
      if (packed.getStreamIndex() == streamId)
      {
         // We allocate a new picture to get the data out of Xuggler
         IVideoPicture picture = IVideoPicture.make(streamCoder.getPixelType(), streamCoder.getWidth(), streamCoder.getHeight());

         int offset = 0;
         while (offset < packed.getSize())
         {
            // Now, we decode the video, checking for any errors.
            int bytesDecoded = streamCoder.decodeVideo(picture, packed, offset);
            if (bytesDecoded < 0)
            {
               throw new RuntimeException("got error decoding video");
            }

            offset += bytesDecoded;

            /*
             * Some decoders will consume data in a packet, but will not be able to construct
             * a full video picture yet.  Therefore you should always check if you
             * got a complete picture from the decoder
             */
            if (picture.isComplete())
            {
               newPic = picture;

               /*
                * If the resampler is not null, that means we didn't get the
                * video in BGR24 format and
                * need to convert it into BGR24 format.
                */
               if (videoResampler != null)
               {
                  // we must resample
                  newPic = IVideoPicture.make(videoResampler.getOutputPixelFormat(), picture.getWidth(), picture.getHeight());

                  if (videoResampler.resample(newPic, picture) < 0)
                  {
                     throw new RuntimeException("could not resample video");
                  }
               }

               if (newPic.getPixelType() != IPixelFormat.Type.BGR24)
               {
                  throw new RuntimeException("could not decode video as BGR 24 bit data");
               }


               // And finally, convert the BGR24 to an Java buffered image

            }
         }
      }
      else
      {
         // This packet isn't part of our video stream, so we just silently drop it.
      }

      return newPic;
   }

   public void close()
   {
      /*
       * Technically since we're exiting anyway, these will be cleaned up by
       * the garbage collector... but because we're nice people and want
       * to be invited places for Christmas, we're going to show how to clean up.
       */
      if (leftEyeVideoCoder != null)
      {
         leftEyeVideoCoder.close();
         leftEyeVideoCoder = null;
      }

      if (leftEyeContainer != null)
      {
         leftEyeContainer.close();
         leftEyeContainer = null;
      }
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
