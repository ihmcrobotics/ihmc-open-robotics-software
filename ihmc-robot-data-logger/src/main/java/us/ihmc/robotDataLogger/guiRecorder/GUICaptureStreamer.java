package us.ihmc.robotDataLogger.guiRecorder;

import java.awt.Dimension;
import java.awt.Rectangle;
import java.awt.Toolkit;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

import javax.swing.JFrame;

import us.ihmc.codecs.generated.RGBPicture;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.generated.YUVPicture.YUVSubsamplingType;
import us.ihmc.codecs.screenCapture.ScreenCapture;
import us.ihmc.codecs.screenCapture.ScreenCaptureFactory;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.pubsub.Domain;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.pubsub.attributes.ParticipantAttributes;
import us.ihmc.pubsub.attributes.PublisherAttributes;
import us.ihmc.pubsub.attributes.ReliabilityKind;
import us.ihmc.pubsub.participant.Participant;
import us.ihmc.pubsub.publisher.Publisher;
import us.ihmc.pubsub.types.ByteBufferPubSubType;
import us.ihmc.robotDataLogger.rtps.LogParticipantSettings;

public class GUICaptureStreamer
{
   public static final String topicType = "us::ihmc::robotDataLogger::gui::screenshot";
   public static final String partition = LogParticipantSettings.partition + LogParticipantSettings.namespaceSeperator + "GuiStreamer";
   public static final int MAXIMUM_IMAGE_DATA_SIZE= 1024*1024;
   private final Supplier<Rectangle> windowBoundsProvider;
   private final int fps;

   private final ScreenCapture screenCapture = ScreenCaptureFactory.getScreenCapture();

   private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor(ThreadTools.getNamedThreadFactory("GUICaptureStreamer"));
   private final CaptureRunner captureRunner = new CaptureRunner();
   private final Dimension size = new Dimension();

   private Domain domain = DomainFactory.getDomain(PubSubImplementation.FAST_RTPS);
   private Participant participant;
   private Publisher publisher;
   private final String topicName;
   
   private JPEGEncoder encoder = new JPEGEncoder();
   
   private ScheduledFuture<?> future = null;

   public GUICaptureStreamer(JFrame window, int fps, float quality, int domainID, String topicName)
   {
	   this(() -> window.getBounds(), fps, quality, domainID, topicName);
   }

   public GUICaptureStreamer(Supplier<Rectangle> windowBoundsProvider, int fps, float quality, int domainID, String topicName)
   {
      this.windowBoundsProvider = windowBoundsProvider;
      this.fps = fps;
      this.topicName = topicName;
      try
      {
         ParticipantAttributes attributes = domain.createParticipantAttributes(domainID, getClass().getSimpleName());
         participant = domain.createParticipant(attributes);
                  
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   public synchronized void start()
   {
      if(future != null)
      {
         future.cancel(false);
      }
      ByteBufferPubSubType pubSubType = new ByteBufferPubSubType(topicType, MAXIMUM_IMAGE_DATA_SIZE);
      PublisherAttributes attributes = domain.createPublisherAttributes(participant, pubSubType, topicName, ReliabilityKind.BEST_EFFORT, partition);
      try
      {
         publisher = domain.createPublisher(participant, attributes);
      }
      catch (IllegalArgumentException | IOException e)
      {
         throw new RuntimeException(e);
      }
      scheduler.scheduleAtFixedRate(captureRunner, 10, 1000000000 / fps, TimeUnit.NANOSECONDS);
   }

   public synchronized void stop()
   {
      if(future != null)
      {
         future.cancel(false);
      }
      domain.removeParticipant(participant);
   }

   private class CaptureRunner implements Runnable
   {
      @Override
      public void run()
      {
         Rectangle windowBounds = windowBoundsProvider.get();
         Rectangle screen = new Rectangle(Toolkit.getDefaultToolkit().getScreenSize());
         Rectangle captureRectangle = windowBounds.intersection(screen);
         Dimension windowSize = captureRectangle.getSize();
         
         if (!windowSize.equals(size))
         {
            size.setSize(windowSize);
         }

         try
         {
            RGBPicture img = screenCapture.createScreenCapture(captureRectangle);

            if (img != null)
            {
               YUVPicture yuv = img.toYUV(YUVSubsamplingType.YUV420);
               ByteBuffer buffer = encoder.encode(yuv, 90);
               if(buffer.remaining() <= MAXIMUM_IMAGE_DATA_SIZE)
               {
                  publisher.write(buffer);
               }
               else
               {
                  System.err.println("Not sending screen capture, image size exceeds " + MAXIMUM_IMAGE_DATA_SIZE);
               }
               yuv.delete();
               img.delete();
            }
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

   }

   public void destroy()
   {
      domain.removeParticipant(participant);
      domain = null;
      participant = null;
      scheduler.shutdownNow();
   }
}
