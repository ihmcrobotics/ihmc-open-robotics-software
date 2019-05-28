package us.ihmc.quadrupedUI.video;

import java.awt.image.BufferedImage;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import controller_msgs.msg.dds.VideoPacket;
import javafx.scene.image.ImageView;
import javafx.scene.image.PixelWriter;
import javafx.scene.image.WritableImage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.javaFXVisualizers.PrivateAnimationTimer;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.ros2.Ros2Node;

public class JavaFXROS2VideoView extends ImageView
{
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);
   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
   private final ConcurrentRingBuffer<WritableImage> writableImageBuffer;
   private final boolean flipX;
   private final boolean flipY;

   private volatile boolean running = false;

   public JavaFXROS2VideoView(int width, int height, boolean flipX, boolean flipY)
   {
      this.flipX = flipX;
      this.flipY = flipY;

      writableImageBuffer = new ConcurrentRingBuffer<>(() -> new WritableImage(width, height), 4);
   }

   public void start(Ros2Node ros2Node)
   {
      if (running)
      {
         LogTools.error("Video view is already running.");
         return;
      }

      running = true;
      new ROS2Callback<>(ros2Node, VideoPacket.class, null, null, null, this::acceptVideo);
      animationTimer.start();
   }

   public void start(Messager messager, Topic<VideoPacket> videoTopic)
   {
      if (running)
      {
         LogTools.error("Video view is already running.");
         return;
      }

      running = true;
      messager.registerTopicListener(videoTopic, this::acceptVideo);
      animationTimer.start();
   }

   public void stop()
   {
      running = false;
      animationTimer.stop();
      executorService.shutdownNow();
   }

   private void acceptVideo(VideoPacket message)
   {
      if (running && VideoSource.fromByte(message.getVideoSource()) == VideoSource.MULTISENSE_LEFT_EYE)
      {
         executorService.submit(() ->
         {
            // decompress and pack writableimage
            BufferedImage bufferedImage = jpegDecompressor.decompressJPEGDataToBufferedImage(message.getData().toArray());
            LogTools.trace("res x: {}, y: {}", bufferedImage.getWidth(), bufferedImage.getHeight());

            WritableImage nextImage = writableImageBuffer.next();

            if (nextImage != null)
            {
               PixelWriter pixelWriter = nextImage.getPixelWriter();
               for (int x = 0; x < bufferedImage.getWidth(); x++)
               {
                  for (int y = 0; y < bufferedImage.getHeight(); y++)
                  {
                     pixelWriter.setArgb(flipX ? bufferedImage.getWidth()  - 1 - x : x,
                                         flipY ? bufferedImage.getHeight() - 1 - y : y,
                                         bufferedImage.getRGB(x, y));
                  }
               }

               writableImageBuffer.commit();
            }
         });
      }
   }

   private void handle(long now)
   {
      if (writableImageBuffer.poll())
      {
         WritableImage latestImage = null;
         WritableImage image = null;
         while ((image = writableImageBuffer.read()) != null)
         {
            latestImage = image;
         }

         // set image in scene
         if (latestImage != null)
            setImage(latestImage);

         writableImageBuffer.flush();
      }
   }
}
