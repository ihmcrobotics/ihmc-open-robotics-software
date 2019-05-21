package us.ihmc.avatar.video;

import controller_msgs.msg.dds.VideoPacket;
import javafx.scene.image.ImageView;
import javafx.scene.image.PixelWriter;
import javafx.scene.image.WritableImage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.javaFXVisualizers.PrivateAnimationTimer;
import us.ihmc.log.LogTools;
import us.ihmc.ros2.Ros2Node;

import java.awt.image.BufferedImage;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class JavaFXROS2VideoView extends ImageView
{
   private final Ros2Node ros2Node;
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);
   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
   private final WritableImage writableImage;
   private final int width;
   private final int height;
   private final boolean flipX;
   private final boolean flipY;

   private volatile boolean running = false;

   public JavaFXROS2VideoView(Ros2Node ros2Node, int width, int height, boolean flipX, boolean flipY)
   {
      this.ros2Node = ros2Node;
      this.width = width;
      this.height = height;
      this.flipX = flipX;
      this.flipY = flipY;

      writableImage = new WritableImage(width, height);
   }

   public void start()
   {
      running = true;

      new ROS2Callback<>(ros2Node, VideoPacket.class, null, null, null, this::acceptVideo);

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

            synchronized (this)
            {
               PixelWriter pixelWriter = writableImage.getPixelWriter();
               for (int x = 0; x < bufferedImage.getWidth(); x++)
               {
                  for (int y = 0; y < bufferedImage.getHeight(); y++)
                  {
                     pixelWriter.setArgb(flipX ? bufferedImage.getWidth()  - 1 - x : x,
                                         flipY ? bufferedImage.getHeight() - 1 - y : y,
                                         bufferedImage.getRGB(x, y));
                  }
               }
            }
         });
      }
   }

   private void handle(long now)
   {
      synchronized (this)
      {
         // set image in scene
         setImage(writableImage);
      }
   }
}
