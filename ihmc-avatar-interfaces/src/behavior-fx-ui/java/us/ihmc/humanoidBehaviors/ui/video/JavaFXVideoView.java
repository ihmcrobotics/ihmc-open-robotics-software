package us.ihmc.humanoidBehaviors.ui.video;

import javafx.scene.image.ImageView;
import javafx.scene.image.PixelWriter;
import javafx.scene.image.WritableImage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.idl.IDLSequence;
import us.ihmc.javaFXVisualizers.PrivateAnimationTimer;
import us.ihmc.log.LogTools;

import java.awt.image.BufferedImage;
import java.util.concurrent.ExecutorService;
import java.util.function.Supplier;

public class JavaFXVideoView extends ImageView
{
   private final ExecutorService executorService = ThreadTools.newSingleThreadExecutor(getClass().getSimpleName());
   private final PrivateAnimationTimer animationTimer = new PrivateAnimationTimer(this::handle);
   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
   private final ConcurrentRingBuffer<WritableImage> writableImageBuffer;
   private final boolean flipX;
   private final boolean flipY;

   private volatile boolean running = false;

   public JavaFXVideoView(int width, int height, boolean flipX, boolean flipY)
   {
      this.flipX = flipX;
      this.flipY = flipY;

      writableImageBuffer = new ConcurrentRingBuffer<>(() -> new WritableImage(width, height), 4);
   }

   public void start()
   {
      running = true;
      animationTimer.start();
   }

   public void stop()
   {
      running = false;
      animationTimer.stop();
   }

   public void destroy()
   {
      stop();
      executorService.shutdownNow();
   }

   protected void acceptVideo(BufferedImage bufferedImage)
   {
      submitTask(() -> bufferedImage);
   }

   protected void acceptVideo(IDLSequence.Byte compressedImageData)
   {
      submitTask(() -> jpegDecompressor.decompressJPEGDataToBufferedImage(compressedImageData.toArray()));
   }

   private void submitTask(Supplier<BufferedImage> bufferedImageSupplier)
   {
      if (running)
      {
         executorService.submit(() ->
         {
            try
            {
               commitImage(bufferedImageSupplier.get());
            }
            catch (Throwable t)
            {
               LogTools.error("Exception in thread: {}: {}", Thread.currentThread().getName(), t.getMessage());
               t.printStackTrace();
               throw t;
            }
         });
      }
   }

   private void commitImage(BufferedImage bufferedImage)
   {
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
