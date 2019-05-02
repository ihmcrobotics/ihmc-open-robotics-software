package us.ihmc.robotEnvironmentAwareness.fusion;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.ImageMessage;
import javafx.animation.AnimationTimer;
import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;

public class FusionSensorImageViewer
{
   private final VBox imageViewPane = new VBox();
   private final ImageView streamingView = new ImageView();

   private final AtomicReference<ImageMessage> newImageMessageToView;
   private final AtomicReference<BufferedImage> recentBufferedImage;
   private final AtomicReference<BufferedImage> newBufferedImageToView;

   private final AtomicReference<Boolean> enableStreaming;
   private final AtomicReference<Boolean> snapshot;
   private final AtomicReference<Boolean> clearImages;

   private final List<BufferedImage> imagesToView = new ArrayList<>();

   private final AnimationTimer imageStreamer;

   public FusionSensorImageViewer(SharedMemoryJavaFXMessager messager, Pane imagePane)
   {
      streamingView.setFitWidth(LidarImageFusionProcessorUI.imageStreamingImageFixedWidth);
      streamingView.setPreserveRatio(true);
      imagePane.getChildren().add(streamingView);
      imagePane.getChildren().add(imageViewPane);

      enableStreaming = messager.createInput(LidarImageFusionAPI.EnableStreaming, false);
      snapshot = messager.createInput(LidarImageFusionAPI.ImageSnapShot, false);
      clearImages = messager.createInput(LidarImageFusionAPI.ImageViewClear, false);

      newImageMessageToView = messager.createInput(LidarImageFusionAPI.ImageState);
      recentBufferedImage = new AtomicReference<BufferedImage>(null);
      newBufferedImageToView = messager.createInput(LidarImageFusionAPI.ImageResultState);
      
      imageStreamer = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            update();
         }
      };
   }

   private void unpackImage(ImageMessage imageMessage)
   {
      if (imageMessage == null)
         return;

      BufferedImage bufferedImage = convertImageMessageToBufferedImage(imageMessage);
      recentBufferedImage.set(bufferedImage);
      Image streamingImage = SwingFXUtils.toFXImage(bufferedImage, null);
      streamingView.setImage(streamingImage);
   }

   public void update()
   {
      if (!enableStreaming.get())
         return;

      if (clearImages.getAndSet(false))
         clearImageView();

      if (newImageMessageToView.get() == null)
         return;
      
      if(newBufferedImageToView.get() != null)
         imagesToView.add(newBufferedImageToView.getAndSet(null));

      unpackImage(newImageMessageToView.getAndSet(null));

      if (snapshot.getAndSet(false))
      {
         if (recentBufferedImage.get() != null)
         {
            imagesToView.add(recentBufferedImage.getAndSet(null));
         }
      }

      imageViewPane.getChildren().clear();
      for (BufferedImage bufferedImage : imagesToView)
      {
         Image image = SwingFXUtils.toFXImage(bufferedImage, null);
         ImageView imageView = new ImageView();
         imageView.setImage(image);
         imageView.setFitWidth(LidarImageFusionProcessorUI.imageStreamingImageFixedWidth);
         imageView.setPreserveRatio(true);
         imageViewPane.getChildren().add(imageView);
      }
   }

   public void start()
   {
      imageStreamer.start();
   }

   public void clearImageView()
   {
      imagesToView.clear();
   }

   //TODO : create a helper class and move to there.
   public static BufferedImage convertImageMessageToBufferedImage(ImageMessage imageMessage)
   {
      int width = imageMessage.getWidth();
      int height = imageMessage.getHeight();
      BufferedImage bufferedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
      int pixelIndex = 0;
      for (int i = 0; i < height; i++)
      {
         for (int j = 0; j < width; j++)
         {
            bufferedImage.setRGB(j, i, imageMessage.getRgbdata().get(pixelIndex));
            pixelIndex++;
         }
      }
      return bufferedImage;
   }
}