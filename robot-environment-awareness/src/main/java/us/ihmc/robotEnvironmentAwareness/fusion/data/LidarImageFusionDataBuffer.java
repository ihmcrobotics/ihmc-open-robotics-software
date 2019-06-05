package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.awt.image.BufferedImage;
import java.util.concurrent.atomic.AtomicReference;

import boofcv.struct.calib.IntrinsicParameters;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.ImageSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.LidarImageFusionDataFactory;

public class LidarImageFusionDataBuffer
{
   private final Messager messager;
   private final IntrinsicParameters intrinsicParameters;

   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionPointCloudMessage = new AtomicReference<>(null);
   private final AtomicReference<BufferedImage> latestBufferedImage = new AtomicReference<>(null);

   private final AtomicReference<ImageSegmentationParameters> latestImageSegmentationParaeters;
   private final AtomicReference<LidarImageFusionData> newBuffer = new AtomicReference<>(null);

   public LidarImageFusionDataBuffer(Messager messager, IntrinsicParameters intrinsic)
   {
      this.messager = messager;
      intrinsicParameters = intrinsic;
      latestImageSegmentationParaeters = messager.createInput(LidarImageFusionAPI.ImageSegmentationParameters, new ImageSegmentationParameters());
   }

   public LidarImageFusionData pollNewBuffer()
   {
      return newBuffer.getAndSet(null);
   }

   public void updateNewBuffer()
   {
      long updateStartTime = System.nanoTime();

      StereoVisionPointCloudMessage stereoVisionPointCloudMessage = latestStereoVisionPointCloudMessage.get();
      ImageSegmentationParameters imageSegmentationParameters = latestImageSegmentationParaeters.get();

      LidarImageFusionData data = LidarImageFusionDataFactory.createLidarImageFusionData(MessageTools.unpackScanPoint3ds(stereoVisionPointCloudMessage),
                                                                                         latestBufferedImage.get(), intrinsicParameters,
                                                                                         imageSegmentationParameters.getPixelSize(),
                                                                                         imageSegmentationParameters.getPixelRuler(),
                                                                                         imageSegmentationParameters.getMinElementSize(),
                                                                                         imageSegmentationParameters.getIterate());
      newBuffer.set(data);

      double updatingTime = Conversions.nanosecondsToSeconds(System.nanoTime() - updateStartTime);
      System.out.println("LidarImageFusionDataBuffer updatingTime " + updatingTime);
   }

   public void updateLatestStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      latestStereoVisionPointCloudMessage.set(message);
   }

   public void updateLatestBufferedImage(BufferedImage bufferedImage)
   {
      latestBufferedImage.set(bufferedImage);
   }

}
