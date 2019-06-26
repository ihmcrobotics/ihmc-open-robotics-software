package us.ihmc.robotEnvironmentAwareness.fusion.data;

import java.awt.image.BufferedImage;
import java.util.concurrent.atomic.AtomicReference;

import boofcv.struct.calib.IntrinsicParameters;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.ImageSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.parameters.SegmentationRawDataFilteringParameters;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.LidarImageFusionDataFactory;

public class LidarImageFusionDataBuffer
{
   //private final IntrinsicParameters intrinsicParameters;

   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionPointCloudMessage = new AtomicReference<>(null);
   private final AtomicReference<BufferedImage> latestBufferedImage = new AtomicReference<>(null);

   private final AtomicReference<Point3D> latestCameraPosition;
   private final AtomicReference<Quaternion> latestCameraOrientation;
   private final AtomicReference<IntrinsicParameters> latestCameraIntrinsicParameters;
   
   private final AtomicReference<Integer> bufferSize;

   private final AtomicReference<SegmentationRawDataFilteringParameters> latestSegmentationRawDataFilteringParameters;
   private final AtomicReference<ImageSegmentationParameters> latestImageSegmentationParaeters;
   private final AtomicReference<LidarImageFusionData> newBuffer = new AtomicReference<>(null);

   public LidarImageFusionDataBuffer(Messager messager, IntrinsicParameters intrinsic)
   {
      //intrinsicParameters = intrinsic;
      bufferSize = messager.createInput(LidarImageFusionAPI.StereoBufferSize, 50000);
      latestImageSegmentationParaeters = messager.createInput(LidarImageFusionAPI.ImageSegmentationParameters, new ImageSegmentationParameters());
      latestSegmentationRawDataFilteringParameters = messager.createInput(LidarImageFusionAPI.SegmentationRawDataFilteringParameters,
                                                                          new SegmentationRawDataFilteringParameters());
      
      latestCameraPosition = messager.createInput(LidarImageFusionAPI.CameraPositionState, new Point3D());
      latestCameraOrientation = messager.createInput(LidarImageFusionAPI.CameraOrientationState, new Quaternion());
      latestCameraIntrinsicParameters = messager.createInput(LidarImageFusionAPI.CameraIntrinsicParametersState, new IntrinsicParameters());
   }

   public LidarImageFusionData pollNewBuffer()
   {
      return newBuffer.getAndSet(null);
   }

   public void updateNewBuffer()
   {
      StereoVisionPointCloudMessage stereoVisionPointCloudMessage = latestStereoVisionPointCloudMessage.get();
      ImageSegmentationParameters imageSegmentationParameters = latestImageSegmentationParaeters.get();

      SegmentationRawDataFilteringParameters segmentationRawDataFilteringParameters = latestSegmentationRawDataFilteringParameters.get();
      LidarImageFusionData data = LidarImageFusionDataFactory.createLidarImageFusionData(MessageTools.unpackScanPoint3ds(stereoVisionPointCloudMessage),
                                                                                         latestBufferedImage.get(), latestCameraIntrinsicParameters.get(), bufferSize.get(),
                                                                                         latestCameraPosition.get(), latestCameraOrientation.get(),
                                                                                         imageSegmentationParameters.getPixelSize(),
                                                                                         imageSegmentationParameters.getPixelRuler(),
                                                                                         true, imageSegmentationParameters.getMinElementSize(),
                                                                                         imageSegmentationParameters.getIterate(),
                                                                                         segmentationRawDataFilteringParameters.isEnableFilterFlyingPoint(),
                                                                                         segmentationRawDataFilteringParameters.getFlyingPointThreshold(),
                                                                                         segmentationRawDataFilteringParameters.getMinimumNumberOfFlyingPointNeighbors());
      newBuffer.set(data);
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
