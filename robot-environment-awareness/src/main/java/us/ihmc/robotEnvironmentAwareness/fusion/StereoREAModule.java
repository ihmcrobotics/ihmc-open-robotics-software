package us.ihmc.robotEnvironmentAwareness.fusion;

import java.awt.image.BufferedImage;
import java.text.DecimalFormat;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionData;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionDataBuffer;
import us.ihmc.robotEnvironmentAwareness.fusion.data.StereoREAPlanarRegionFeatureUpdater;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;

public class StereoREAModule implements Runnable
{
   private final Messager reaMessager;
   private final Messager messager;

   private final AtomicReference<Boolean> isRunning = new AtomicReference<Boolean>(false);
   private final LidarImageFusionDataBuffer lidarImageFusionDataBuffer;
   private final StereoREAPlanarRegionFeatureUpdater planarRegionFeatureUpdater;

   public StereoREAModule(Messager reaMessager, SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
      this.reaMessager = reaMessager;
      lidarImageFusionDataBuffer = new LidarImageFusionDataBuffer(messager, PointCloudProjectionHelper.multisenseOnCartIntrinsicParameters);
      planarRegionFeatureUpdater = new StereoREAPlanarRegionFeatureUpdater(reaMessager);
   }

   public void updateLatestStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      lidarImageFusionDataBuffer.updateLatestStereoVisionPointCloudMessage(message);
   }

   public void updateLatestBufferedImage(BufferedImage bufferedImage)
   {
      lidarImageFusionDataBuffer.updateLatestBufferedImage(bufferedImage);
   }

   @Override
   public void run()
   {
      isRunning.set(true);
      long runningStartTime = System.nanoTime();

      long updateStartTime = System.nanoTime();
      lidarImageFusionDataBuffer.updateNewBuffer();
      double updatingTime = Conversions.nanosecondsToSeconds(System.nanoTime() - updateStartTime);
      System.out.println("LidarImageFusionDataBuffer updatingTime " + updatingTime);

      long submitStartTime = System.nanoTime();
      LidarImageFusionData newBuffer = lidarImageFusionDataBuffer.pollNewBuffer();
      messager.submitMessage(LidarImageFusionAPI.FusionDataState, newBuffer);
      double submittingTime = Conversions.nanosecondsToSeconds(System.nanoTime() - submitStartTime);
      System.out.println("LidarImageFusionDataBuffer submittingTime " + submittingTime);

      planarRegionFeatureUpdater.updateLatestLidarImageFusionData(newBuffer);

      long calculationStartTime = System.nanoTime();
      if (planarRegionFeatureUpdater.update())
      {
         reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, true); // TODO: replace, or modify.
         reportPlanarRegionState();
      }
      double calculationTime = Conversions.nanosecondsToSeconds(System.nanoTime() - calculationStartTime);
      System.out.println("LidarImageFusionDataBuffer calculationTime " + calculationTime);

      double runningTime = Conversions.nanosecondsToSeconds(System.nanoTime() - runningStartTime);
      String computationTime = new DecimalFormat("##.###").format(runningTime) + "(sec)";
      messager.submitMessage(LidarImageFusionAPI.ComputationTime, computationTime);
      isRunning.set(false);
   }

   private void reportPlanarRegionState()
   {
      if (planarRegionFeatureUpdater.getPlanarRegionsList() != null)
      {
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionFeatureUpdater.getPlanarRegionsList());
         reaMessager.submitMessage(REAModuleAPI.PlanarRegionsState, planarRegionsListMessage);
      }
   }
}
