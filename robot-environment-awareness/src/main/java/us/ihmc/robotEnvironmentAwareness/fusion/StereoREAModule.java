package us.ihmc.robotEnvironmentAwareness.fusion;

import java.awt.image.BufferedImage;
import java.text.DecimalFormat;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.Image32;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.fusion.data.StereoREAPlanarRegionFeatureUpdater;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.ImageVisualizationHelper;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class StereoREAModule implements Runnable
{
   private final Messager reaMessager;
   private final Messager messager;

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> isRunning = new AtomicReference<Boolean>(false);
   private final StereoREAPlanarRegionFeatureUpdater planarRegionFeatureUpdater;

   private final REANetworkProvider networkProvider;

   private final AtomicReference<StereoVisionPointCloudMessage> loadedStereoVisionPointCloudMessage;
   private final AtomicReference<Image32> loadedImage32Message;

   public StereoREAModule(REANetworkProvider networkProvider, Messager reaMessager, SharedMemoryJavaFXMessager messager)
   {
      this.networkProvider = networkProvider;
      this.messager = messager;
      this.reaMessager = reaMessager;
      planarRegionFeatureUpdater = new StereoREAPlanarRegionFeatureUpdater(reaMessager, messager);

      enable = messager.createInput(LidarImageFusionAPI.EnableREA, false);

      loadedStereoVisionPointCloudMessage = reaMessager.createInput(REAModuleAPI.StereoVisionPointCloudState);
      loadedImage32Message = messager.createInput(LidarImageFusionAPI.ImageState);

      initializeREAPlanarRegionPublicNetworkProvider();
   }

   private void initializeREAPlanarRegionPublicNetworkProvider()
   {
      reaMessager.submitMessage(REAModuleAPI.LidarBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.StereoVisionBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.OcTreeClear, false);
      reaMessager.submitMessage(REAModuleAPI.LidarMinRange, Double.NEGATIVE_INFINITY);
      reaMessager.submitMessage(REAModuleAPI.LidarMaxRange, Double.POSITIVE_INFINITY);
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxParameters, new BoundingBoxParametersMessage());
   }

   public void dispatchCustomPlanarRegion(PlanarRegionsListMessage message)
   {
      PlanarRegionsList customPlanarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);
      customPlanarRegions.getPlanarRegionsAsList().forEach(planarRegionFeatureUpdater::registerCustomPlanarRegion);
   }

   public void updateLatestStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
   }

   public void updateLatestBufferedImage(BufferedImage bufferedImage)
   {
   }

   @Override
   public void run()
   {
      if (!enable.get())
         return;

      singleRun();
   }

   public void singleRun()
   {
      StereoVisionPointCloudMessage loadedStereoMessage = loadedStereoVisionPointCloudMessage.getAndSet(null);
      if (loadedStereoMessage != null)
         updateLatestStereoVisionPointCloudMessage(loadedStereoMessage);

      Image32 loadedImageMessage = loadedImage32Message.getAndSet(null);
      if (loadedImageMessage != null)
         updateLatestBufferedImage(ImageVisualizationHelper.convertImageMessageToBufferedImage(loadedImageMessage));

      isRunning.set(true);
      long runningStartTime = System.nanoTime();


      if (planarRegionFeatureUpdater.update())
      {
         reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, true); // TODO: replace, or modify.
         reportPlanarRegionState();
      }

      double runningTime = Conversions.nanosecondsToSeconds(System.nanoTime() - runningStartTime);
      String computationTime = new DecimalFormat("##.###").format(runningTime) + "(sec)";
      messager.submitMessage(LidarImageFusionAPI.ComputationTime, computationTime);
      isRunning.set(false);
   }

   public void enable()
   {
      enable.set(true);
   }

   private void reportPlanarRegionState()
   {
      if (planarRegionFeatureUpdater.getPlanarRegionsList() != null)
      {
         PlanarRegionsList planarRegionsList = planarRegionFeatureUpdater.getPlanarRegionsList();
         PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
         reaMessager.submitMessage(REAModuleAPI.PlanarRegionsState, planarRegionsListMessage);

         networkProvider.update(planarRegionFeatureUpdater, true);
         networkProvider.publishCurrentState();
      }
   }
}
