package us.ihmc.robotEnvironmentAwareness.fusion;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.publisherTopicNameGenerator;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberTopicNameGenerator;

import java.awt.image.BufferedImage;
import java.text.DecimalFormat;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionData;
import us.ihmc.robotEnvironmentAwareness.fusion.data.LidarImageFusionDataBuffer;
import us.ihmc.robotEnvironmentAwareness.fusion.data.StereoREAPlanarRegionFeatureUpdater;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.Ros2Node;

public class StereoREAModule implements Runnable
{
   private final Ros2Node ros2Node;
   private final Messager reaMessager;
   private final Messager messager;

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> isRunning = new AtomicReference<Boolean>(false);
   private final LidarImageFusionDataBuffer lidarImageFusionDataBuffer;
   private final StereoREAPlanarRegionFeatureUpdater planarRegionFeatureUpdater;

   private final REAPlanarRegionPublicNetworkProvider planarRegionNetworkProvider;

   public StereoREAModule(Ros2Node ros2Node, Messager reaMessager, SharedMemoryJavaFXMessager messager)
   {
      this.ros2Node = ros2Node;
      this.messager = messager;
      this.reaMessager = reaMessager;
      lidarImageFusionDataBuffer = new LidarImageFusionDataBuffer(messager, PointCloudProjectionHelper.multisenseOnCartIntrinsicParameters);
      planarRegionFeatureUpdater = new StereoREAPlanarRegionFeatureUpdater(reaMessager, messager);

      enable = messager.createInput(LidarImageFusionAPI.EnableREA, false);

      planarRegionNetworkProvider = new REAPlanarRegionPublicNetworkProvider(reaMessager, planarRegionFeatureUpdater, ros2Node, publisherTopicNameGenerator,
                                                                             subscriberTopicNameGenerator);

      //TODO : initialize.
      reaMessager.submitMessage(REAModuleAPI.LidarBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.StereoVisionBufferEnable, false);
      reaMessager.submitMessage(REAModuleAPI.OcTreeClear, false);
      reaMessager.submitMessage(REAModuleAPI.LidarMinRange, Double.NEGATIVE_INFINITY);
      reaMessager.submitMessage(REAModuleAPI.LidarMaxRange, Double.POSITIVE_INFINITY);
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxParameters, new BoundingBoxParametersMessage());
   }

   public void registerCustomPlanarRegion(PlanarRegion planarRegion)
   {
      planarRegionFeatureUpdater.registerCustomPlanarRegion(planarRegion);
   }

   public void dispatchCustomPlanarRegion(PlanarRegionsListMessage message)
   {
      PlanarRegionsList customPlanarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);
      customPlanarRegions.getPlanarRegionsAsList().forEach(planarRegionFeatureUpdater::registerCustomPlanarRegion);
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
      if (!enable.get())
         return;

      isRunning.set(true);
      long runningStartTime = System.nanoTime();

      lidarImageFusionDataBuffer.updateNewBuffer();

      LidarImageFusionData newBuffer = lidarImageFusionDataBuffer.pollNewBuffer();
      messager.submitMessage(LidarImageFusionAPI.FusionDataState, newBuffer);

      planarRegionFeatureUpdater.updateLatestLidarImageFusionData(newBuffer);

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

   public void singleRun()
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

         planarRegionNetworkProvider.update(true);
         planarRegionNetworkProvider.publishCurrentState();
      }
   }
}
