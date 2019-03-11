package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.REASensorDataFilterParametersMessage;
import controller_msgs.msg.dds.REAStatusMessage;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.ros2.Ros2Node;

public class REAPlanarRegionPublicNetworkProvider
{
   private final IHMCROS2Publisher<PlanarRegionsListMessage> planarRegionPublisher;
   private final IHMCROS2Publisher<REAStatusMessage> currentStatePublisher;

   private final RegionFeaturesProvider regionFeaturesProvider;

   private final AtomicReference<Boolean> isRunning, hasCleared, isUsingLidar, isUsingStereoVision;
   private final AtomicReference<Double> minRange;
   private final AtomicReference<Double> maxRange;
   private final AtomicReference<BoundingBoxParametersMessage> boundingBoxParameters;
   private final REAStatusMessage currentState = new REAStatusMessage();

   public REAPlanarRegionPublicNetworkProvider(Messager messager, RegionFeaturesProvider regionFeaturesProvider, Ros2Node ros2Node,
                                               MessageTopicNameGenerator publisherTopicNameGenerator, MessageTopicNameGenerator subscriberTopicNameGenerator)
   {
      this.regionFeaturesProvider = regionFeaturesProvider;
      planarRegionPublisher = ROS2Tools.createPublisher(ros2Node, PlanarRegionsListMessage.class, publisherTopicNameGenerator);

      if (messager != null)
      {
         currentStatePublisher = ROS2Tools.createPublisher(ros2Node, REAStatusMessage.class, publisherTopicNameGenerator);
         isRunning = messager.createInput(REAModuleAPI.OcTreeEnable);
         // This should be the only input with a default value, the rest gets populated at the very start.
         hasCleared = messager.createInput(REAModuleAPI.OcTreeClear, false);
         isUsingLidar = messager.createInput(REAModuleAPI.LidarBufferEnable);
         isUsingStereoVision = messager.createInput(REAModuleAPI.StereoVisionBufferEnable);
         minRange = messager.createInput(REAModuleAPI.LidarMinRange);
         maxRange = messager.createInput(REAModuleAPI.LidarMaxRange);
         boundingBoxParameters = messager.createInput(REAModuleAPI.OcTreeBoundingBoxParameters);
      }
      else
      { // The ConstantPlanarRegionsPublisher creates this class without the messager.
         currentStatePublisher = null;
         isRunning = null;
         hasCleared = null;
         isUsingLidar = null;
         isUsingStereoVision = null;
         minRange = null;
         maxRange = null;
         boundingBoxParameters = null;
      }
   }

   private PlanarRegionsListMessage lastPlanarRegionsListMessage;

   public void update(boolean planarRegionsHaveBeenUpdated)
   {
      if (regionFeaturesProvider.getPlanarRegionsList() == null)
         return;

      if (regionFeaturesProvider.getPlanarRegionsList().isEmpty())
         return;

      if (planarRegionsHaveBeenUpdated)
         lastPlanarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regionFeaturesProvider.getPlanarRegionsList());

      planarRegionPublisher.publish(lastPlanarRegionsListMessage);
   }

   public void publishCurrentState()
   {
      if (currentStatePublisher == null)
         return;

      currentState.setIsRunning(isRunning.get());
      currentState.setIsUsingLidar(isUsingLidar.get());
      currentState.setIsUsingStereoVision(isUsingStereoVision.get());
      currentState.setHasCleared(hasCleared.getAndSet(false));
      REASensorDataFilterParametersMessage sensorFilterParameters = currentState.getCurrentSensorFilterParameters();
      sensorFilterParameters.setSensorMinRange(minRange.get());
      sensorFilterParameters.setSensorMaxRange(maxRange.get());
      sensorFilterParameters.getBoundingBoxMin().set(boundingBoxParameters.get().getMin());
      sensorFilterParameters.getBoundingBoxMax().set(boundingBoxParameters.get().getMax());
      currentStatePublisher.publish(currentState);
   }
}
