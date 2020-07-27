package us.ihmc.robotEnvironmentAwareness.updaters;

import controller_msgs.msg.dds.*;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.ros.REAModuleROS2Subscription;
import us.ihmc.robotEnvironmentAwareness.ros.REASourceType;
import us.ihmc.ros2.NewMessageListener;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.inputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.subscriberCustomRegionsTopicName;

public class REACurrentStateProvider
{
   private final IHMCROS2Publisher<REAStatusMessage> currentStatePublisher;
   private final AtomicReference<Boolean> isRunning, hasCleared, isUsingLidar, isUsingStereoVision, isUsingDepthCloud;
   private final AtomicReference<Double> minRange, maxRange;
   private final AtomicReference<BoundingBoxParametersMessage> boundingBoxParameters;
   private final REAStatusMessage currentState = new REAStatusMessage();

   public REACurrentStateProvider(Ros2Node ros2Node,
                                  ROS2Topic outputTopic,
                                  Messager messager)
   {
      currentStatePublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, REAStatusMessage.class, outputTopic);
      isRunning = messager.createInput(REAModuleAPI.OcTreeEnable);
      // This should be the only input with a default value, the rest gets populated at the very start.
      hasCleared = messager.createInput(REAModuleAPI.OcTreeClear, false);
      isUsingLidar = messager.createInput(REAModuleAPI.LidarBufferEnable);
      isUsingStereoVision = messager.createInput(REAModuleAPI.StereoVisionBufferEnable);
      isUsingDepthCloud = messager.createInput(REAModuleAPI.DepthCloudBufferEnable);
      minRange = messager.createInput(REAModuleAPI.LidarMinRange);
      maxRange = messager.createInput(REAModuleAPI.LidarMaxRange);
      boundingBoxParameters = messager.createInput(REAModuleAPI.OcTreeBoundingBoxParameters);
   }

   public void publishCurrentState()
   {
      currentState.setIsRunning(isRunning.get());
      currentState.setIsUsingLidar(isUsingLidar.get());
      currentState.setIsUsingStereoVision(isUsingStereoVision.get());
      //currentState.setIsusingDepthCloud(isUsingDepthCloud.get()); // todo
      currentState.setHasCleared(hasCleared.getAndSet(false));
      REASensorDataFilterParametersMessage sensorFilterParameters = currentState.getCurrentSensorFilterParameters();
      sensorFilterParameters.setSensorMinRange(minRange.get());
      sensorFilterParameters.setSensorMaxRange(maxRange.get());
      sensorFilterParameters.getBoundingBoxMin().set(boundingBoxParameters.get().getMin());
      sensorFilterParameters.getBoundingBoxMax().set(boundingBoxParameters.get().getMax());
      currentStatePublisher.publish(currentState);
   }
}
