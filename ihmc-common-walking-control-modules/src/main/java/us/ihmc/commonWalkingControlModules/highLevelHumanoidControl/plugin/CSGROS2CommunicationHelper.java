package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.ContinuousStepGeneratorInputMessage;
import controller_msgs.ContinuousStepGeneratorParametersMessage;
import controller_msgs.ContinuousStepGeneratorStatusMessage;
import std_msgs.Empty;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGenerator;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGeneratorParametersBasics;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.StepGeneratorAPIDefinition;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.HumanoidROS2Topic;
import us.ihmc.communication.ros2.ROS2Heartbeat;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;

import java.util.function.Consumer;

/**
 * This helper facilitates communication with the {@link ContinuousStepGenerator} in the AvatarStepGeneratorThread
 * via the {@link StepGeneratorAPIDefinition} and {@link StepGeneratorNetworkSubscriber}. It can send walking inputs using
 * {@link ContinuousStepGeneratorInputMessage}, and walking parameters using {@link ContinuousStepGeneratorParametersCommand}.
 * Large discrete jumps in CSG parameters are prevented, since the {@link ContinuousStepGeneratorParametersCommand} is regularly
 * reset to current parameter values which are provided via a subscription to {@link ContinuousStepGeneratorStatusMessage}.
 * Additionally, both message types can optionally be published at a lower, throttled rate in order to decrease network traffic.
 *
 * @author Stefan Fasano
 */
public class CSGROS2CommunicationHelper
{
   // This defines the update thread rate for information being sent over ROS2. The reason this is a very low frequency is that
   // we only have a limited amount of networking bandwidth when using WIFI.
   private static final double THROTTLER_THREAD_HERTZ = 11.0;

   public static final ROS2Topic<Empty> CSG_HEARTBEAT_TOPIC = new HumanoidROS2Topic<>().withPrefix("ihmc")
                                                                                       .withModule("continuous_step_generator")
                                                                                       .withSuffix("heartbeat")
                                                                                       .withType(Empty.class);

   private final ROS2Node ros2Node;
   private final String robotName;

   // CSG ROS Commands
   private final ContinuousStepGeneratorInputMessage csgInputCommand = new ContinuousStepGeneratorInputMessage();
   private final ContinuousStepGeneratorParametersMessage csgParametersCommand = new ContinuousStepGeneratorParametersMessage();

   // CSG ROS Statuses
   private final ContinuousStepGeneratorStatusMessage csgStatusMessage = new ContinuousStepGeneratorStatusMessage();

   private final ROS2Publisher<ContinuousStepGeneratorInputMessage> csgInputCommandPublisher;
   private final ROS2Publisher<ContinuousStepGeneratorParametersMessage> csgParametersCommandPublisher;

   // Heartbeat and throttles to ensure StepGeneratorCommandInputManager stops walking if connection is broken
   private final ROS2Heartbeat heartbeat;
   private Throttler ros2Throttler;

   public CSGROS2CommunicationHelper(String robotName, ROS2Node ros2Node, WalkingControllerParameters walkingControllerParameters)
   {
      this(robotName, ros2Node);
      initializeCSGParameters(walkingControllerParameters);

      ros2Throttler = new Throttler();
      ros2Throttler.setFrequency(THROTTLER_THREAD_HERTZ);
   }

   public CSGROS2CommunicationHelper(String robotName, ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
      this.robotName = robotName;

      var csgStatusTopic = HumanoidControllerAPI.getTopic(ContinuousStepGeneratorStatusMessage.class, robotName);
      ros2Node.createSubscriptionSampler(csgStatusTopic, continuousStepGeneratorStatusMessage ->
      {
         csgStatusMessage.set(continuousStepGeneratorStatusMessage);
         setCSGCommandsToCurrentValues(continuousStepGeneratorStatusMessage);
      });

      csgInputCommandPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(ContinuousStepGeneratorInputMessage.class, robotName));
      csgParametersCommandPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(ContinuousStepGeneratorParametersMessage.class, robotName));

      heartbeat = new ROS2Heartbeat(ros2Node, CSG_HEARTBEAT_TOPIC);
      heartbeat.setAlive(true); // Heartbeat stays alive. The StepGeneratorCommandInputManager will stop if this process dies or Wi-Fi connection is lost
   }

   public void publish(ContinuousStepGeneratorInputMessage continuousStepGeneratorInputMessage)
   {
      csgInputCommandPublisher.publish(continuousStepGeneratorInputMessage);
   }

   public void publish(ContinuousStepGeneratorParametersMessage continuousStepGeneratorParametersMessage)
   {
      csgParametersCommandPublisher.publish(continuousStepGeneratorParametersMessage);
   }

   public void publishAtThrottledRate(ContinuousStepGeneratorInputMessage continuousStepGeneratorInputMessage)
   {
      if (ros2Throttler.run())
         csgInputCommandPublisher.publish(continuousStepGeneratorInputMessage);
   }

   public void publishAtThrottledRate(ContinuousStepGeneratorParametersMessage continuousStepGeneratorParametersMessage)
   {
      if (ros2Throttler.run())
         csgParametersCommandPublisher.publish(continuousStepGeneratorParametersMessage);
   }

   public void addVolatileCSGStatusCallbackSubscription(Consumer<ContinuousStepGeneratorStatusMessage> callback)
   {
      var csgStatusTopic = HumanoidControllerAPI.getTopic(ContinuousStepGeneratorStatusMessage.class, robotName);
      ros2Node.createSubscriptionSampler(csgStatusTopic, callback::accept);
   }

   /**
    * Resets our CSG input and parameter commands to the current values being used in
    * CSG to prevent discrete jumps in commands
    */
   private void setCSGCommandsToCurrentValues(ContinuousStepGeneratorStatusMessage csgStatusMessage)
   {
      csgParametersCommand.setSwingDuration(csgStatusMessage.getCurrentSwingDuration());
      csgParametersCommand.setTransferDuration(csgStatusMessage.getCurrentTransferDuration());

      csgParametersCommand.setSwingHeight(csgStatusMessage.getCurrentSwingHeight());
      csgParametersCommand.setMaxStepLengthForwards(csgStatusMessage.getCurrentMaxStepLengthForwards());
      csgParametersCommand.setMaxStepLengthBackwards(csgStatusMessage.getCurrentMaxStepLengthBackwards());
      csgParametersCommand.setDefaultStepWidth(csgStatusMessage.getCurrentDefaultStepWidth());
      csgParametersCommand.setMinStepWidth(csgStatusMessage.getCurrentMinStepWidth());
      csgParametersCommand.setMaxStepWidth(csgStatusMessage.getCurrentMaxStepWidth());
      csgParametersCommand.setTurnMaxAngleInward(csgStatusMessage.getCurrentTurnMaxAngleInward());
      csgParametersCommand.setTurnMaxAngleOutward(csgStatusMessage.getCurrentTurnMaxAngleOutward());
   }

   /**
    * Initialize CSG parameter command to default CSG parameters
    */
   public void initializeCSGParameters(WalkingControllerParameters walkingControllerParameters)
   {
      csgInputCommand.setWalk(false);
      csgInputCommand.setForwardVelocity(0.0);
      csgInputCommand.setLateralVelocity(0.0);
      csgInputCommand.setTurnVelocity(0.0);

      csgParametersCommand.setNumberOfFootstepsToPlan(ContinuousStepGeneratorParametersBasics.DEFAULT_NUMBER_OF_FOOTSTEPS_TO_PLAN);
      csgParametersCommand.setNumberOfFixedFootsteps(ContinuousStepGeneratorParametersBasics.DEFAULT_NUMBER_OF_FIXED_FOOTSTEPS);
      csgParametersCommand.setSwingDuration(walkingControllerParameters.getDefaultSwingTime());
      csgParametersCommand.setTransferDuration(walkingControllerParameters.getDefaultTransferTime());

      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParametersForStepGeneration();
      csgParametersCommand.setSwingHeight(walkingControllerParameters.getSwingTrajectoryParameters().getDefaultSwingHeight());
      csgParametersCommand.setMaxStepLengthForwards(steppingParameters.getMaxStepLength());
      csgParametersCommand.setMaxStepLengthBackwards(steppingParameters.getMaxBackwardStepLength());
      csgParametersCommand.setDefaultStepWidth(steppingParameters.getInPlaceWidth());
      csgParametersCommand.setMinStepWidth(steppingParameters.getMinStepWidth());
      csgParametersCommand.setMaxStepWidth(steppingParameters.getMaxStepWidth());
      csgParametersCommand.setTurnMaxAngleInward(steppingParameters.getMaxAngleTurnInwards());
      csgParametersCommand.setTurnMaxAngleOutward(steppingParameters.getMaxAngleTurnOutwards());

      // CSG status message
      csgStatusMessage.setIsWalking(false);
      csgStatusMessage.setAreVelocitiesNormalized(true);
      csgStatusMessage.setCurrentForwardVelocity(0.0);
      csgStatusMessage.setCurrentLateralVelocity(0.0);
      csgStatusMessage.setCurrentTurnVelocity(0.0);

      csgStatusMessage.setCurrentSwingHeight(walkingControllerParameters.getSwingTrajectoryParameters().getDefaultSwingHeight());
      csgStatusMessage.setCurrentSwingDuration(walkingControllerParameters.getDefaultSwingTime());
      csgStatusMessage.setCurrentTransferDuration(walkingControllerParameters.getDefaultTransferTime());
      csgStatusMessage.setCurrentMaxStepLengthForwards(steppingParameters.getMaxStepLength());
      csgStatusMessage.setCurrentMaxStepLengthBackwards(steppingParameters.getMaxBackwardStepLength());
      csgStatusMessage.setCurrentMaxStepWidth(steppingParameters.getMaxStepWidth());
      csgStatusMessage.setCurrentMinStepWidth(steppingParameters.getMinStepWidth());
      csgStatusMessage.setCurrentDefaultStepWidth(steppingParameters.getInPlaceWidth());
      csgStatusMessage.setCurrentTurnMaxAngleOutward(steppingParameters.getMaxAngleTurnOutwards());
      csgStatusMessage.setCurrentTurnMaxAngleInward(steppingParameters.getMaxAngleTurnInwards());
      csgStatusMessage.setAreStepsAdjustable(csgParametersCommand.getStepsAreAdjustable());
   }

   public ContinuousStepGeneratorInputMessage getCSGInputCommand()
   {
      return csgInputCommand;
   }

   public ContinuousStepGeneratorParametersMessage getCSGParametersCommand()
   {
      return csgParametersCommand;
   }

   public ContinuousStepGeneratorStatusMessage getCSGStatusMessage()
   {
      return csgStatusMessage;
   }

   public void destroy()
   {
      heartbeat.destroy();
   }
}
