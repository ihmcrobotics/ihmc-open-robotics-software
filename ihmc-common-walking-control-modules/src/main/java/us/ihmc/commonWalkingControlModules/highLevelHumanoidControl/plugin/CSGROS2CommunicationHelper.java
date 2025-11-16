package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.plugin;

import controller_msgs.msg.dds.ContinuousStepGeneratorInputMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorParametersMessage;
import controller_msgs.msg.dds.ContinuousStepGeneratorStatusMessage;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.ContinuousStepGeneratorParametersBasics;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.QueuedROS2Subscription;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

import java.util.function.Consumer;

public class CSGROS2CommunicationHelper
{
   private final ROS2Node ros2Node;
   private final String robotName;

   // CSG ROS Commands
   private final ContinuousStepGeneratorInputMessage csgInputCommand = new ContinuousStepGeneratorInputMessage();
   private final ContinuousStepGeneratorParametersMessage csgParametersCommand = new ContinuousStepGeneratorParametersMessage();

   // CSG ROS Statuses
   private final QueuedROS2Subscription<ContinuousStepGeneratorStatusMessage> csgStatusSubscription;
   private final ContinuousStepGeneratorStatusMessage csgStatusMessage = new ContinuousStepGeneratorStatusMessage();

   private final ROS2Publisher<ContinuousStepGeneratorInputMessage> csgInputCommandPublisher;
   private final ROS2Publisher<ContinuousStepGeneratorParametersMessage> csgParametersCommandPublisher;

   public CSGROS2CommunicationHelper(String robotName, ROS2Node ros2Node, WalkingControllerParameters walkingControllerParameters)
   {
      this(robotName, ros2Node);
      initializeCSGParameters(walkingControllerParameters);
   }

   public CSGROS2CommunicationHelper(String robotName, ROS2Node ros2Node)
   {
      this.ros2Node = ros2Node;
      this.robotName = robotName;

      csgStatusSubscription = ros2Node.createQueuedSubscription(HumanoidControllerAPI.getTopic(ContinuousStepGeneratorStatusMessage.class, robotName), 10);

      ROS2Tools.createVolatileCallbackSubscription(ros2Node,
                                                   HumanoidControllerAPI.getTopic(ContinuousStepGeneratorStatusMessage.class, robotName),
                                                   continuousStepGeneratorStatusMessage ->
                                                   {
                                                      csgStatusMessage.set(continuousStepGeneratorStatusMessage);
                                                      setCSGCommandsToCurrentValues(continuousStepGeneratorStatusMessage);
                                                   });

      csgInputCommandPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(ContinuousStepGeneratorInputMessage.class, robotName));
      csgParametersCommandPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getTopic(ContinuousStepGeneratorParametersMessage.class, robotName));
   }

   public void publish(ContinuousStepGeneratorInputMessage continuousStepGeneratorInputMessage)
   {
      csgInputCommandPublisher.publish(continuousStepGeneratorInputMessage);
   }

   public void publish(ContinuousStepGeneratorParametersMessage continuousStepGeneratorParametersMessage)
   {
      csgParametersCommandPublisher.publish(continuousStepGeneratorParametersMessage);
   }

   public void addVolatileCSGStatusCallbackSubscription(Consumer<ContinuousStepGeneratorStatusMessage> callback)
   {
      ROS2Tools.createVolatileCallbackSubscription(ros2Node, HumanoidControllerAPI.getTopic(ContinuousStepGeneratorStatusMessage.class, robotName), callback);
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
      csgStatusMessage.setIsInUnitVelocities(false);
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
}
