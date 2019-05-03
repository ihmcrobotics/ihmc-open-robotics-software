package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelHumanoidControllerFactory;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.TypedNotification;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;

public class RemoteRobotControllerInterface
{
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private final IHMCROS2Publisher<PauseWalkingMessage> pausePublisher;

   private final ArrayList<TypedNotification<WalkingStatusMessage>> walkingCompletedNotifications = new ArrayList<>();
   private final SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;
   private final ROS2Input<HighLevelStateChangeStatusMessage> controllerState;

   public RemoteRobotControllerInterface(Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      footstepDataListPublisher = ROS2Tools.createPublisher(ros2Node,
                                                            ROS2Tools.newMessageInstance(FootstepDataListCommand.class).getMessageClass(),
                                                            ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));
      pausePublisher = ROS2Tools.createPublisher(ros2Node,
                                                 ROS2Tools.newMessageInstance(PauseWalkingCommand.class).getMessageClass(),
                                                 ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName()));

      new ROS2Callback<>(ros2Node,
                         WalkingStatusMessage.class,
                         robotModel.getSimpleRobotName(),
                         HighLevelHumanoidControllerFactory.ROS2_ID,
                         this::acceptWalkingStatus);

      HighLevelStateChangeStatusMessage initialState = new HighLevelStateChangeStatusMessage();
      initialState.setInitialHighLevelControllerName(HighLevelControllerName.DO_NOTHING_BEHAVIOR.toByte());
      initialState.setEndHighLevelControllerName(HighLevelControllerName.WALKING.toByte());
      controllerState = new ROS2Input<>(ros2Node,
                                        HighLevelStateChangeStatusMessage.class,
                                        robotModel.getSimpleRobotName(),
                                        HighLevelHumanoidControllerFactory.ROS2_ID,
                                        initialState,
                                        this::acceptStatusChange);

      YoVariableRegistry registry = new YoVariableRegistry("swingOver");
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      swingOverPlanarRegionsTrajectoryExpander = new SwingOverPlanarRegionsTrajectoryExpander(robotModel.getWalkingControllerParameters(),
                                                                                              registry,
                                                                                              yoGraphicsListRegistry);
   }

   private boolean acceptStatusChange(HighLevelStateChangeStatusMessage message)
   {
      HighLevelControllerName fromState = HighLevelControllerName.fromByte(message.getInitialHighLevelControllerName());
      HighLevelControllerName toState = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());
      LogTools.debug("Controller state: {} to {}", fromState == null ? fromState : fromState.name(), toState == null ? toState : toState.name());
      return toState != null;
   }

   private void acceptWalkingStatus(WalkingStatusMessage message)
   {
      LogTools.debug("Walking status: {}", WalkingStatus.fromByte(message.getWalkingStatus()).name());
      if (message.getWalkingStatus() == WalkingStatusMessage.COMPLETED)
      {
         while (!walkingCompletedNotifications.isEmpty())
         {
            walkingCompletedNotifications.remove(0).add(message);
         }
      }
   }

   public TypedNotification<WalkingStatusMessage> requestWalk(FootstepPlanningToolboxOutputStatus footstepPlanningToolboxOutput,
                                                              HumanoidReferenceFrames humanoidReferenceFrames,
                                                              boolean swingOverPlanarRegions)
   {

      FootstepDataListMessage footsteps = footstepPlanningToolboxOutput.getFootstepDataList();
      if (swingOverPlanarRegions)
      {
         footsteps = calculateSwingOverTrajectoryExpansions(footsteps, humanoidReferenceFrames, footstepPlanningToolboxOutput.getPlanarRegionsList());
      }

      LogTools.debug("Tasking {} footstep(s) to the robot", footsteps.getFootstepDataList().size());

      footstepDataListPublisher.publish(footsteps);

      TypedNotification<WalkingStatusMessage> walkingCompletedNotification = new TypedNotification<>();
      walkingCompletedNotifications.add(walkingCompletedNotification);
      return walkingCompletedNotification;
   }

   private FootstepDataListMessage calculateSwingOverTrajectoryExpansions(FootstepDataListMessage footsteps,
                                                                          HumanoidReferenceFrames humanoidReferenceFrames,
                                                                          PlanarRegionsListMessage messagePlanarRegionsList)
   {
      LogTools.debug("Calculating swing over planar regions...");

      double swingTime = 0.6;
      double transferTime = 0.25;
      double maxSwingSpeed = 1.0;
      FramePose3D stanceFootPose = new FramePose3D();
      FramePose3D swingStartPose = new FramePose3D();
      FramePose3D swingEndPose = new FramePose3D();
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(messagePlanarRegionsList);

      RobotSide firstSwingFoot = RobotSide.fromByte(footsteps.getFootstepDataList().get(0).getRobotSide());
      stanceFootPose.setFromReferenceFrame(humanoidReferenceFrames.getSoleFrame(firstSwingFoot));
      swingEndPose.setFromReferenceFrame(humanoidReferenceFrames.getSoleFrame(firstSwingFoot.getOppositeSide())); // first stance foot

      int i = 0;
      for (FootstepDataMessage footstepData : footsteps.getFootstepDataList())
      {
         swingStartPose.set(stanceFootPose);
         stanceFootPose.set(swingEndPose);
         swingEndPose.set(footstepData.getLocation(), footstepData.getOrientation());

         double maxSpeedDimensionless = swingOverPlanarRegionsTrajectoryExpander
               .expandTrajectoryOverPlanarRegions(stanceFootPose, swingStartPose, swingEndPose, planarRegionsList);

         LogTools.debug("Step " + ++i + ": " + swingOverPlanarRegionsTrajectoryExpander.getStatus());
         LogTools.debug("Foot: " + footstepData.getRobotSide() + "  Position: " + footstepData.getLocation());

         footstepData.setTrajectoryType(TrajectoryType.CUSTOM.toByte());
         Point3D waypointOne = new Point3D(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(0));
         Point3D waypointTwo = new Point3D(swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints().get(1));
         MessageTools.copyData(new Point3D[] {waypointOne, waypointTwo}, footstepData.getCustomPositionWaypoints());

         double maxSpeed = maxSpeedDimensionless / swingTime;
         if (maxSpeed > maxSwingSpeed)
         {
            double adjustedSwingTime = maxSpeedDimensionless / maxSwingSpeed;
            footstepData.setSwingDuration(adjustedSwingTime);
            footstepData.setTransferDuration(transferTime);
         }
      }
      return footsteps;
   }

   public void pauseWalking()
   {
      LogTools.debug("Sending pause walking to robot");
      PauseWalkingMessage pause = new PauseWalkingMessage();
      pause.setPause(true);
      pausePublisher.publish(pause);
   }

   public HighLevelControllerName latestControllerState()
   {
      return HighLevelControllerName.fromByte(controllerState.getLatest().getEndHighLevelControllerName());
   }
}
