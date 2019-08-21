package us.ihmc.humanoidBehaviors.tools;

import java.util.ArrayList;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import controller_msgs.msg.dds.PauseWalkingMessage;
import controller_msgs.msg.dds.PelvisOrientationTrajectoryMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.communication.*;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.PlanTravelDistance;
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

public class RemoteRobotControllerInterface
{
   //TODO: Clean this up by using DRCUserInterfaceNetworkingManager (After cleaning that up first...)

   private final IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private final IHMCROS2Publisher<PauseWalkingMessage> pausePublisher;
   private final IHMCROS2Publisher<FootTrajectoryMessage> footTrajectoryMessagePublisher;
   private final IHMCROS2Publisher<ArmTrajectoryMessage> armTrajectoryMessagePublisher;
   private final IHMCROS2Publisher<ChestTrajectoryMessage> chestOrientationTrajectoryMessagePublisher;
   private final IHMCROS2Publisher<PelvisOrientationTrajectoryMessage> pelvisOrientationTrajectoryMessagePublisher;
   private final IHMCROS2Publisher<PelvisTrajectoryMessage> pelvisTrajectoryMessagePublisher;
   private final IHMCROS2Publisher<GoHomeMessage> goHomeMessagePublisher;

   private final ArrayList<TypedNotification<WalkingStatusMessage>> walkingCompletedNotifications = new ArrayList<>();
   private final SwingOverPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;
   private final ROS2Input<HighLevelStateChangeStatusMessage> controllerState;

   public RemoteRobotControllerInterface(Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      String robotName = robotModel.getSimpleRobotName();
      ROS2ModuleIdentifier controllerId = ROS2Tools.HUMANOID_CONTROLLER;

      footTrajectoryMessagePublisher = new IHMCROS2Publisher<>(ros2Node, FootTrajectoryMessage.class, robotName, controllerId);
      armTrajectoryMessagePublisher = new IHMCROS2Publisher<>(ros2Node, ArmTrajectoryMessage.class, robotName, controllerId);
      chestOrientationTrajectoryMessagePublisher = new IHMCROS2Publisher<>(ros2Node, ChestTrajectoryMessage.class, robotName, controllerId);
      pelvisOrientationTrajectoryMessagePublisher = new IHMCROS2Publisher<>(ros2Node, PelvisOrientationTrajectoryMessage.class, robotName, controllerId);
      pelvisTrajectoryMessagePublisher = new IHMCROS2Publisher<>(ros2Node, PelvisTrajectoryMessage.class, robotName, controllerId);
      goHomeMessagePublisher = new IHMCROS2Publisher<>(ros2Node, GoHomeMessage.class, robotName, controllerId);
      footstepDataListPublisher = new IHMCROS2Publisher<>(ros2Node, FootstepDataListMessage.class, robotName, controllerId);
      pausePublisher = new IHMCROS2Publisher<>(ros2Node, PauseWalkingMessage.class, robotName, controllerId);

      new ROS2Callback<>(ros2Node, WalkingStatusMessage.class, robotName, controllerId, this::acceptWalkingStatus);

      HighLevelStateChangeStatusMessage initialState = new HighLevelStateChangeStatusMessage();
      initialState.setInitialHighLevelControllerName(HighLevelControllerName.DO_NOTHING_BEHAVIOR.toByte());
      initialState.setEndHighLevelControllerName(HighLevelControllerName.WALKING.toByte());
      controllerState = new ROS2Input<>(ros2Node, HighLevelStateChangeStatusMessage.class, robotName, controllerId, initialState, this::acceptStatusChange);

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

   public TypedNotification<WalkingStatusMessage> requestWalk(FootstepDataListMessage footstepPlan, HumanoidReferenceFrames humanoidReferenceFrames)
   {
      return requestWalk(footstepPlan, humanoidReferenceFrames, false, null);
   }

   public TypedNotification<WalkingStatusMessage> requestWalk(FootstepDataListMessage footstepPlan, HumanoidReferenceFrames humanoidReferenceFrames,
                                                              boolean swingOverPlanarRegions, PlanarRegionsList planarRegionsList)
   {

      if (swingOverPlanarRegions && planarRegionsList != null && decidePlanDistance(footstepPlan, humanoidReferenceFrames) == PlanTravelDistance.FAR)
      {
         footstepPlan = calculateSwingOverTrajectoryExpansions(footstepPlan, humanoidReferenceFrames, planarRegionsList);
      }

      LogTools.debug("Tasking {} footstep(s) to the robot", footstepPlan.getFootstepDataList().size());

      footstepDataListPublisher.publish(footstepPlan);

      TypedNotification<WalkingStatusMessage> walkingCompletedNotification = new TypedNotification<>();
      walkingCompletedNotifications.add(walkingCompletedNotification);
      return walkingCompletedNotification;
   }

   public void requestFootTrajectory(FootTrajectoryMessage message)
   {
      footTrajectoryMessagePublisher.publish(message);
   }

   public void requestArmTrajectory(ArmTrajectoryMessage message)
   {
      armTrajectoryMessagePublisher.publish(message);
   }

   public void requestChestOrientationTrajectory(ChestTrajectoryMessage message)
   {
      chestOrientationTrajectoryMessagePublisher.publish(message);
   }

   public void requestPelvisOrientationTrajectory(PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage)
   {
      pelvisOrientationTrajectoryMessagePublisher.publish(pelvisOrientationTrajectoryMessage);
   }

   public void requestPelvisTrajectory(PelvisTrajectoryMessage pelvisTrajectoryMessage)
   {
      pelvisTrajectoryMessagePublisher.publish(pelvisTrajectoryMessage);
   }

   public void requestGoHome(GoHomeMessage goHomeMessage)
   {
      goHomeMessagePublisher.publish(goHomeMessage);
   }

   private PlanTravelDistance decidePlanDistance(FootstepDataListMessage footstepPlan, HumanoidReferenceFrames humanoidReferenceFrames)
   {
      FramePose3D midFeetZUpPose = new FramePose3D();
      midFeetZUpPose.setFromReferenceFrame(humanoidReferenceFrames.getMidFeetZUpFrame());

      FootstepDataMessage footstep = footstepPlan.getFootstepDataList().get(footstepPlan.getFootstepDataList().size() - 1);

      double distance = midFeetZUpPose.getPositionDistance(footstep.getLocation());

      return distance < PlanTravelDistance.CLOSE_PLAN_RADIUS ? PlanTravelDistance.CLOSE : PlanTravelDistance.FAR;
   }

   private FootstepDataListMessage calculateSwingOverTrajectoryExpansions(FootstepDataListMessage footsteps, HumanoidReferenceFrames humanoidReferenceFrames,
                                                                          PlanarRegionsList planarRegionsList)
   {
      LogTools.debug("Calculating swing over planar regions...");

      double swingTime = 0.6;
      double transferTime = 0.25;
      double maxSwingSpeed = 1.0;
      FramePose3D stanceFootPose = new FramePose3D();
      FramePose3D swingStartPose = new FramePose3D();
      FramePose3D swingEndPose = new FramePose3D();

      RobotSide firstSwingFoot = RobotSide.fromByte(footsteps.getFootstepDataList().get(0).getRobotSide());
      stanceFootPose.setFromReferenceFrame(humanoidReferenceFrames.getSoleFrame(firstSwingFoot));
      swingEndPose.setFromReferenceFrame(humanoidReferenceFrames.getSoleFrame(firstSwingFoot.getOppositeSide())); // first stance foot

      int i = 0;
      for (FootstepDataMessage footstepData : footsteps.getFootstepDataList())
      {
         swingStartPose.set(stanceFootPose);
         stanceFootPose.set(swingEndPose);
         swingEndPose.set(footstepData.getLocation(), footstepData.getOrientation());

         double maxSpeedDimensionless = swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(stanceFootPose, swingStartPose, swingEndPose,
                                                                                                                   planarRegionsList);

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
