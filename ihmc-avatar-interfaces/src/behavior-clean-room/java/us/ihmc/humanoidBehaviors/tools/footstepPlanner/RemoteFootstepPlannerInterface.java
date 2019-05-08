package us.ihmc.humanoidBehaviors.tools.footstepPlanner;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.SettableFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehaviorAPI;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.TypedNotification;

/**
 * Taken mostly from us.ihmc.footstepPlanning.ui.RemoteUIMessageConverter
 */
public class RemoteFootstepPlannerInterface
{
   private static final double CLOSE_PLAN_RADIUS = 1.0;

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final Messager messager;
   private volatile FootstepPlannerParameters footstepPlannerParameters;

   private final IHMCROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final IHMCROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private final IHMCROS2Publisher<FootstepPlannerParametersPacket> parametersPublisher;

   private final AtomicInteger requestCounter = new AtomicInteger(1739);

   private final HashMap<Integer, TypedNotification<RemoteFootstepPlannerResult>> resultNotifications = new HashMap<>();

   public enum PlanType { CLOSE, FAR }

   public RemoteFootstepPlannerInterface(Ros2Node ros2Node, DRCRobotModel robotModel, Messager messager)
   {
      this.messager = messager;
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      if (messager != null)
      {
         messager.registerTopicListener(PatrolBehaviorAPI.PlannerParameters, parameters -> // TODO this class should not use patrol specific API
         {
            SettableFootstepPlannerParameters settableFootstepPlannerParameters = new SettableFootstepPlannerParameters(footstepPlannerParameters);
            parameters.packFootstepPlannerParameters(settableFootstepPlannerParameters);
            footstepPlannerParameters = settableFootstepPlannerParameters;
         }); // updated from UI
      }

      toolboxStatePublisher =
            ROS2Tools.createPublisher(ros2Node,
                                      ToolboxStateMessage.class,
                                      FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotModel.getSimpleRobotName()));
      footstepPlanningRequestPublisher =
            ROS2Tools.createPublisher(ros2Node,
                                      FootstepPlanningRequestPacket.class,
                                      FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotModel.getSimpleRobotName()));
      parametersPublisher =
            ROS2Tools.createPublisher(ros2Node,
                                      FootstepPlannerParametersPacket.class,
                                      FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(robotModel.getSimpleRobotName()));

      new ROS2Callback<>(ros2Node,
                         FootstepPlanningToolboxOutputStatus.class,
                         robotModel.getSimpleRobotName(),
                         MultiStageFootstepPlanningModule.ROS2_ID,
                         this::acceptFootstepPlannerResult);
   }

   private void acceptFootstepPlannerResult(FootstepPlanningToolboxOutputStatus footstepPlanningToolboxOutputStatus)
   {
      if (resultNotifications.containsKey(footstepPlanningToolboxOutputStatus.getPlanId()))
      {
         resultNotifications.remove(footstepPlanningToolboxOutputStatus.getPlanId()).add(new RemoteFootstepPlannerResult(footstepPlanningToolboxOutputStatus));
      }
   }

   public TypedNotification<RemoteFootstepPlannerResult> requestPlan(FramePose3DReadOnly start,
                                                                             FramePose3DReadOnly goal,
                                                                             PlanarRegionsListMessage planarRegionsListMessage)
   {
      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));  // This is necessary! - @dcalvert 190318

      SettableFootstepPlannerParameters settableFootstepPlannerParameters = new SettableFootstepPlannerParameters(footstepPlannerParameters);
      if (decidePlanType(start, goal) == PlanType.CLOSE)
      {
         settableFootstepPlannerParameters.setMaximumStepYaw(1.5); // enable quick turn arounds
      }

      FootstepPlannerParametersPacket footstepPlannerParametersPacket = new FootstepPlannerParametersPacket();
      FootstepPlannerMessageTools.copyParametersToPacket(footstepPlannerParametersPacket, settableFootstepPlannerParameters);
      parametersPublisher.publish(footstepPlannerParametersPacket);

      FramePose3D soleStart = new FramePose3D(start);
      double midFeetToSoleOffset = footstepPlannerParameters.getIdealFootstepWidth() / 2;
      soleStart.appendTranslation(-midFeetToSoleOffset, 0.0, 0.0);

      LogTools.debug("Planning from {}",
            soleStart.getPosition().getX() + ", " + soleStart.getPosition().getY() + ", yaw: " + soleStart.getOrientation().getYaw()
            + " to "
            + goal.getPosition().getX() + ", " + goal.getPosition().getY() + ", yaw: " + goal.getOrientation().getYaw()
      );

      FootstepPlanningRequestPacket packet = new FootstepPlanningRequestPacket();
      packet.setInitialStanceRobotSide(RobotSide.LEFT.toByte());
      packet.getStanceFootPositionInWorld().set(soleStart.getPosition());             // assuming start pose is left foot center
      packet.getStanceFootOrientationInWorld().set(soleStart.getOrientation());
      packet.getGoalPositionInWorld().set(goal.getPosition());                    // assuming goal position specified in mid feet z up
      packet.getGoalOrientationInWorld().set(goal.getOrientation());

      packet.setTimeout(5);
      packet.setRequestedFootstepPlannerType(FootstepPlanningRequestPacket.FOOTSTEP_PLANNER_TYPE_A_STAR);
      int sentPlannerId = requestCounter.getAndIncrement();
      packet.setPlannerRequestId(sentPlannerId);
      if (planarRegionsListMessage != null)
         packet.getPlanarRegionsListMessage().set(planarRegionsListMessage);
      else
         packet.setAssumeFlatGround(true);

      footstepPlanningRequestPublisher.publish(packet);

      TypedNotification<RemoteFootstepPlannerResult> resultNotification = new TypedNotification<>();
      resultNotifications.put(sentPlannerId, resultNotification);
      return resultNotification;
   }

   public void abortPlanning()
   {
      LogTools.debug("Sending SLEEP to footstep planner");
      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }

   public PlanType decidePlanType(Pose3DReadOnly start, Pose3DReadOnly goal)
   {
      return start.getPositionDistance(goal) < 1.0 ? PlanType.CLOSE : PlanType.FAR;
   }
}
