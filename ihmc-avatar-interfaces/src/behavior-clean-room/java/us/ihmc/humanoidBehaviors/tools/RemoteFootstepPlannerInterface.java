package us.ihmc.humanoidBehaviors.tools;

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;

import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.SettableFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.TypedNotification;

/**
 * Taken mostly from us.ihmc.footstepPlanning.ui.RemoteUIMessageConverter
 */
public class RemoteFootstepPlannerInterface
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FootstepPlannerParameters footstepPlannerParameters;

   private final IHMCROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final IHMCROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private final IHMCROS2Publisher<FootstepPlannerParametersPacket> parametersPublisher;

   private final AtomicInteger requestCounter = new AtomicInteger(1739);

   private final HashMap<Integer, TypedNotification<FootstepPlanningToolboxOutputStatus>> resultNotifications = new HashMap<>();

   public RemoteFootstepPlannerInterface(Ros2Node ros2Node, DRCRobotModel robotModel)
   {
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();

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
                         this::acceptFootstepPlanningStatus);
   }

   private void acceptFootstepPlanningStatus(FootstepPlanningToolboxOutputStatus footstepPlanningToolboxOutputStatus)
   {
      if (resultNotifications.containsKey(footstepPlanningToolboxOutputStatus.getPlanId()))
      {
         resultNotifications.remove(footstepPlanningToolboxOutputStatus.getPlanId()).add(footstepPlanningToolboxOutputStatus);
      }
   }

   public TypedNotification<FootstepPlanningToolboxOutputStatus> requestPlan(FramePose3D start,
                                                                             FramePose3D goal,
                                                                             PlanarRegionsListMessage planarRegionsListMessage)
   {
      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));  // This is necessary! - @dcalvert 190318

      SettableFootstepPlannerParameters settableFootstepPlannerParameters = new SettableFootstepPlannerParameters(footstepPlannerParameters);

      FootstepPlannerParametersPacket footstepPlannerParametersPacket = new FootstepPlannerParametersPacket();
      FootstepPlannerMessageTools.copyParametersToPacket(footstepPlannerParametersPacket, settableFootstepPlannerParameters);
      parametersPublisher.publish(footstepPlannerParametersPacket);

      double midFeetToSoleOffset = footstepPlannerParameters.getIdealFootstepWidth() / 2;
      start.changeFrame(worldFrame);
      start.appendTranslation(-midFeetToSoleOffset, 0.0, 0.0);

      goal.changeFrame(worldFrame);

      LogTools.debug("Planning from {}",
         start.getPosition().getX() + ", " + start.getPosition().getY() + ", yaw: " + start.getOrientation().getYaw()
            + " to "
            + goal.getPosition().getX() + ", " + goal.getPosition().getY() + ", yaw: " + goal.getOrientation().getYaw()
      );

      FootstepPlanningRequestPacket packet = new FootstepPlanningRequestPacket();
      packet.setInitialStanceRobotSide(RobotSide.LEFT.toByte());
      packet.getStanceFootPositionInWorld().set(start.getPosition());             // assuming start pose is left foot center
      packet.getStanceFootOrientationInWorld().set(start.getOrientation());
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

      TypedNotification<FootstepPlanningToolboxOutputStatus> resultNotification = new TypedNotification<>();
      resultNotifications.put(sentPlannerId, resultNotification);
      return resultNotification;
   }

   public void abortPlanning()
   {
      LogTools.debug("Sending SLEEP to footstep planner");
      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.SLEEP));
   }
}
