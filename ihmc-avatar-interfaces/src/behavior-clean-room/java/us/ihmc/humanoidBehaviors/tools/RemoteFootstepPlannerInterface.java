package us.ihmc.humanoidBehaviors.tools;

import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.Ros2QueuedSubscription;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;

import java.util.concurrent.atomic.AtomicInteger;

/**
 * Taken mostly from us.ihmc.footstepPlanning.ui.RemoteUIMessageConverter
 */
public class RemoteFootstepPlannerInterface
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FootstepPlannerParameters footstepPlannerParameters;

   private final IHMCROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final IHMCROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private final Ros2QueuedSubscription<FootstepPlanningToolboxOutputStatus> footstepPlannerResultQueue;

   private final AtomicInteger requestCounter = new AtomicInteger(1739);

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
      footstepPlannerResultQueue =
            ROS2Tools.createQueuedSubscription(ros2Node,
                                               FootstepPlanningToolboxOutputStatus.class,
                                               FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(robotModel.getSimpleRobotName()));
   }

   public FootstepPlanningToolboxOutputStatus requestPlanBlocking(FramePose3D start, FramePose3D goal, PlanarRegionsListMessage planarRegionsListMessage)
   {
      int sentPlannerId = requestPlan(start, goal, planarRegionsListMessage);

      LogTools.debug("Waiting for footstep plan result id {}...", sentPlannerId);

      FootstepPlanningToolboxOutputStatus footstepPlanningResult = new FootstepPlanningToolboxOutputStatus();
      boolean resultIdMatchesLastSent = false;
      while (!resultIdMatchesLastSent)           // wait for our result to arrive
      {
         boolean messageWasAvailable = footstepPlannerResultQueue.poll(footstepPlanningResult);
         if (messageWasAvailable && footstepPlanningResult.getPlanId() == sentPlannerId)
         {
            LogTools.debug("Received footstep plan result id {}", sentPlannerId);
            resultIdMatchesLastSent = true;
         }
         else
         {
            Thread.yield();
         }
      }

      return footstepPlanningResult;
   }

   public int requestPlan(FramePose3D start, FramePose3D goal, PlanarRegionsListMessage planarRegionsListMessage)
   {
      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));  // This is necessary! - @dcalvert 190318

      double idealFootstepWidth = footstepPlannerParameters.getIdealFootstepWidth() / 2;
      LogTools.debug("Ideal footstep width / 2: {}", idealFootstepWidth / 2);

      start.changeFrame(worldFrame);
      start.appendTranslation(-idealFootstepWidth, 0.0, 0.0);

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

      packet.setTimeout(120);
      packet.setRequestedFootstepPlannerType(FootstepPlanningRequestPacket.FOOTSTEP_PLANNER_TYPE_A_STAR);
      int sentPlannerId = requestCounter.getAndIncrement();
      packet.setPlannerRequestId(sentPlannerId);
      if (planarRegionsListMessage != null)
         packet.getPlanarRegionsListMessage().set(planarRegionsListMessage);
      else
         packet.setAssumeFlatGround(true);

      footstepPlanningRequestPublisher.publish(packet);

      return sentPlannerId;
   }
}
