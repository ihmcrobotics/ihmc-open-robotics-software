package us.ihmc.humanoidBehaviors.tools.footstepPlanner;

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

import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Taken mostly from us.ihmc.footstepPlanning.ui.RemoteUIMessageConverter
 */
public class RemoteFootstepPlannerInterface
{
   public static final double STEP_DOWN_DISTANCE_QUALIFIER = 0.05;

   public static final double DEFAULT_TIMEOUT = 12.0;
   public static final double DEFAULT_PERCEIVE_TIME_REQUIRED = 26.0;
   public static final double DEFAULT_TRANSFER_TIME_FLAT_UP = 0.8;
   public static final double DEFAULT_TRANSFER_TIME_DOWN    = 2.0;
   public static final double DEFAULT_SWING_TIME_FLAT_UP    = 1.2;
   public static final double DEFAULT_SWING_TIME_DOWN       = 1.2;

   private volatile FootstepPlannerParameters footstepPlannerParameters;
   private volatile double timeout = DEFAULT_TIMEOUT;
   private volatile double transferTimeFlatUp = DEFAULT_TRANSFER_TIME_FLAT_UP;
   private volatile double transferTimeDown   = DEFAULT_TRANSFER_TIME_DOWN   ;
   private volatile double swingTimeFlatUp    = DEFAULT_SWING_TIME_FLAT_UP   ;
   private volatile double swingTimeDown      = DEFAULT_SWING_TIME_DOWN      ;

   private final IHMCROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final IHMCROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private final IHMCROS2Publisher<FootstepPlannerParametersPacket> parametersPublisher;

   private final AtomicInteger requestCounter = new AtomicInteger(1739);

   private final HashMap<Integer, TypedNotification<RemoteFootstepPlannerResult>> resultNotifications = new HashMap<>();

   public RemoteFootstepPlannerInterface(Ros2Node ros2Node, DRCRobotModel robotModel, Messager messager)
   {
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      if (messager != null)
      {
         messager.registerTopicListener(PatrolBehaviorAPI.PlannerParameters, parameters -> // TODO this class should not use patrol specific API
         {
            SettableFootstepPlannerParameters settableFootstepPlannerParameters = new SettableFootstepPlannerParameters(footstepPlannerParameters);
            parameters.packFootstepPlannerParameters(settableFootstepPlannerParameters);
            footstepPlannerParameters = settableFootstepPlannerParameters;
            timeout = parameters.getTimeout();
            transferTimeFlatUp = parameters.getTransferTimeFlatUp();
            transferTimeDown   = parameters.getTransferTimeDown  ();
            swingTimeFlatUp    = parameters.getSwingTimeFlatUp   ();
            swingTimeDown      = parameters.getSwingTimeDown     ();
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
         RemoteFootstepPlannerResult result = new RemoteFootstepPlannerResult(footstepPlanningToolboxOutputStatus);

         result.getFootstepDataListMessage().setDefaultSwingDuration(swingTimeFlatUp);
         result.getFootstepDataListMessage().setDefaultTransferDuration(transferTimeFlatUp);

         if (result.isValidForExecution() && result.getFootstepPlan().getNumberOfSteps() > 1)
         {
            boolean containsStepDown = false;
            int index = 0;
            double lastStepZ = result.getFootstepPlan().getFootstep(index).getSoleFramePose().getZ();
            do
            {
               ++index;
               double thisStepZ = result.getFootstepPlan().getFootstep(index).getSoleFramePose().getZ();
               double difference = thisStepZ - lastStepZ;
               if (difference <= -STEP_DOWN_DISTANCE_QUALIFIER)
               {
                  LogTools.info("Detected downward plan: thisX: {} <= lastZ: {}", thisStepZ, lastStepZ);
                  containsStepDown = true;
               }
               lastStepZ = thisStepZ;
            }
            while (!containsStepDown && index < result.getFootstepPlan().getNumberOfSteps() - 1);

            if (containsStepDown)
            {
               result.getFootstepDataListMessage().setDefaultSwingDuration(swingTimeDown);
               result.getFootstepDataListMessage().setDefaultTransferDuration(transferTimeDown);
            }
         }

         resultNotifications.remove(footstepPlanningToolboxOutputStatus.getPlanId()).add(result);
      }
   }

   public TypedNotification<RemoteFootstepPlannerResult> requestPlan(FramePose3DReadOnly start,
                                                                     FramePose3DReadOnly goal,
                                                                     PlanarRegionsListMessage planarRegionsListMessage)
   {
      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));  // This is necessary! - @dcalvert 190318

      SettableFootstepPlannerParameters settableFootstepPlannerParameters = new SettableFootstepPlannerParameters(footstepPlannerParameters);
      if (decidePlanType(start, goal) == PlanTravelDistance.CLOSE)
      {
         settableFootstepPlannerParameters.setMaximumStepYaw(1.1); // enable quick turn arounds
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

      packet.setTimeout(timeout);
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

   public PlanTravelDistance decidePlanType(Pose3DReadOnly start, Pose3DReadOnly goal)
   {
      return start.getPositionDistance(goal) < PlanTravelDistance.CLOSE_PLAN_RADIUS ? PlanTravelDistance.CLOSE : PlanTravelDistance.FAR;
   }
}
