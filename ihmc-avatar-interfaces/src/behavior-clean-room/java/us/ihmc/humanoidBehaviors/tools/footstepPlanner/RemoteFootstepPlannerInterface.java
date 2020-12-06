package us.ihmc.humanoidBehaviors.tools.footstepPlanner;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehaviorAPI;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2NodeInterface;

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

   private volatile FootstepPlannerParametersReadOnly footstepPlannerParameters;
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

   public RemoteFootstepPlannerInterface(ROS2NodeInterface ros2Node, DRCRobotModel robotModel, Messager messager)
   {
      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      if (messager != null)
      {
         messager.registerTopicListener(PatrolBehaviorAPI.PlannerParameters, parameters -> // TODO this class should not use patrol specific API
         {
            DefaultFootstepPlannerParameters settableFootstepPlannerParameters = new DefaultFootstepPlannerParameters(); // TODO: This might not be thread safe
            parameters.packFootstepPlannerParameters(settableFootstepPlannerParameters);              // TODO: Clean this up  - @dcalvert
            footstepPlannerParameters = settableFootstepPlannerParameters;
            timeout = parameters.getTimeout();
            transferTimeFlatUp = parameters.getTransferTimeFlatUp();
            transferTimeDown   = parameters.getTransferTimeDown  ();
            swingTimeFlatUp    = parameters.getSwingTimeFlatUp   ();
            swingTimeDown      = parameters.getSwingTimeDown     ();
         }); // updated from UI
      }

      toolboxStatePublisher =
            ROS2Tools.createPublisherTypeNamed(ros2Node,
                                               ToolboxStateMessage.class,
                                               FootstepPlannerCommunicationProperties.inputTopic(robotModel.getSimpleRobotName()));
      footstepPlanningRequestPublisher =
            ROS2Tools.createPublisherTypeNamed(ros2Node,
                                               FootstepPlanningRequestPacket.class,
                                               FootstepPlannerCommunicationProperties.inputTopic(robotModel.getSimpleRobotName()));
      parametersPublisher =
            ROS2Tools.createPublisherTypeNamed(ros2Node,
                                               FootstepPlannerParametersPacket.class,
                                               FootstepPlannerCommunicationProperties.inputTopic(robotModel.getSimpleRobotName()));

      new ROS2Callback<>(ros2Node,
                         FootstepPlanningToolboxOutputStatus.class,
                         ROS2Tools.FOOTSTEP_PLANNER.withRobot(robotModel.getSimpleRobotName()).withOutput(),
                         this::acceptFootstepPlannerResult);
   }

   private void acceptFootstepPlannerResult(FootstepPlanningToolboxOutputStatus footstepPlanningToolboxOutputStatus)
   {
      FootstepPlanningResult footstepPlanningResult = FootstepPlanningResult.fromByte(footstepPlanningToolboxOutputStatus.getFootstepPlanningResult());
      if (resultNotifications.containsKey(footstepPlanningToolboxOutputStatus.getPlanId()) && footstepPlanningResult != FootstepPlanningResult.PLANNING)
      {
         RemoteFootstepPlannerResult result = new RemoteFootstepPlannerResult(footstepPlanningToolboxOutputStatus);

         result.getFootstepDataListMessage().setDefaultSwingDuration(swingTimeFlatUp);
         result.getFootstepDataListMessage().setDefaultTransferDuration(transferTimeFlatUp);

         if (result.isValidForExecution() && result.getFootstepPlan().getNumberOfSteps() > 1)
         {
            boolean containsStepDown = false;
            int index = 0;
            double lastStepZ = result.getFootstepPlan().getFootstep(index).getFootstepPose().getZ();
            do
            {
               ++index;
               double thisStepZ = result.getFootstepPlan().getFootstep(index).getFootstepPose().getZ();
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

         resultNotifications.remove(footstepPlanningToolboxOutputStatus.getPlanId()).set(result);
      }
   }

   /**
    * Assume flat ground.
    */
   public TypedNotification<RemoteFootstepPlannerResult> requestPlan(FramePose3DReadOnly start, FramePose3DReadOnly goal)
   {
      return requestPlan(start, goal, null, new DefaultFootstepPlannerParameters(), SwingPlannerType.NONE);
   }

   public TypedNotification<RemoteFootstepPlannerResult> requestPlan(FramePose3DReadOnly start, FramePose3DReadOnly goal, PlanarRegionsList planarRegionsList)
   {
      return requestPlan(start,
                         goal,
                         PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList),
                         new DefaultFootstepPlannerParameters(),
                         SwingPlannerType.NONE);
   }

   public TypedNotification<RemoteFootstepPlannerResult> requestPlan(FramePose3DReadOnly start,
                                                                     FramePose3DReadOnly goal,
                                                                     PlanarRegionsListMessage planarRegionsListMessage,
                                                                     FootstepPlannerParametersBasics settableFootstepPlannerParameters,
                                                                     SwingPlannerType swingPlannerType)
   {
      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));  // This is necessary! - @dcalvert 190318

      FootstepPlannerParametersPacket footstepPlannerParametersPacket = new FootstepPlannerParametersPacket();
      FootstepPlannerMessageTools.copyParametersToPacket(footstepPlannerParametersPacket, settableFootstepPlannerParameters);
      parametersPublisher.publish(footstepPlannerParametersPacket);

      RobotSide initialStanceSide = RobotSide.LEFT;
      SideDependentList<Pose3D> startSteps = PlannerTools.createSquaredUpFootsteps(start, footstepPlannerParameters.getIdealFootstepWidth());
      SideDependentList<Pose3D> goalSteps = PlannerTools.createSquaredUpFootsteps(goal, footstepPlannerParameters.getIdealFootstepWidth());

      Pose3D initialStanceFoot = startSteps.get(initialStanceSide);
      LogTools.debug("Planning from {}",
                     initialStanceFoot.getX() + ", " + initialStanceFoot.getY() + ", yaw: " + initialStanceFoot.getOrientation().getYaw() + " to "
                     + goal.getPosition().getX() + ", " + goal.getPosition().getY() + ", yaw: " + goal.getOrientation().getYaw());

      FootstepPlanningRequestPacket packet = new FootstepPlanningRequestPacket();

      boolean planBodyPath = false;
      boolean performAStarSearch = true;
      packet.setPlanBodyPath(planBodyPath);
      packet.setPerformAStarSearch(performAStarSearch);

      packet.setRequestedInitialStanceSide(initialStanceSide.toByte());
      packet.getStartLeftFootPose().set(startSteps.get(RobotSide.LEFT));
      packet.getStartRightFootPose().set(startSteps.get(RobotSide.RIGHT));
      packet.getGoalLeftFootPose().set(goalSteps.get(RobotSide.LEFT));
      packet.getGoalRightFootPose().set(goalSteps.get(RobotSide.RIGHT));
      packet.setRequestedSwingPlanner(swingPlannerType.toByte());

      packet.setTimeout(timeout);
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
}
