package us.ihmc.footstepPlanning.ui;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.HeadTrajectoryMessage;
import controller_msgs.msg.dds.NeckTrajectoryMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.SpineTrajectoryMessage;
import org.apache.commons.lang3.tuple.Pair;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.TerrainMapMessage;
import toolbox_msgs.msg.dds.FootstepPlannerActionMessage;
import toolbox_msgs.msg.dds.FootstepPlannerParametersPacket;
import toolbox_msgs.msg.dds.FootstepPlanningRequestPacket;
import toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import toolbox_msgs.msg.dds.SwingPlannerParametersPacket;
import toolbox_msgs.msg.dds.SwingPlanningRequestPacket;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
import toolbox_msgs.msg.dds.VisibilityGraphsParametersPacket;
import toolbox_msgs.msg.dds.WalkingControllerPreviewInputMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.FootstepPlannerAPI;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.BodyPathPlanningResult;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerRequestedAction;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * This class is required when using a local version of the Footstep Planner UI and the footstep planning algorithms
 * located in the Footstep Planner Toolbox. It allows users to view the resulting plans calculated by the toolbox. It
 * also allows the user to tune the planner parameters, and request a new plan from the planning toolbox.
 *
 * This class is used to convert the shared memory messages from Java FX used by the Footstep Planner UI
 * to RTPS messages, and vice-versa. It specifically listens to the planning request messages and the output
 * status messages sent to and from the toolbox, and visualizes those. It also allows the user to request a
 * new plan using the local version of the edit footstep planning parameters and a footstep request, requiring
 * conversion from the Java FX messages to the RTPS messages.
 */
public class RemoteUIMessageConverter
{
   private static final boolean verbose = false;

   private final RealtimeROS2Node ros2Node;

   private final Messager messager;

   private final String robotName;

   private final AtomicReference<DefaultFootstepPlannerParametersReadOnly> plannerParametersReference;
   private final AtomicReference<SwingPlannerParametersReadOnly> swingPlannerParametersReference;
   private final AtomicReference<Pose3DReadOnly> leftFootPose;
   private final AtomicReference<Pose3DReadOnly> rightFootPose;
   private final AtomicReference<Pose3DReadOnly> goalLeftFootPose;
   private final AtomicReference<Pose3DReadOnly> goalRightFootPose;
   private final AtomicReference<Boolean> snapGoalSteps;
   private final AtomicReference<Boolean> abortIfGoalStepSnapFails;
   private final AtomicReference<TerrainMapMessage> terrainMapReference;
   private final AtomicReference<Boolean> planBodyPath;
   private final AtomicReference<Boolean> performAStarSearch;
   private final AtomicReference<SwingPlannerType> requestedSwingPlanner;
   private final AtomicReference<Double> plannerTimeoutReference;
   private final AtomicReference<Integer> maxIterations;
   private final AtomicReference<RobotSide> plannerInitialSupportSideReference;
   private final AtomicReference<Integer> plannerRequestIdReference;
   private final AtomicReference<Double> plannerHorizonLengthReference;
   private final AtomicReference<Boolean> acceptNewPlanarRegionsReference;
   private final AtomicReference<Integer> currentPlanRequestId;
   private final AtomicReference<Boolean> assumeFlatGround;
   private final AtomicReference<Double> goalDistanceProximity;
   private final AtomicReference<Double> goalYawProximity;
   private final AtomicReference<FootstepPlan> referencePlan;

   private final AtomicReference<ConvexPolygon2D> postProcessingLeftFootSupportPolygonReference;
   private final AtomicReference<ConvexPolygon2D> postProcessingRightFootSupportPolygonReference;
   private final AtomicReference<FootstepDataListMessage> footstepPlanResponseReference;
   private final AtomicReference<PlanarRegionsList> planarRegionListReference;

   private ROS2Publisher<FootstepPlannerActionMessage> plannerActionPublisher;
   private ROS2Publisher<VisibilityGraphsParametersPacket> visibilityGraphsParametersPublisher;
   private ROS2Publisher<FootstepPlannerParametersPacket> plannerParametersPublisher;
   private ROS2Publisher<SwingPlannerParametersPacket> swingPlannerParametersPublisher;
   private ROS2Publisher<SwingPlanningRequestPacket> swingReplanRequestPublisher;

   private ROS2Publisher<FootstepPlanningRequestPacket> footstepPlanningRequestPublisher;
   private ROS2Publisher<FootstepDataListMessage> footstepDataListPublisher;
   private ROS2Publisher<GoHomeMessage> goHomePublisher;
   private ROS2Publisher<ToolboxStateMessage> walkingPreviewToolboxStatePublisher;
   private ROS2Publisher<WalkingControllerPreviewInputMessage> walkingPreviewRequestPublisher;

   private ROS2Publisher<ArmTrajectoryMessage> armTrajectoryMessagePublisher;
   private ROS2Publisher<HandTrajectoryMessage> handTrajectoryMessagePublisher;
   private ROS2Publisher<FootTrajectoryMessage> footTrajectoryMessagePublisher;
   private ROS2Publisher<ChestTrajectoryMessage> chestTrajectoryMessagePublisher;
   private ROS2Publisher<SpineTrajectoryMessage> spineTrajectoryMessagePublisher;
   private ROS2Publisher<HeadTrajectoryMessage> headTrajectoryMessagePublisher;
   private ROS2Publisher<NeckTrajectoryMessage> neckTrajectoryMessagePublisher;

   public static RemoteUIMessageConverter createConverter(Messager messager, String robotName)
   {
      RealtimeROS2Node ros2Node = new ROS2NodeBuilder().buildRealtime("ihmc_footstep_planner_ui");
      return new RemoteUIMessageConverter(ros2Node, messager, robotName);
   }

   public RemoteUIMessageConverter(RealtimeROS2Node ros2Node, Messager messager, String robotName)
   {
      this.messager = messager;
      this.robotName = robotName;
      this.ros2Node = ros2Node;

      plannerParametersReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerParameters, null);
      swingPlannerParametersReference = messager.createInput(FootstepPlannerMessagerAPI.SwingPlannerParameters, null);

      leftFootPose = messager.createInput(FootstepPlannerMessagerAPI.LeftFootPose);
      rightFootPose = messager.createInput(FootstepPlannerMessagerAPI.RightFootPose);
      goalLeftFootPose = messager.createInput(FootstepPlannerMessagerAPI.LeftFootGoalPose);
      goalRightFootPose = messager.createInput(FootstepPlannerMessagerAPI.RightFootGoalPose);
      snapGoalSteps = messager.createInput(FootstepPlannerMessagerAPI.SnapGoalSteps);
      abortIfGoalStepSnapFails = messager.createInput(FootstepPlannerMessagerAPI.AbortIfGoalStepSnapFails);
      terrainMapReference = messager.createInput(FootstepPlannerMessagerAPI.terrainMapMessage);
      planBodyPath = messager.createInput(FootstepPlannerMessagerAPI.PlanBodyPath, false);
      performAStarSearch = messager.createInput(FootstepPlannerMessagerAPI.PerformAStarSearch, true);
      requestedSwingPlanner = messager.createInput(FootstepPlannerMessagerAPI.RequestedSwingPlannerType, SwingPlannerType.NONE);
      plannerTimeoutReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerTimeout, 5.0);
      maxIterations = messager.createInput(FootstepPlannerMessagerAPI.MaxIterations, -1);
      plannerInitialSupportSideReference = messager.createInput(FootstepPlannerMessagerAPI.InitialSupportSide, RobotSide.LEFT);
      plannerRequestIdReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerRequestId);
      plannerHorizonLengthReference = messager.createInput(FootstepPlannerMessagerAPI.PlannerHorizonLength);
      acceptNewPlanarRegionsReference = messager.createInput(FootstepPlannerMessagerAPI.AcceptNewPlanarRegions, true);
      currentPlanRequestId = messager.createInput(FootstepPlannerMessagerAPI.PlannerRequestId, 0);
      assumeFlatGround = messager.createInput(FootstepPlannerMessagerAPI.AssumeFlatGround, false);
      goalDistanceProximity = messager.createInput(FootstepPlannerMessagerAPI.GoalDistanceProximity, 0.0);
      goalYawProximity = messager.createInput(FootstepPlannerMessagerAPI.GoalYawProximity, 0.0);
      referencePlan = messager.createInput(FootstepPlannerMessagerAPI.ReferencePlan, null);

      postProcessingLeftFootSupportPolygonReference = messager.createInput(FootstepPlannerMessagerAPI.LeftFootStartSupportPolygon, null);
      postProcessingRightFootSupportPolygonReference = messager.createInput(FootstepPlannerMessagerAPI.RightFootStartSupportPolygon, null);
      footstepPlanResponseReference = messager.createInput(FootstepPlannerMessagerAPI.FootstepPlanResponse, null);
      planarRegionListReference = messager.createInput(FootstepPlannerMessagerAPI.PlanarRegionData, null);

      registerPubSubs(ros2Node);

      ros2Node.spin();
   }

   public void destroy()
   {
      ros2Node.destroy();
   }

   private void registerPubSubs(RealtimeROS2Node ros2Node)
   {
      /* footstep planner module outgoing messages */
      ros2Node.createSubscription(FootstepPlannerAPI.inputTopic(robotName).withTypeName(FootstepPlanningRequestPacket.class),
                                  s -> processFootstepPlanningRequestPacket(s.takeNextData()));
      ros2Node.createSubscription(FootstepPlannerAPI.outputTopic(robotName).withTypeName(FootstepPlanningToolboxOutputStatus.class),
                                  s -> processFootstepPlanningOutputStatus(s.takeNextData()));
      ros2Node.createSubscription(FootstepPlannerAPI.swingReplanOutputTopic(robotName), subscriber ->
      {
         LogTools.info("Received replanned swing");
         messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanResponse, subscriber.takeNextData());
      });

      /* controller messages */
      ros2Node.createSubscription(HumanoidControllerAPI.getOutputTopic(robotName).withTypeName(RobotConfigurationData.class),
                                  s -> messager.submitMessage(FootstepPlannerMessagerAPI.RobotConfigurationData, s.takeNextData()));
      ros2Node.createSubscription(HumanoidControllerAPI.getOutputTopic(robotName).withTypeName(CapturabilityBasedStatus.class), s -> processCapturabilityStatus(s.takeNextData()));
      ros2Node.createSubscription(HumanoidControllerAPI.getOutputTopic(robotName).withTypeName(FootstepStatusMessage.class),
                                  s -> messager.submitMessage(FootstepPlannerMessagerAPI.FootstepStatusMessage, s.takeNextData()));

      /* publishers */
      plannerParametersPublisher = ros2Node.createPublisher(FootstepPlannerAPI.inputTopic(robotName).withTypeName(FootstepPlannerParametersPacket.class));
      visibilityGraphsParametersPublisher = ros2Node.createPublisher(FootstepPlannerAPI.inputTopic(robotName).withTypeName(VisibilityGraphsParametersPacket.class));
      plannerActionPublisher = ros2Node.createPublisher(FootstepPlannerAPI.inputTopic(robotName).withTypeName(FootstepPlannerActionMessage.class));
      swingPlannerParametersPublisher = ros2Node.createPublisher(FootstepPlannerAPI.FOOTSTEP_PLANNER.withRobot(robotName).withInput().withTypeName(SwingPlannerParametersPacket.class));
      swingReplanRequestPublisher = ros2Node.createPublisher(FootstepPlannerAPI.inputTopic(robotName).withTypeName(SwingPlanningRequestPacket.class));

      footstepPlanningRequestPublisher = ros2Node.createPublisher(FootstepPlannerAPI.inputTopic(robotName).withTypeName(FootstepPlanningRequestPacket.class));
      footstepDataListPublisher = ros2Node.createPublisher(HumanoidControllerAPI.getInputTopic(robotName).withTypeName(FootstepDataListMessage.class));
      goHomePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getInputTopic(robotName).withTypeName(GoHomeMessage.class));

      ROS2Topic controllerPreviewInputTopic = ToolboxAPIs.WALKING_PREVIEW_TOOLBOX.withRobot(robotName).withInput();
      walkingPreviewToolboxStatePublisher = ros2Node.createPublisher(controllerPreviewInputTopic.withTypeName(ToolboxStateMessage.class));
      walkingPreviewRequestPublisher = ros2Node.createPublisher(controllerPreviewInputTopic.withTypeName(WalkingControllerPreviewInputMessage.class));
      armTrajectoryMessagePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getInputTopic(robotName).withTypeName(ArmTrajectoryMessage.class));
      handTrajectoryMessagePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getInputTopic(robotName).withTypeName(HandTrajectoryMessage.class));
      footTrajectoryMessagePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getInputTopic(robotName).withTypeName(FootTrajectoryMessage.class));
      chestTrajectoryMessagePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getInputTopic(robotName).withTypeName(ChestTrajectoryMessage.class));
      spineTrajectoryMessagePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getInputTopic(robotName).withTypeName(SpineTrajectoryMessage.class));
      headTrajectoryMessagePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getInputTopic(robotName).withTypeName(HeadTrajectoryMessage.class));
      neckTrajectoryMessagePublisher = ros2Node.createPublisher(HumanoidControllerAPI.getInputTopic(robotName).withTypeName(NeckTrajectoryMessage.class));

      messager.addTopicListener(FootstepPlannerMessagerAPI.ComputePath, request -> requestNewPlan());
      messager.addTopicListener(FootstepPlannerMessagerAPI.ReplanSwing, request ->
      {
         if (requestedSwingPlanner.get() != SwingPlannerType.NONE)
         {
            SwingPlanningRequestPacket swingPlanningRequestPacket = new SwingPlanningRequestPacket();
            swingPlanningRequestPacket.setRequestedSwingPlanner(requestedSwingPlanner.get().toByte());
            swingReplanRequestPublisher.publish(swingPlanningRequestPacket);
         }
      });

      messager.addTopicListener(FootstepPlannerMessagerAPI.HaltPlanning, request -> requestHaltPlanning());
      messager.addTopicListener(FootstepPlannerMessagerAPI.GoHomeTopic, goHomePublisher::publish);
      messager.addTopicListener(FootstepPlannerMessagerAPI.FootstepPlanToRobot, footstepDataListPublisher::publish);

      messager.addTopicListener(FootstepPlannerMessagerAPI.RequestedArmJointAngles, request ->
      {
         RobotSide robotSide = request.getKey();
         double[] jointAngles = request.getValue();
         double trajectoryTime = 4.0;

         ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryTime, jointAngles);
         armTrajectoryMessagePublisher.publish(armTrajectoryMessage);

      });

      messager.addTopicListener(FootstepPlannerMessagerAPI.ArmTrajectoryMessageTopic, armTrajectoryMessagePublisher::publish);
      messager.addTopicListener(FootstepPlannerMessagerAPI.HandTrajectoryMessageTopic, handTrajectoryMessagePublisher::publish);
      messager.addTopicListener(FootstepPlannerMessagerAPI.FootTrajectoryMessageTopic, footTrajectoryMessagePublisher::publish);
      messager.addTopicListener(FootstepPlannerMessagerAPI.ChestTrajectoryMessageTopic, chestTrajectoryMessagePublisher::publish);
      messager.addTopicListener(FootstepPlannerMessagerAPI.SpineTrajectoryMessageTopic, spineTrajectoryMessagePublisher::publish);
      messager.addTopicListener(FootstepPlannerMessagerAPI.HeadTrajectoryMessageTopic, headTrajectoryMessagePublisher::publish);
      messager.addTopicListener(FootstepPlannerMessagerAPI.NeckTrajectoryMessageTopic, neckTrajectoryMessagePublisher::publish);
   }

   private void processFootstepPlanningRequestPacket(FootstepPlanningRequestPacket packet)
   {
      if (verbose)
         LogTools.info("Received a planning request.");

      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootPose, packet.getStartLeftFootPose());
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootPose, packet.getStartRightFootPose());
      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootGoalPose, packet.getGoalLeftFootPose());
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootGoalPose, packet.getGoalRightFootPose());
      RobotSide initialSupportSide = RobotSide.fromByte(packet.getRequestedInitialStanceSide());
      int plannerRequestId = packet.getPlannerRequestId();

      double timeout = packet.getTimeout();
      double horizonLength = packet.getHorizonLength();

      messager.submitMessage(FootstepPlannerMessagerAPI.PerformAStarSearch, packet.getPerformAStarSearch());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanBodyPath, packet.getPlanBodyPath());
      messager.submitMessage(FootstepPlannerMessagerAPI.RequestedSwingPlannerType, SwingPlannerType.fromByte(packet.getRequestedSwingPlanner()));

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeout, timeout);
      messager.submitMessage(FootstepPlannerMessagerAPI.MaxIterations, packet.getMaxIterations());
      messager.submitMessage(FootstepPlannerMessagerAPI.InitialSupportSide, initialSupportSide);
      messager.submitMessage(FootstepPlannerMessagerAPI.SnapGoalSteps, packet.getSnapGoalSteps());
      messager.submitMessage(FootstepPlannerMessagerAPI.AbortIfGoalStepSnapFails, packet.getAbortIfGoalStepSnappingFails());

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerRequestId, plannerRequestId);

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerHorizonLength, horizonLength);
   }

   private void processFootstepPlanningOutputStatus(FootstepPlanningToolboxOutputStatus packet)
   {
      FootstepDataListMessage footstepDataListMessage = packet.getFootstepDataList();
      int plannerRequestId = packet.getPlanId();
      BodyPathPlanningResult bodyPathPlanningResult = BodyPathPlanningResult.fromByte(packet.getBodyPathPlanningResult());
      FootstepPlanningResult footstepPlanningResult = FootstepPlanningResult.fromByte(packet.getFootstepPlanningResult());
      List<? extends Pose3DReadOnly> bodyPath = packet.getBodyPath();
      List<? extends Point3D> bodyPathUnsmoothed = packet.getBodyPathUnsmoothed();
      Pose3D lowLevelGoal = packet.getGoalPose();

      if (plannerRequestId > currentPlanRequestId.get())
         messager.submitMessage(FootstepPlannerMessagerAPI.PlannerRequestId, plannerRequestId);

      ThreadTools.sleep(100);

      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanResponse, footstepDataListMessage);
      messager.submitMessage(FootstepPlannerMessagerAPI.ReceivedPlanId, plannerRequestId);
      messager.submitMessage(FootstepPlannerMessagerAPI.BodyPathPlanningResultTopic, bodyPathPlanningResult);
      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanningResultTopic, footstepPlanningResult);
      messager.submitMessage(FootstepPlannerMessagerAPI.BodyPathData, Pair.of(bodyPath, bodyPathUnsmoothed));

      if (lowLevelGoal != null)
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.LowLevelGoalPosition, lowLevelGoal.getPosition());
         messager.submitMessage(FootstepPlannerMessagerAPI.LowLevelGoalOrientation, lowLevelGoal.getOrientation());
      }
      if (footstepPlanningResult == FootstepPlanningResult.EXCEPTION)
      {
         StringBuilder stackTrace = new StringBuilder();
         stackTrace.append(packet.getExceptionMessage()).append("\n");
         for (int i = 0; i < packet.getStacktrace().size(); i++)
         {
            stackTrace.append(packet.getStacktrace().get(i)).append("\n");
         }
         messager.submitMessage(FootstepPlannerMessagerAPI.PlannerExceptionStackTrace, stackTrace.toString());
      }
      else
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.PlannerExceptionStackTrace,
                                "No stack trace available, planner status wasn't " + FootstepPlanningResult.EXCEPTION + ", it was: " + footstepPlanningResult);
      }

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimings, packet.getPlannerTimings());

      if (verbose)
         LogTools.info("Received a footstep planning result from planner module.");
   }

   private void processIncomingPlanarRegionMessage(PlanarRegionsListMessage packet)
   {
      if (acceptNewPlanarRegionsReference.get())
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, PlanarRegionMessageConverter.convertToPlanarRegionsList(packet));

         if (verbose)
            LogTools.info("Received updated planner regions.");
      }
   }

   private void processCapturabilityStatus(CapturabilityBasedStatus packet)
   {
      ConvexPolygon2D leftFootPolygon = new ConvexPolygon2D();
      ConvexPolygon2D rightFootPolygon = new ConvexPolygon2D();
      packet.getLeftFootSupportPolygon3d().forEach(leftFootPolygon::addVertex);
      packet.getRightFootSupportPolygon3d().forEach(rightFootPolygon::addVertex);
      leftFootPolygon.update();
      rightFootPolygon.update();

      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootStartSupportPolygon, leftFootPolygon);
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootStartSupportPolygon, rightFootPolygon);
   }

   private void requestNewPlan()
   {
      if (!checkRequireds())
      {
         return;
      }

      if (verbose)
         LogTools.info("Told the toolbox to wake up.");

      DefaultFootstepPlannerParametersReadOnly footstepPlannerParameters = plannerParametersReference.get();
      if(footstepPlannerParameters != null)
      {
         FootstepPlannerParametersPacket plannerParametersPacket = new FootstepPlannerParametersPacket();
         FootstepPlannerMessageTools.copyParametersToPacket(plannerParametersPacket, footstepPlannerParameters);
         plannerParametersPublisher.publish(plannerParametersPacket);
      }

      SwingPlannerParametersReadOnly swingPlannerParameters = swingPlannerParametersReference.get();
      if (swingPlannerParameters != null)
      {
         swingPlannerParametersPublisher.publish(swingPlannerParameters.getAsPacket());
      }

      if (verbose)
         LogTools.info("Sent out some parameters");

      submitFootstepPlanningRequestPacket();
   }

   private boolean checkRequireds()
   {
      String errorMessage = "";

      if (leftFootPose.get() == null || rightFootPose.get() == null)
         errorMessage += "Need to set start poses.\n";
      if (goalLeftFootPose.get() == null || goalRightFootPose.get() == null)
         errorMessage += "Need to set goal poses.";

      if (!errorMessage.isEmpty())
      {
         LogTools.warn(errorMessage);
         return false;
      }

      return true;
   }

   private boolean checkPostProcessingRequireds()
   {
      String errorMessage = "";

      if (leftFootPose.get() == null || rightFootPose.get() == null)
         errorMessage += "Need to set foot poses.\n";
      if (postProcessingLeftFootSupportPolygonReference.get() == null || postProcessingRightFootSupportPolygonReference.get() == null)
         errorMessage += "Need to set foot polygons.\n";

      if (footstepPlanResponseReference.get() == null)
         errorMessage += "Need a footstep plan to post process.\n";
      if (planarRegionListReference.get() == null)
         errorMessage += "Need planar regions.\n";

      if (!errorMessage.isEmpty())
      {
         LogTools.warn(errorMessage);
         return false;
      }

      return true;
   }

   private void requestHaltPlanning()
   {
      if (verbose)
         LogTools.info("Sending out a sleep request.");
      FootstepPlannerActionMessage footstepPlannerActionMessage = new FootstepPlannerActionMessage();
      footstepPlannerActionMessage.setRequestedAction(FootstepPlannerRequestedAction.HALT.toByte());
      plannerActionPublisher.publish(footstepPlannerActionMessage);
   }

   private void submitFootstepPlanningRequestPacket()
   {
      FootstepPlanningRequestPacket packet = new FootstepPlanningRequestPacket();
      packet.getStartLeftFootPose().set(leftFootPose.get());
      packet.getStartRightFootPose().set(rightFootPose.get());
      packet.getGoalLeftFootPose().set(goalLeftFootPose.get());
      packet.getGoalRightFootPose().set(goalRightFootPose.get());

      if (plannerInitialSupportSideReference.get() != null)
         packet.setRequestedInitialStanceSide(plannerInitialSupportSideReference.get().toByte());
      if (plannerTimeoutReference.get() != null)
         packet.setTimeout(plannerTimeoutReference.get());

      packet.setPlanBodyPath(planBodyPath.get());
      packet.setPerformAStarSearch(performAStarSearch.get());
      packet.setRequestedSwingPlanner(requestedSwingPlanner.get().toByte());

      if (plannerRequestIdReference.get() != null)
         packet.setPlannerRequestId(plannerRequestIdReference.get());
      if (plannerHorizonLengthReference.get() != null)
         packet.setHorizonLength(plannerHorizonLengthReference.get());
      if (terrainMapReference.get() != null)
         packet.getTerrainMapMessage().set(terrainMapReference.get());

      packet.setAssumeFlatGround(assumeFlatGround.get());
      packet.setGoalDistanceProximity(goalDistanceProximity.get());
      packet.setGoalYawProximity(goalYawProximity.get());
      packet.setSnapGoalSteps(snapGoalSteps.get());
      packet.setAbortIfGoalStepSnappingFails(abortIfGoalStepSnapFails.get());
      packet.setMaxIterations(maxIterations.get());

      if (referencePlan.get() != null && !referencePlan.get().isEmpty())
         packet.getReferencePlan().set(FootstepDataMessageConverter.createFootstepDataListFromPlan(referencePlan.get(), -1.0, -1.0));

      footstepPlanningRequestPublisher.publish(packet);
   }
}
