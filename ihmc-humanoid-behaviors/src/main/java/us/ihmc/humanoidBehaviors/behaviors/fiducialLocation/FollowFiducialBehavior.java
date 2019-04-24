package us.ihmc.humanoidBehaviors.behaviors.fiducialLocation;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.HeadTrajectoryMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import controller_msgs.msg.dds.WalkOverTerrainGoalPacket;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.humanoidBehaviors.behaviors.fiducialLocation.FollowFiducialBehavior.FollowFiducialState;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.time.YoStopwatch;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class FollowFiducialBehavior extends StateMachineBehavior<FollowFiducialState>
{
   enum FollowFiducialState
   {
      WAIT, PLAN_FROM_DOUBLE_SUPPORT, PLAN_FROM_SINGLE_SUPPORT, FINAL_STEPS
   }

   private boolean walkingComplete = false;

   private static final double HOW_CLOSE_TO_COME_TO_GOAL_WHEN_WALKING = 1;
   private static final double DISTANCE_TO_STOP_UPDATING_GOAL_PLAN = 1.5;

   private static final double defaultSwingTime = 2.2;
   private static final double defaultTransferTime = 0.5;

   private final WaitState waitState;
   private final PlanFromDoubleSupportState planFromDoubleSupportState;
   private final PlanFromSingleSupportState planFromSingleSupportState;

   private final HumanoidReferenceFrames referenceFrames;
   private final AtomicReference<FramePose3D> finalGoalPose = new AtomicReference<>();
   private final AtomicReference<FramePose3D> currentGoalPose = new AtomicReference<>();

   private final AtomicReference<FootstepPlanningToolboxOutputStatus> plannerResult = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();

   private final YoDouble swingTime = new YoDouble("swingTime", registry);
   private final YoDouble transferTime = new YoDouble("transferTime", registry);
   private final YoInteger planId = new YoInteger("planId", registry);
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepPublisher;
   private final IHMCROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final IHMCROS2Publisher<FootstepPlanningRequestPacket> planningRequestPublisher;
   private final IHMCROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher;
   private final IHMCROS2Publisher<HeadTrajectoryMessage> headTrajectoryPublisher;

   private final AtomicReference<WalkingStatusMessage> walkingStatus = new AtomicReference<>();

   private final GoalDetectorBehaviorService fiducialDetectorBehaviorService;
   private YoDouble yoTime;

   public FollowFiducialBehavior(String robotName, Ros2Node ros2Node, YoDouble yoTime, WholeBodyControllerParameters wholeBodyControllerParameters,
                                 HumanoidReferenceFrames referenceFrames, GoalDetectorBehaviorService goalDetectorBehaviorService)
   {
      super(robotName, "followFiducial", FollowFiducialState.class, yoTime, ros2Node);
      this.yoTime = yoTime;
      //createBehaviorInputSubscriber(FootstepPlanningToolboxOutputStatus.class, plannerResult::set);
      this.fiducialDetectorBehaviorService = goalDetectorBehaviorService;
      addBehaviorService(fiducialDetectorBehaviorService);

      createSubscriber(FootstepPlanningToolboxOutputStatus.class, footstepPlanningToolboxPubGenerator, plannerResult::set);

      createBehaviorInputSubscriber(WalkOverTerrainGoalPacket.class,
                                    (packet) -> finalGoalPose.set(new FramePose3D(ReferenceFrame.getWorldFrame(), packet.getPosition(),
                                                                                  packet.getOrientation())));
      createSubscriber(PlanarRegionsListMessage.class, REACommunicationProperties.publisherTopicNameGenerator, planarRegions::set);

      waitState = new WaitState(yoTime);
      planFromDoubleSupportState = new PlanFromDoubleSupportState();
      planFromSingleSupportState = new PlanFromSingleSupportState();

      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);
      this.referenceFrames = referenceFrames;

      swingTime.set(defaultSwingTime);
      transferTime.set(defaultTransferTime);

      footstepPublisher = createPublisherForController(FootstepDataListMessage.class);
      headTrajectoryPublisher = createPublisherForController(HeadTrajectoryMessage.class);
      toolboxStatePublisher = createPublisher(ToolboxStateMessage.class, footstepPlanningToolboxSubGenerator);
      planningRequestPublisher = createPublisher(FootstepPlanningRequestPacket.class, footstepPlanningToolboxSubGenerator);
      reaStateRequestPublisher = createPublisher(REAStateRequestMessage.class, REACommunicationProperties.subscriberTopicNameGenerator);
      setupStateMachine();
   }

   @Override
   protected FollowFiducialState configureStateMachineAndReturnInitialKey(StateMachineFactory<FollowFiducialState, BehaviorAction> factory)
   {

      BehaviorAction finalSteps = new BehaviorAction()
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("********************Entering Final Steps State********************************");

            super.setBehaviorInput();
         }

         @Override
         public boolean isDone(double timeInState)
         {
            // TODO Auto-generated method stub
            return doneWalking();
         }

         boolean doneWalking()
         {
            return (walkingStatus.get() != null) && (walkingStatus.get().getWalkingStatus() == WalkingStatus.COMPLETED.toByte());
         }

         @Override
         public void doPostBehaviorCleanup()
         {
            walkingComplete = true;
            super.doPostBehaviorCleanup();
         }
      };

      factory.setNamePrefix(getName() + "StateMachine").setRegistry(registry).buildYoClock(yoTime);

      factory.addState(FollowFiducialState.WAIT, waitState);
      factory.addState(FollowFiducialState.PLAN_FROM_DOUBLE_SUPPORT, planFromDoubleSupportState);
      factory.addState(FollowFiducialState.PLAN_FROM_SINGLE_SUPPORT, planFromSingleSupportState);

      StateTransitionCondition planFromDoubleSupportToWait = (time) -> plannerResult.get() != null
            && !FootstepPlanningResult.fromByte(plannerResult.get().getFootstepPlanningResult()).validForExecution();
      StateTransitionCondition planFromDoubleSupportToWalking = (time) -> plannerResult.get() != null
            && FootstepPlanningResult.fromByte(plannerResult.get().getFootstepPlanningResult()).validForExecution();

      factory.addState(FollowFiducialState.FINAL_STEPS, finalSteps);

      factory.addTransition(FollowFiducialState.PLAN_FROM_DOUBLE_SUPPORT, FollowFiducialState.FINAL_STEPS, t -> closeToGoal());
      factory.addTransition(FollowFiducialState.PLAN_FROM_SINGLE_SUPPORT, FollowFiducialState.FINAL_STEPS, t -> closeToGoal());

      factory.addTransition(FollowFiducialState.PLAN_FROM_DOUBLE_SUPPORT, FollowFiducialState.WAIT, planFromDoubleSupportToWait);
      factory.addTransition(FollowFiducialState.PLAN_FROM_DOUBLE_SUPPORT, FollowFiducialState.PLAN_FROM_SINGLE_SUPPORT, planFromDoubleSupportToWalking);
      factory.addTransition(FollowFiducialState.WAIT, FollowFiducialState.PLAN_FROM_DOUBLE_SUPPORT, t -> waitState.isDoneWaiting());
      factory.addTransition(FollowFiducialState.PLAN_FROM_SINGLE_SUPPORT, FollowFiducialState.WAIT, t -> planFromSingleSupportState.doneWalking());


      return FollowFiducialState.PLAN_FROM_DOUBLE_SUPPORT;
   }

   private boolean closeToGoal()
   {
      if ((walkingStatus.get() != null) && (walkingStatus.get().getWalkingStatus() == WalkingStatus.STARTED.toByte()))
      {
         if (finalGoalPose.get() != null)
         {
            FramePose3D goalPoseInMidFeetZUpFrame = new FramePose3D(finalGoalPose.get());
            goalPoseInMidFeetZUpFrame.changeFrame(referenceFrames.getMidFeetZUpFrame());
            double goalXYDistance = EuclidGeometryTools.pythagorasGetHypotenuse(Math.abs(goalPoseInMidFeetZUpFrame.getX()),
                                                                                Math.abs(goalPoseInMidFeetZUpFrame.getY()));
            double yawFromGoal = Math.abs(EuclidCoreTools.trimAngleMinusPiToPi(goalPoseInMidFeetZUpFrame.getYaw()));
            return goalXYDistance < DISTANCE_TO_STOP_UPDATING_GOAL_PLAN;// && yawFromGoal < Math.toRadians(25.0);
         }
      }
      return false;
   }

   @Override
   public void onBehaviorEntered()
   {
      sendTextToSpeechPacket("**************Entered follow fiducial behavior*******************");
      finalGoalPose.set(null);
      walkingStatus.set(null);
      currentGoalPose.set(null);
      plannerResult.set(null);
      walkingComplete = false;
      fiducialDetectorBehaviorService.initialize();
      getStateMachine().initialize();
      super.onBehaviorEntered();
   }

   private double currentTime = 0;
   private double waitTime = 2000;

   @Override
   public void doControl()
   {

      if (fiducialDetectorBehaviorService.getGoalHasBeenLocated())
      {

         FramePose3D tmpFP = new FramePose3D();
         fiducialDetectorBehaviorService.getReportedGoalPoseWorldFrame(tmpFP);
         setGoalPose(tmpFP);

         /*
          * if (System.currentTimeMillis() > currentTime + waitTime) { Point3D
          * location = new Point3D(); Quaternion orientation = new Quaternion();
          * tmpFP.get(location, orientation);
          * publishUIPositionCheckerPacket(location, orientation); currentTime =
          * System.currentTimeMillis();
          * //publishTextToSpeech("found "+tmpFP.getPosition().getX()+","+tmpFP.
          * getPosition().getY()+","+tmpFP.getPosition().getZ()); }
          */

      }

      super.doControl();
   }

   public boolean isDone()
   {

      return walkingComplete;
      //TODO change to is done when walk is complete, and make it so goal location does not get closer that a set ammount and does not update when close
      /*
       * if (finalGoalPose.get() != null) { FramePose3D
       * goalPoseInMidFeetZUpFrame = new FramePose3D(finalGoalPose.get());
       * goalPoseInMidFeetZUpFrame.changeFrame(referenceFrames.
       * getMidFeetZUpFrame()); double goalXYDistance =
       * EuclidGeometryTools.pythagorasGetHypotenuse(Math.abs(
       * goalPoseInMidFeetZUpFrame.getX()),
       * Math.abs(goalPoseInMidFeetZUpFrame.getY())); double yawFromGoal =
       * Math.abs(EuclidCoreTools.trimAngleMinusPiToPi(goalPoseInMidFeetZUpFrame
       * .getYaw())); return goalXYDistance < 1.0;// && yawFromGoal <
       * Math.toRadians(25.0); } return false;
       */
   }

   class WaitState extends BehaviorAction
   {
      private static final double initialWaitTime = 5.0;
      private static final double maxWaitTime = 10.0;

      private final YoDouble waitTime = new YoDouble("waitTime", registry);
      private final YoBoolean hasWalkedBetweenWaiting = new YoBoolean("hasWalkedBetweenWaiting", registry);
      private final YoStopwatch stopwatch;

      WaitState(YoDouble yoTime)
      {
         stopwatch = new YoStopwatch("waitStopWatch", yoTime, registry);
         stopwatch.start();
         waitTime.set(initialWaitTime);
      }

      @Override
      public void doAction(double timeInState)
      {

      }

      @Override
      public void onEntry()
      {
         lookDown();
         clearPlanarRegionsList();

         stopwatch.reset();

         if (hasWalkedBetweenWaiting.getBooleanValue())
         {
            waitTime.set(initialWaitTime);
            hasWalkedBetweenWaiting.set(false);
         }
         else
         {
            waitTime.set(Math.min(maxWaitTime, 2.0 * waitTime.getDoubleValue()));
         }

         sendTextToSpeechPacket("Waiting for " + waitTime.getDoubleValue() + " seconds");
      }

      private void lookDown()
      {
         AxisAngle orientationAxisAngle = new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0);
         Quaternion headOrientation = new Quaternion();
         headOrientation.set(orientationAxisAngle);
         HeadTrajectoryMessage headTrajectoryMessage = HumanoidMessageTools.createHeadTrajectoryMessage(1.0, headOrientation, ReferenceFrame.getWorldFrame(),
                                                                                                        referenceFrames.getChestFrame());
         headTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
         headTrajectoryPublisher.publish(headTrajectoryMessage);
      }

      private void lookUp()
      {
         AxisAngle orientationAxisAngle = new AxisAngle(0.0, 1.0, 0.0, 0);
         Quaternion headOrientation = new Quaternion();
         headOrientation.set(orientationAxisAngle);
         HeadTrajectoryMessage headTrajectoryMessage = HumanoidMessageTools.createHeadTrajectoryMessage(1.0, headOrientation, ReferenceFrame.getWorldFrame(),
                                                                                                        referenceFrames.getChestFrame());
         headTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
         headTrajectoryPublisher.publish(headTrajectoryMessage);
      }

      private void clearPlanarRegionsList()
      {
         REAStateRequestMessage clearRequest = new REAStateRequestMessage();
         clearRequest.setRequestClear(true);
         reaStateRequestPublisher.publish(clearRequest);
      }

      @Override
      public void onExit()
      {
         lookUp();
      }

      boolean isDoneWaiting()
      {
         return stopwatch.totalElapsed() >= waitTime.getDoubleValue();
      }
   }

   class PlanFromDoubleSupportState extends BehaviorAction
   {
      private final YoBoolean planningRequestHasBeenSent = new YoBoolean("planningRequestHasBeenSent", registry);

      @Override
      public void doAction(double timeInState)
      {
         if (!goalHasBeenSet())
         {
            return;
         }

         if (!planningRequestHasBeenSent.getBooleanValue())
         {
            PrintTools.info("sending planning request...");
            MovingReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FramePoint3D goalPoint = new FramePoint3D(finalGoalPose.get().getPosition());
            goalPoint.changeFrame(pelvisZUpFrame);

            RobotSide initialStanceSide = goalPoint.getY() > 0.0 ? RobotSide.RIGHT : RobotSide.LEFT;
            FramePose3D initialStanceFootPose = new FramePose3D(referenceFrames.getSoleFrame(initialStanceSide));
            initialStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());
            sendPlanningRequest(initialStanceFootPose, initialStanceSide);

            planningRequestHasBeenSent.set(true);
         }

         FootstepPlanningToolboxOutputStatus plannerResult = FollowFiducialBehavior.this.plannerResult.get();
      }

      @Override
      public void onEntry()
      {
         planningRequestHasBeenSent.set(false);
         sendTextToSpeechPacket("Starting PlanFromDoubleSupportState state");

      }

      @Override
      public void onExit()
      {
         sendFootstepPlan();
         sendTextToSpeechPacket("transitioning from planning to walking");
      }

   }

   class PlanFromSingleSupportState extends BehaviorAction
   {
      private final AtomicReference<FootstepStatusMessage> footstepStatus = new AtomicReference<>();

      private final FramePose3D touchdownPose = new FramePose3D();
      private final YoEnum<RobotSide> swingSide = YoEnum.create("swingSide", RobotSide.class, registry);

      PlanFromSingleSupportState()
      {
         createSubscriberFromController(FootstepStatusMessage.class, footstepStatus::set);
         createSubscriberFromController(WalkingStatusMessage.class, packet -> {
            walkingStatus.set(packet);
            waitState.hasWalkedBetweenWaiting.set(true);
         });
      }

      @Override
      public void doAction(double timeInState)
      {
         FootstepStatusMessage footstepStatus = this.footstepStatus.getAndSet(null);
         if (footstepStatus != null && footstepStatus.getFootstepStatus() == FootstepStatus.STARTED.toByte())
         {
            Point3D touchdownPosition = footstepStatus.getDesiredFootPositionInWorld();
            Quaternion touchdownOrientation = footstepStatus.getDesiredFootOrientationInWorld();
            touchdownPose.set(touchdownPosition, touchdownOrientation);
            swingSide.set(footstepStatus.getRobotSide());
            sendPlanningRequest(touchdownPose, swingSide.getEnumValue());
         }

         FootstepPlanningToolboxOutputStatus plannerResult = FollowFiducialBehavior.this.plannerResult.get();

         if (plannerResult != null && FootstepPlanningResult.fromByte(plannerResult.getFootstepPlanningResult()).validForExecution())
         {
            sendFootstepPlan();
         }
      }

      @Override
      public void onEntry()
      {
         this.footstepStatus.set(null);
         sendTextToSpeechPacket("Starting PlanFromSingleSupportState state");

      }

      @Override
      public void onExit()
      {
         footstepStatus.set(null);
      }

      boolean doneWalking()
      {
         return (walkingStatus.get() != null) && (walkingStatus.get().getWalkingStatus() == WalkingStatus.COMPLETED.toByte());
      }
   }

   public void setGoalPose(FramePose3D newGoalPose)
   {
      newGoalPose.appendYawRotation(Math.toRadians(90));
      adjustGoalToBeCloser(newGoalPose);
      finalGoalPose.set(newGoalPose);
   }

   private boolean goalHasBeenSet()
   {
      return finalGoalPose.get() != null;
   }

   private void sendFootstepPlan()
   {

      FootstepPlanningToolboxOutputStatus plannerResult = this.plannerResult.getAndSet(null);
      if (plannerResult != null)
      {
         FootstepDataListMessage footstepDataListMessage = plannerResult.getFootstepDataList();
         footstepDataListMessage.setDefaultSwingDuration(swingTime.getValue());
         footstepDataListMessage.setDefaultTransferDuration(transferTime.getDoubleValue());

         footstepDataListMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
         footstepPublisher.publish(footstepDataListMessage);
      }
   }

   private void sendPlanningRequest(FramePose3D initialStanceFootPose, RobotSide initialStanceSide)
   {

      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

      planId.increment();

      currentGoalPose.set(adjustGoalToBeCloser(finalGoalPose.get()));

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      currentGoalPose.get().get(location, orientation);
      publishUIPositionCheckerPacket(location, orientation);

      FootstepPlanningRequestPacket request = FootstepPlannerMessageTools.createFootstepPlanningRequestPacket(initialStanceFootPose, initialStanceSide,
                                                                                                              currentGoalPose.get(),
                                                                                                              FootstepPlannerType.A_STAR); //  FootstepPlannerType.VIS_GRAPH_WITH_A_STAR);

      if (planarRegions.get() != null)
         request.getPlanarRegionsListMessage().set(planarRegions.get());
      request.setTimeout(swingTime.getDoubleValue() - 0.25);
      request.setPlannerRequestId(planId.getIntegerValue());
      request.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE.ordinal());
      planningRequestPublisher.publish(request);
      plannerResult.set(null);
   }

   private void sendTextToSpeechPacket(String text)
   {
      publishTextToSpeech(text);
   }

   private FramePose3D adjustGoalToBeCloser(FramePose3D finalGoalPose)
   {

      FramePose3D goalPose = new FramePose3D(finalGoalPose);
      //      sendPacketToUI(new UIPositionCheckerPacket(goalPose.getFramePointCopy().getPoint(), goalPose.getFrameOrientationCopy().getQuaternion()));
      FramePose3D midFeetWorld = new FramePose3D(referenceFrames.getMidFeetZUpFrame());
      midFeetWorld.changeFrame(ReferenceFrame.getWorldFrame());

      Point3D pointBetweenFeet = new Point3D();
      Point3D goalPosition = new Point3D();
      Point3D shorterGoalPosition = new Point3D();
      Vector3D vectorFromFeetToGoal = new Vector3D();

      goalPosition.set(goalPose.getPosition());
      pointBetweenFeet.set(midFeetWorld.getPosition());
      vectorFromFeetToGoal.sub(goalPosition, pointBetweenFeet);

      double shorterGoalLength = HOW_CLOSE_TO_COME_TO_GOAL_WHEN_WALKING;

      if (vectorFromFeetToGoal.length() > shorterGoalLength)
      {
         vectorFromFeetToGoal.scale(shorterGoalLength / vectorFromFeetToGoal.length());
      }
      shorterGoalPosition.set(pointBetweenFeet);
      shorterGoalPosition.add(vectorFromFeetToGoal);
      goalPose.setPosition(shorterGoalPosition);

      double headingFromFeetToGoal = Math.atan2(vectorFromFeetToGoal.getY(), vectorFromFeetToGoal.getX());
      AxisAngle goalOrientation = new AxisAngle(0.0, 0.0, 1.0, headingFromFeetToGoal);
      goalPose.setOrientation(goalOrientation);
      return goalPose;

   }

   @Override
   public void onBehaviorExited()
   {
      sendTextToSpeechPacket("Follow fiducial Behavior Complete");
   }

}
