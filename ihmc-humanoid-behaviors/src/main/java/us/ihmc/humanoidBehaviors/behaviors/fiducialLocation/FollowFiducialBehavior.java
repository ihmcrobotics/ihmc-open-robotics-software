package us.ihmc.humanoidBehaviors.behaviors.fiducialLocation;

import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepDataListMessage;
import toolbox_msgs.msg.dds.FootstepPlanningRequestPacket;
import toolbox_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.HeadTrajectoryMessage;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import perception_msgs.msg.dds.REAStateRequestMessage;
import toolbox_msgs.msg.dds.ToolboxStateMessage;
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
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.humanoidBehaviors.behaviors.fiducialLocation.FollowFiducialBehavior.FollowFiducialState;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Node;
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

   private static final double HOW_CLOSE_TO_COME_TO_GOAL_WHEN_WALKING = 1;
   private static final double DISTANCE_TO_STOP_UPDATING_GOAL_PLAN = 1.5;

   private static final double defaultSwingTime = 2.2;
   private static final double defaultTransferTime = 0.5;

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
   private final double waitTimeValue = 5.0;
   private boolean hasWalkedBetweenWaiting = false;

   private final AtomicReference<WalkingStatusMessage> walkingStatus = new AtomicReference<>();

   private final GoalDetectorBehaviorService fiducialDetectorBehaviorService;
   private YoDouble yoTime;

   private final AtomicReference<FootstepStatusMessage> footstepStatusReference = new AtomicReference<>();
   private static int id = 0;
   private final double idealStanceWidth;

   public FollowFiducialBehavior(String robotName, ROS2Node ros2Node, YoDouble yoTime, WholeBodyControllerParameters wholeBodyControllerParameters,
                                 HumanoidReferenceFrames referenceFrames, GoalDetectorBehaviorService goalDetectorBehaviorService)
   {
      super(robotName, "followFiducial-"+id++, FollowFiducialState.class, yoTime, ros2Node);
      this.yoTime = yoTime;
      this.idealStanceWidth = wholeBodyControllerParameters.getWalkingControllerParameters().getSteppingParameters().getInPlaceWidth();
      //createBehaviorInputSubscriber(FootstepPlanningToolboxOutputStatus.class, plannerResult::set);
      this.fiducialDetectorBehaviorService = goalDetectorBehaviorService;
      addBehaviorService(fiducialDetectorBehaviorService);

      createSubscriber(FootstepPlanningToolboxOutputStatus.class, footstepPlannerOutputTopic, plannerResult::set);

      createBehaviorInputSubscriber(WalkOverTerrainGoalPacket.class,
                                    (packet) -> finalGoalPose.set(new FramePose3D(ReferenceFrame.getWorldFrame(), packet.getPosition(),
                                                                                  packet.getOrientation())));
      createSubscriber(PlanarRegionsListMessage.class, REACommunicationProperties.outputTopic, planarRegions::set);

      createSubscriberFromController(FootstepStatusMessage.class, footstepStatusReference::set);
      createSubscriberFromController(WalkingStatusMessage.class, packet -> {
         walkingStatus.set(packet);
         hasWalkedBetweenWaiting = true;
      });
      
      planFromDoubleSupportState = new PlanFromDoubleSupportState();
      planFromSingleSupportState = new PlanFromSingleSupportState();

      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);
      this.referenceFrames = referenceFrames;

      swingTime.set(defaultSwingTime);
      transferTime.set(defaultTransferTime);

      footstepPublisher = createPublisherForController(FootstepDataListMessage.class);
      headTrajectoryPublisher = createPublisherForController(HeadTrajectoryMessage.class);
      toolboxStatePublisher = createPublisher(ToolboxStateMessage.class, footstepPlannerInputTopic);
      planningRequestPublisher = createPublisher(FootstepPlanningRequestPacket.class, footstepPlannerInputTopic);
      reaStateRequestPublisher = createPublisher(REAStateRequestMessage.class, REACommunicationProperties.inputTopic);
      setupStateMachine();
   }

   @Override
   protected FollowFiducialState configureStateMachineAndReturnInitialKey(StateMachineFactory<FollowFiducialState, BehaviorAction> factory)
   {

      BehaviorAction finalSteps = new BehaviorAction()
      {
         @Override
         public void onEntry()
         {
            PrintTools.info("finalSteps");

            super.onEntry();
         }
         @Override
         public boolean isDone()
         {
            return (walkingStatus.get() != null) && (walkingStatus.get().getWalkingStatus() == WalkingStatus.COMPLETED.toByte());
         }
      };

      BehaviorAction waitForPlanarRegions = new BehaviorAction(new SleepBehavior(robotName, ros2Node, yoTime))
      {
       
         @Override
         protected void setBehaviorInput()
         {
            
            super.setBehaviorInput();
         }
            @Override
            public void onEntry()
            {
               lookDown();
               if (hasWalkedBetweenWaiting)
               {
                  hasWalkedBetweenWaiting = false;
                  clearPlanarRegionsList();
               }
               sendTextToSpeechPacket("Waiting for " + waitTimeValue + " seconds");
               super.onEntry();
            }

            @Override
            public void onExit(double timeInState)
            {
               lookUp();
               super.onExit(timeInState);
            }

            private void lookDown()
            {
               AxisAngle orientationAxisAngle = new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0);
               Quaternion headOrientation = new Quaternion();
               headOrientation.set(orientationAxisAngle);
               HeadTrajectoryMessage headTrajectoryMessage = HumanoidMessageTools.createHeadTrajectoryMessage(1.0, headOrientation,
                                                                                                              ReferenceFrame.getWorldFrame(),
                                                                                                              referenceFrames.getChestFrame());
               headTrajectoryMessage.setDestination(PacketDestination.CONTROLLER.ordinal());
               headTrajectoryPublisher.publish(headTrajectoryMessage);
            }

            private void lookUp()
            {
               AxisAngle orientationAxisAngle = new AxisAngle(0.0, 1.0, 0.0, 0);
               Quaternion headOrientation = new Quaternion();
               headOrientation.set(orientationAxisAngle);
               HeadTrajectoryMessage headTrajectoryMessage = HumanoidMessageTools.createHeadTrajectoryMessage(1.0, headOrientation,
                                                                                                              ReferenceFrame.getWorldFrame(),
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

         
      };

      factory.addState(FollowFiducialState.WAIT, waitForPlanarRegions);
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
      factory.addTransition(FollowFiducialState.WAIT, FollowFiducialState.PLAN_FROM_DOUBLE_SUPPORT, t -> waitForPlanarRegions.isDone());
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
      }
      super.doControl();
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
            FramePose3D startLeftFootPose = new FramePose3D(referenceFrames.getSoleFrame(RobotSide.LEFT));
            FramePose3D startRightFootPose = new FramePose3D(referenceFrames.getSoleFrame(RobotSide.RIGHT));
            startLeftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
            startRightFootPose.changeFrame(ReferenceFrame.getWorldFrame());
            sendPlanningRequest(initialStanceSide, startLeftFootPose, startRightFootPose);

            planningRequestHasBeenSent.set(true);
         }

         FootstepPlanningToolboxOutputStatus plannerResult = FollowFiducialBehavior.this.plannerResult.get();
      }

      @Override
      public void onEntry()
      {
         planningRequestHasBeenSent.set(false);
         PrintTools.info("enter");

      }

      @Override
      public void onExit(double timeInState)
      {
         PrintTools.info("leaving");

         sendFootstepPlan();
      }

   }

   class PlanFromSingleSupportState extends BehaviorAction
   {
      private final FramePose3D touchdownPose = new FramePose3D();
      private final FramePose3D leftFootStartPose = new FramePose3D();
      private final FramePose3D rightFootStartPose = new FramePose3D();
      private final YoEnum<RobotSide> swingSide = new YoEnum<>("swingSide", registry, RobotSide.class);

      PlanFromSingleSupportState()
      {
       
      }

      @Override
      public void doAction(double timeInState)
      {
         FootstepStatusMessage footstepStatus = footstepStatusReference.getAndSet(null);
         if (footstepStatus != null && footstepStatus.getFootstepStatus() == FootstepStatus.STARTED.toByte())
         {
            Point3D touchdownPosition = footstepStatus.getDesiredFootPositionInWorld();
            Quaternion touchdownOrientation = footstepStatus.getDesiredFootOrientationInWorld();
            touchdownPose.set(touchdownPosition, touchdownOrientation);
            swingSide.set(footstepStatus.getRobotSide());

            if (swingSide.getValue() == RobotSide.LEFT)
            {
               leftFootStartPose.set(touchdownPose);
               rightFootStartPose.setToZero(referenceFrames.getSoleFrame(RobotSide.RIGHT));
               rightFootStartPose.changeFrame(ReferenceFrame.getWorldFrame());
            }
            else
            {
               rightFootStartPose.set(touchdownPose);
               leftFootStartPose.setToZero(referenceFrames.getSoleFrame(RobotSide.LEFT));
               leftFootStartPose.changeFrame(ReferenceFrame.getWorldFrame());
            }

            sendPlanningRequest(swingSide.getEnumValue(), leftFootStartPose, rightFootStartPose);
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
         footstepStatusReference.set(null);
         sendTextToSpeechPacket("Starting PlanFromSingleSupportState state");

      }

      @Override
      public void onExit(double timeInState)
      {
         footstepStatusReference.set(null);
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

   private void sendPlanningRequest(RobotSide initialStanceSide, FramePose3D startLeftFootPose, FramePose3D startRightFootPose)
   {

      toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

      planId.increment();

      currentGoalPose.set(adjustGoalToBeCloser(finalGoalPose.get()));

      Point3D location = new Point3D();
      Quaternion orientation = new Quaternion();
      currentGoalPose.get().get(location, orientation);
      publishUIPositionCheckerPacket(location, orientation);
      boolean planBodyPath = false;

      FootstepPlanningRequestPacket request = FootstepPlannerMessageTools.createFootstepPlanningRequestPacket(initialStanceSide,
                                                                                                              startLeftFootPose,
                                                                                                              startRightFootPose,
                                                                                                              currentGoalPose.get(),
                                                                                                              idealStanceWidth,
                                                                                                              planBodyPath);

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
      goalPose.getPosition().set(shorterGoalPosition);

      double headingFromFeetToGoal = Math.atan2(vectorFromFeetToGoal.getY(), vectorFromFeetToGoal.getX());
      AxisAngle goalOrientation = new AxisAngle(0.0, 0.0, 1.0, headingFromFeetToGoal);
      goalPose.getOrientation().set(goalOrientation);
      return goalPose;

   }

   @Override
   public void onBehaviorExited()
   {
      sendTextToSpeechPacket("Follow fiducial Behavior Complete");
   }

}
