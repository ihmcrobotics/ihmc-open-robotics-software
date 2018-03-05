package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.PlanarRegionsListMessage;
import us.ihmc.communication.packets.PlanarRegionsRequestType;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.packets.ToolboxStateMessage;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.WalkOverTerrainGoalPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningRequestPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepPlanningToolboxOutputStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachine.old.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.robotics.time.YoStopwatch;
import us.ihmc.wholeBodyController.WholeBodyControllerParameters;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class WalkOverTerrainStateMachineBehavior extends AbstractBehavior
{
   enum WalkOverTerrainState
   {
      WAIT, PLAN_FROM_DOUBLE_SUPPORT, PLAN_FROM_SINGLE_SUPPORT
   }

   private static final double defaultSwingTime = 2.2;
   private static final double defaultTransferTime = 0.5;

   private final StateMachine<WalkOverTerrainState> stateMachine;

   private final WaitState waitState;
   private final PlanFromDoubleSupportState planFromDoubleSupportState;
   private final PlanFromSingleSupportState planFromSingleSupportState;

   private final HumanoidReferenceFrames referenceFrames;
   private final AtomicReference<FramePose3D> goalPose = new AtomicReference<>();
   private final AtomicReference<FootstepPlanningToolboxOutputStatus> plannerResult = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> planarRegions = new AtomicReference<>();

   private final YoDouble swingTime = new YoDouble("swingTime", registry);
   private final YoDouble transferTime = new YoDouble("transferTime", registry);
   private final YoInteger planId = new YoInteger("planId", registry);

   public WalkOverTerrainStateMachineBehavior(CommunicationBridgeInterface communicationBridge, YoDouble yoTime, WholeBodyControllerParameters wholeBodyControllerParameters, HumanoidReferenceFrames referenceFrames)
   {
      super(communicationBridge);

      communicationBridge.attachListener(FootstepPlanningToolboxOutputStatus.class, plannerResult::set);
      communicationBridge.attachListener(WalkOverTerrainGoalPacket.class, (packet) -> goalPose.set(new FramePose3D(ReferenceFrame.getWorldFrame(), packet.position, packet.orientation)));
      communicationBridge.attachListener(PlanarRegionsListMessage.class, planarRegions::set);

      stateMachine = new StateMachine<>(getName() + "StateMachine", getName() + "StateMachineSwitchTime", WalkOverTerrainState.class, yoTime, registry);

      waitState = new WaitState(yoTime);
      planFromDoubleSupportState = new PlanFromDoubleSupportState();
      planFromSingleSupportState = new PlanFromSingleSupportState();

      communicationBridge.attachListener(WalkOverTerrainGoalPacket.class, (packet) ->
      {
         goalPose.set(new FramePose3D(ReferenceFrame.getWorldFrame(), packet.position, packet.orientation));
      });

      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);
      this.referenceFrames = referenceFrames;

      swingTime.set(defaultSwingTime);
      transferTime.set(defaultTransferTime);

      setupStateMachine();
   }

   private void setupStateMachine()
   {
      stateMachine.addState(waitState);
      stateMachine.addState(planFromDoubleSupportState);
      stateMachine.addState(planFromSingleSupportState);

      StateTransitionCondition planFromDoubleSupportToWait = () -> plannerResult.get() != null && !FootstepPlanningResult.fromByte(plannerResult.get().footstepPlanningResult).validForExecution();
      StateTransitionCondition planFromDoubleSupportToWalking = () -> plannerResult.get() != null && FootstepPlanningResult.fromByte(plannerResult.get().footstepPlanningResult).validForExecution();

      planFromDoubleSupportState.addStateTransition(WalkOverTerrainState.WAIT, planFromDoubleSupportToWait);
      planFromDoubleSupportState.addStateTransition(WalkOverTerrainState.PLAN_FROM_SINGLE_SUPPORT, planFromDoubleSupportToWalking);
      waitState.addStateTransition(WalkOverTerrainState.PLAN_FROM_DOUBLE_SUPPORT, waitState::isDoneWaiting);
      planFromSingleSupportState.addStateTransition(WalkOverTerrainState.WAIT, planFromSingleSupportState::doneWalking);

      stateMachine.setCurrentState(WalkOverTerrainState.PLAN_FROM_DOUBLE_SUPPORT);
   }

   @Override
   public void onBehaviorEntered()
   {
      sendTextToSpeechPacket("Starting walk over terrain behavior");
   }

   @Override
   public void onBehaviorExited()
   {
   }

   @Override
   public void doControl()
   {
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
   }

   @Override
   public void onBehaviorAborted()
   {

   }

   @Override
   public void onBehaviorPaused()
   {

   }

   @Override
   public void onBehaviorResumed()
   {

   }

   @Override
   public boolean isDone()
   {
      FramePose3D goalPoseInMidFeetZUpFrame = new FramePose3D(goalPose.get());
      goalPoseInMidFeetZUpFrame.changeFrame(referenceFrames.getMidFeetZUpFrame());
      double goalXYDistance = EuclidGeometryTools.pythagorasGetHypotenuse(goalPoseInMidFeetZUpFrame.getX(), goalPoseInMidFeetZUpFrame.getY());
      double yawFromGoal = Math.abs(EuclidCoreTools.trimAngleMinusPiToPi(goalPoseInMidFeetZUpFrame.getYaw()));
      return goalXYDistance < 0.2 && yawFromGoal < Math.toRadians(25.0);
   }

   class WaitState extends State<WalkOverTerrainState>
   {
      private static final double initialWaitTime = 5.0;
      private static final double maxWaitTime = 30.0;

      private final YoDouble waitTime = new YoDouble("waitTime", registry);
      private final YoBoolean hasWalkedBetweenWaiting = new YoBoolean("hasWalkedBetweenWaiting", registry);
      private final YoStopwatch stopwatch;

      WaitState(YoDouble yoTime)
      {
         super(WalkOverTerrainState.WAIT);

         stopwatch = new YoStopwatch("waitStopWatch", yoTime, registry);
         stopwatch.start();
         waitTime.set(initialWaitTime);
      }

      @Override
      public void doAction()
      {

      }

      @Override
      public void doTransitionIntoAction()
      {
         lookDown();
         clearPlanarRegionsList();

         stopwatch.reset();

         if(hasWalkedBetweenWaiting.getBooleanValue())
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
         HeadTrajectoryMessage headTrajectoryMessage = HumanoidMessageTools.createHeadTrajectoryMessage(1.0, headOrientation, ReferenceFrame.getWorldFrame(), referenceFrames.getChestFrame());
         headTrajectoryMessage.setDestination(PacketDestination.CONTROLLER);
         sendPacket(headTrajectoryMessage);
      }

      private void clearPlanarRegionsList()
      {
         RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = MessageTools.createRequestPlanarRegionsListMessage(PlanarRegionsRequestType.CLEAR);
         requestPlanarRegionsListMessage.setDestination(PacketDestination.REA_MODULE);
         sendPacket(requestPlanarRegionsListMessage);
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }

      boolean isDoneWaiting()
      {
         return stopwatch.totalElapsed() >= waitTime.getDoubleValue();
      }
   }

   class PlanFromDoubleSupportState extends State<WalkOverTerrainState>
   {
      private final YoBoolean planningRequestHasBeenSent = new YoBoolean("planningRequestHasBeenSent", registry);

      PlanFromDoubleSupportState()
      {
         super(WalkOverTerrainState.PLAN_FROM_DOUBLE_SUPPORT);
      }

      @Override
      public void doAction()
      {
         if(!goalHasBeenSet())
         {
            return;
         }

         if(!planningRequestHasBeenSent.getBooleanValue())
         {
            PrintTools.info("sending planning request...");
            MovingReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
            FramePoint3D goalPoint = new FramePoint3D(goalPose.get().getPosition());
            goalPoint.changeFrame(pelvisZUpFrame);

            RobotSide initialStanceSide = goalPoint.getY() > 0.0 ? RobotSide.RIGHT : RobotSide.LEFT;
            FramePose3D initialStanceFootPose = new FramePose3D(referenceFrames.getSoleFrame(initialStanceSide));
            initialStanceFootPose.changeFrame(ReferenceFrame.getWorldFrame());
            sendPlanningRequest(initialStanceFootPose, initialStanceSide);

            planningRequestHasBeenSent.set(true);
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
         planningRequestHasBeenSent.set(false);
      }

      @Override
      public void doTransitionOutOfAction()
      {
         sendFootstepPlan();
         PrintTools.info("transitioning to walking");
      }
   }

   class PlanFromSingleSupportState extends State<WalkOverTerrainState>
   {
      private final AtomicReference<FootstepStatusMessage> footstepStatus = new AtomicReference<>();
      private final AtomicReference<WalkingStatusMessage> walkingStatus = new AtomicReference<>();

      private final FramePose3D touchdownPose = new FramePose3D();
      private final YoEnum<RobotSide> swingSide = YoEnum.create("swingSide", RobotSide.class, registry);

      PlanFromSingleSupportState()
      {
         super(WalkOverTerrainState.PLAN_FROM_SINGLE_SUPPORT);
         communicationBridge.attachListener(FootstepStatusMessage.class, footstepStatus::set);
         communicationBridge.attachListener(WalkingStatusMessage.class, packet -> { walkingStatus.set(packet); waitState.hasWalkedBetweenWaiting.set(true);});
      }

      @Override
      public void doAction()
      {
         FootstepStatusMessage footstepStatus = this.footstepStatus.getAndSet(null);
         if(footstepStatus != null && footstepStatus.footstepStatus == FootstepStatus.STARTED.toByte())
         {
            Point3D touchdownPosition = footstepStatus.getDesiredFootPositionInWorld();
            Quaternion touchdownOrientation = footstepStatus.getDesiredFootOrientationInWorld();
            touchdownPose.set(touchdownPosition, touchdownOrientation);
            swingSide.set(footstepStatus.getRobotSide());
            sendPlanningRequest(touchdownPose, swingSide.getEnumValue());
         }

         FootstepPlanningToolboxOutputStatus plannerResult = WalkOverTerrainStateMachineBehavior.this.plannerResult.get();
         if(plannerResult != null && FootstepPlanningResult.fromByte(plannerResult.footstepPlanningResult).validForExecution())
         {
            sendFootstepPlan();
         }
      }

      @Override
      public void doTransitionIntoAction()
      {
      }

      @Override
      public void doTransitionOutOfAction()
      {
         footstepStatus.set(null);
      }

      boolean doneWalking()
      {
         return (walkingStatus.get() != null) && (walkingStatus.get().walkingStatus == WalkingStatus.COMPLETED.toByte());
      }
   }

   private boolean goalHasBeenSet()
   {
      return goalPose.get() != null;
   }

   private void sendFootstepPlan()
   {
      FootstepPlanningToolboxOutputStatus plannerResult = this.plannerResult.getAndSet(null);
      FootstepDataListMessage footstepDataListMessage = plannerResult.footstepDataList;
      footstepDataListMessage.setDefaultSwingDuration(swingTime.getValue());
      footstepDataListMessage.setDefaultTransferDuration(transferTime.getDoubleValue());

      footstepDataListMessage.setDestination(PacketDestination.CONTROLLER);
      communicationBridge.sendPacket(footstepDataListMessage);
   }

   private void sendPlanningRequest(FramePose3D initialStanceFootPose, RobotSide initialStanceSide)
   {
      ToolboxStateMessage wakeUp = MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP);
      wakeUp.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE);
      communicationBridge.sendPacket(wakeUp);

      planId.increment();
      FootstepPlanningRequestPacket request = HumanoidMessageTools.createFootstepPlanningRequestPacket(initialStanceFootPose, initialStanceSide, goalPose.get(), FootstepPlannerType.A_STAR); //  FootstepPlannerType.VIS_GRAPH_WITH_A_STAR);
      request.setPlanarRegionsListMessage(planarRegions.get());
      request.setTimeout(swingTime.getDoubleValue() - 0.25);
      request.setPlannerRequestId(planId.getIntegerValue());
      request.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE);
      communicationBridge.sendPacket(request);
      plannerResult.set(null);
   }

   private void sendTextToSpeechPacket(String text)
   {
      sendPacketToUI(MessageTools.createTextToSpeechPacket(text));
   }
}
