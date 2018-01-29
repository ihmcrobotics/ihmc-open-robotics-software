package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage.RequestType;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionAction;
import us.ihmc.robotics.time.YoStopwatch;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.concurrent.atomic.AtomicReference;

public class WalkOverTerrainStateMachineBehavior extends AbstractBehavior
{
   enum WalkOverTerrainState
   {
      WAIT, PLAN_FOOTSTEPS, WALKING
   }

   private final StateMachine<WalkOverTerrainState> stateMachine;

   private final WaitState waitState;
   private final PlanFootstepsState planPathState;
   private final WalkingState walkingState;

   private final ReferenceFrame chestFrame, midFeetZUpFrame;
   private final AtomicReference<FramePose3D> goalPose = new AtomicReference<>();

   public WalkOverTerrainStateMachineBehavior(CommunicationBridge communicationBridge, YoDouble yoTime, HumanoidReferenceFrames referenceFrames)
   {
      super(communicationBridge);

      stateMachine = new StateMachine<WalkOverTerrainState>(getName() + "StateMachine", getName() + "StateMachine",
                                                            WalkOverTerrainState.class, yoTime, registry);

      waitState = new WaitState(yoTime);
      planPathState = new PlanFootstepsState(communicationBridge, referenceFrames.getSoleFrames(), registry);
      walkingState = new WalkingState(communicationBridge);

      communicationBridge.attachListener(WalkOverTerrainGoalPacket.class, (packet) ->
      {
         goalPose.set(packet.goalPose);
         planPathState.setGoalPose(packet.goalPose);
      });

      this.chestFrame = referenceFrames.getChestFrame();
      this.midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();

      setupStateMachine();
   }

   private void setupStateMachine()
   {
      stateMachine.addState(waitState);
      stateMachine.addState(planPathState);
      stateMachine.addState(walkingState);

      StateTransitionAction planningToWalkingAction = () ->
      {
         waitState.hasWalkedBetweenWaiting.set(true);
      };

      planPathState.addStateTransition(new StateTransition<>(WalkOverTerrainState.WALKING, () -> planPathState.planHasBeenReceived() && planPathState.planIsValidForExecution(),
                                                           planningToWalkingAction));
      planPathState.addStateTransition(WalkOverTerrainState.WAIT, () -> planPathState.planHasBeenReceived() && !planPathState.planIsValidForExecution());
      waitState.addStateTransition(WalkOverTerrainState.PLAN_FOOTSTEPS, waitState::isDoneWaiting);
      walkingState.addStateTransition(WalkOverTerrainState.PLAN_FOOTSTEPS, walkingState::stepHasCompleted);

      stateMachine.setCurrentState(WalkOverTerrainState.PLAN_FOOTSTEPS);
   }

   @Override
   public void onBehaviorEntered()
   {
   }

   @Override
   public void onBehaviorExited()
   {
      stateMachine.setCurrentState(WalkOverTerrainState.PLAN_FOOTSTEPS);
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
      goalPoseInMidFeetZUpFrame.changeFrame(midFeetZUpFrame);
      double goalXYDistance = EuclidGeometryTools.pythagorasGetHypotenuse(goalPoseInMidFeetZUpFrame.getX(), goalPoseInMidFeetZUpFrame.getY());
      return goalXYDistance < 0.2;
   }

   class WaitState extends State<WalkOverTerrainState>
   {
      private static final double initialWaitTime = 5.0;

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
            waitTime.set(2.0 * waitTime.getDoubleValue());
         }
      }

      private void lookDown()
      {
         AxisAngle orientationAxisAngle = new AxisAngle(0.0, 1.0, 0.0, Math.PI / 2.0);
         Quaternion headOrientation = new Quaternion();
         headOrientation.set(orientationAxisAngle);
         HeadTrajectoryMessage headTrajectoryMessage = new HeadTrajectoryMessage(2.0, headOrientation, ReferenceFrame.getWorldFrame(), chestFrame);
         headTrajectoryMessage.setDestination(PacketDestination.CONTROLLER);
         sendPacket(headTrajectoryMessage);
      }

      private void clearPlanarRegionsList()
      {
         RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = new RequestPlanarRegionsListMessage(RequestType.CLEAR);
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

   class WalkingState extends State<WalkOverTerrainState>
   {
      private final AtomicReference<FootstepStatus> footstepStatusMessage = new AtomicReference<>();

      WalkingState(CommunicationBridge communicationBridge)
      {
         super(WalkOverTerrainState.WALKING);
         communicationBridge.attachListener(FootstepStatus.class, footstepStatusMessage::set);
      }

      @Override
      public void doAction()
      {

      }

      @Override
      public void doTransitionIntoAction()
      {
         // TODO adjust com based on upcoming footsteps
      }

      @Override
      public void doTransitionOutOfAction()
      {

      }

      boolean stepHasCompleted()
      {
         FootstepStatus footstepStatus = this.footstepStatusMessage.getAndSet(null);
         return (footstepStatus != null) && (footstepStatus.status == FootstepStatus.Status.COMPLETED);
      }
   }
}
