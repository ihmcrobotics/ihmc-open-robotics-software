package us.ihmc.humanoidBehaviors.behaviors.roughTerrain;

import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage.RequestType;
import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatusMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachine;
import us.ihmc.robotics.time.YoStopwatch;
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

   private final ReferenceFrame chestFrame;

   public WalkOverTerrainStateMachineBehavior(CommunicationBridge communicationBridge, YoDouble yoTime, FullHumanoidRobotModel fullRobotModel,
                                              HumanoidReferenceFrames referenceFrames)
   {
      super(communicationBridge);

      stateMachine = new StateMachine<WalkOverTerrainState>(getName() + "StateMachine", getName() + "StateMachine",
                                                            WalkOverTerrainState.class, yoTime, registry);

      waitState = new WaitState(yoTime, 5.0);
      planPathState = new PlanFootstepsState(communicationBridge, referenceFrames.getSoleFrames(), registry);
      walkingState = new WalkingState(communicationBridge);

      planPathState.setSwingTime(1.5);

      this.chestFrame = referenceFrames.getChestFrame();

      setupStateMachine();
   }

   private void setupStateMachine()
   {
      stateMachine.addState(waitState);
      stateMachine.addState(planPathState);
      stateMachine.addState(walkingState);

      planPathState.addStateTransition(WalkOverTerrainState.WALKING, () -> planPathState.planHasBeenReceived() && planPathState.planIsValidForExecution());
      planPathState.addStateTransition(WalkOverTerrainState.WAIT, () -> planPathState.planHasBeenReceived() && planPathState.planIsValidForExecution());
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
      return false;
   }

   class WaitState extends State<WalkOverTerrainState>
   {
      private final YoDouble waitTime = new YoDouble("waitTime", registry);
      private final YoStopwatch stopwatch;

      WaitState(YoDouble yoTime, double waitTime)
      {
         super(WalkOverTerrainState.WAIT);
         stopwatch = new YoStopwatch("waitStopWatch", yoTime, registry);
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
         stopwatch.start();
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

      public boolean isDoneWaiting()
      {
         return stopwatch.totalElapsed() >= waitTime.getDoubleValue();
      }
   }

   class WalkingState extends State<WalkOverTerrainState>
   {
      private final AtomicReference<WalkingStatusMessage> walkingStatusMessage = new AtomicReference<>();

      WalkingState(CommunicationBridge communicationBridge)
      {
         super(WalkOverTerrainState.WALKING);
         communicationBridge.attachListener(WalkingStatusMessage.class, walkingStatusMessage::set);
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
         WalkingStatusMessage walkingStatusMessage = this.walkingStatusMessage.get();
         return (walkingStatusMessage != null) && (walkingStatusMessage.status == WalkingStatusMessage.Status.COMPLETED);
      }
   }
}
