package us.ihmc.behaviors.demo;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeControlFlowNode;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.behaviors.BehaviorDefinition;
import us.ihmc.behaviors.BehaviorInterface;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.stairs.TraverseStairsBehavior;
import us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.BehaviorControlModeEnum;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.CurrentBehaviorStatus;
import us.ihmc.humanoidRobotics.communication.packets.behaviors.HumanoidBehaviorType;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI.*;
import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI.ConfirmDoor;

public class BuildingExplorationBehavior extends BehaviorTreeControlFlowNode implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Building Exploration",
                                                                              BuildingExplorationBehavior::new,
                                                                              BuildingExplorationBehaviorAPI.API);

   private static final int UPDATE_RATE_MILLIS = 50;
   private static final double xyProximityToDoorToStopWalking = 1.6;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoEnum<BuildingExplorationStateName> requestedState = new YoEnum<>("requestedState", "", registry, BuildingExplorationStateName.class, true);
   private final StateMachine<BuildingExplorationStateName, State> stateMachine;

   private final AtomicBoolean isRunning = new AtomicBoolean();
   private final AtomicBoolean stopRequested = new AtomicBoolean();

   private final AtomicReference<DoorLocationPacket> doorLocationPacket = new AtomicReference<>();
   private final AtomicReference<RobotConfigurationData> robotConfigurationData = new AtomicReference<>();
   private final Pose3D bombPose = new Pose3D();

   private final ScheduledExecutorService executorService;
   private final LookAndStepBehavior lookAndStepBehavior;
   private final BehaviorHelper helper;
   private ScheduledFuture<?> stateMachineTask = null;
   private PausablePeriodicThread toolboxWakerThread;

   private Consumer<BuildingExplorationStateName> stateChangedCallback = state -> {};
   private Runnable doorDetectedCallback = () -> {};

   private final BuildingExplorationBehaviorTeleopState teleopState;
   private final BuildingExplorationBehaviorLookAndStepState lookAndStepState;
   private final WalkThroughDoorState walkThroughDoorState;
   private final TraverseStairsState traverseStairsState;
   private final AtomicReference<Pose3D> goal;

   public BuildingExplorationBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      LogTools.info("Constructing");
      executorService = Executors.newSingleThreadScheduledExecutor();
      goal = helper.subscribeViaReference(Goal, new Pose3D());
      lookAndStepBehavior = new LookAndStepBehavior(helper);
      teleopState = new BuildingExplorationBehaviorTeleopState(helper);
      lookAndStepState = new BuildingExplorationBehaviorLookAndStepState(this, helper, bombPose);
      walkThroughDoorState = new WalkThroughDoorState(helper);
      traverseStairsState = new TraverseStairsState(helper, bombPose);

      addChild(lookAndStepBehavior);

      helper.subscribeViaCallback(RequestedState, this::requestState);
      AtomicReference<BuildingExplorationStateName> requestedState = helper.subscribeViaReference(RequestedState, BuildingExplorationStateName.TELEOP);

      helper.subscribeViaCallback(Start, start ->
      {
         LogTools.info("Starting");
         setBombPose(goal.get());
         requestState(requestedState.get());
         start();
      });
      helper.subscribeViaCallback(Stop, s -> stop());
      setStateChangedCallback(newState -> helper.publish(CurrentState, newState));
      setDebrisDetectedCallback(() -> helper.publish(DebrisDetected, true));
      setStairsDetectedCallback(() -> helper.publish(StairsDetected, true));
      setDoorDetectedCallback(() -> helper.publish(DoorDetected, true));
      helper.subscribeViaCallback(IgnoreDebris, ignore -> ignoreDebris());
      helper.subscribeViaCallback(ConfirmDoor, confirm -> proceedWithDoorBehavior());
      helper.subscribeToDoorLocationViaCallback(doorLocationPacket::set);
      helper.subscribeToRobotConfigurationDataViaCallback(robotConfigurationData::set);

      startWakeUpToolboxesThread();

      try
      {
         stateMachine = buildStateMachine();
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      stateMachine.addStateChangedListener((from, to) -> stateChangedCallback.accept(to));
   }

   private void startWakeUpToolboxesThread()
   {
      toolboxWakerThread = new PausablePeriodicThread("ToolboxWaker", 1.0, true, () ->
      {
         helper.publishToolboxState(FiducialDetectorToolboxModule::getInputTopic, ToolboxState.WAKE_UP);
         helper.publishToolboxState(ObjectDetectorToolboxModule::getInputTopic, ToolboxState.WAKE_UP);
      });
      toolboxWakerThread.start();
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      return BehaviorTreeNodeStatus.SUCCESS;
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      helper.setCommunicationCallbacksEnabled(enabled);
      if (!enabled)
         lookAndStepBehavior.setEnabled(false);
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }

   public void setBombPose(Pose3DReadOnly bombPose)
   {
      this.bombPose.set(bombPose);
   }

   public void requestState(BuildingExplorationStateName requestedState)
   {
      this.requestedState.set(requestedState);
   }

   public void setStateChangedCallback(Consumer<BuildingExplorationStateName> stateChangedCallback)
   {
      this.stateChangedCallback = stateChangedCallback;
   }

   public void setDebrisDetectedCallback(Runnable debrisDetectedCallback)
   {
      lookAndStepState.setDebrisDetectedCallback(debrisDetectedCallback);
   }

   public void setDoorDetectedCallback(Runnable doorDetectedCallback)
   {
      this.doorDetectedCallback = doorDetectedCallback;
   }

   public void setStairsDetectedCallback(Runnable stairsDetectedCallback)
   {
      lookAndStepState.setStairsDetectedCallback(stairsDetectedCallback);
   }

   public void ignoreDebris()
   {
      lookAndStepState.ignoreDebris();
      requestState(BuildingExplorationStateName.LOOK_AND_STEP);
   }

   public void proceedWithDoorBehavior()
   {
      walkThroughDoorState.proceedWithDoorBehavior();
   }

   public void start()
   {
      if (!isRunning.getAndSet(true))
      {
         LogTools.info("Starting behavior coordinator");

         doorLocationPacket.set(null);
         robotConfigurationData.set(null);

         stateMachineTask = executorService.scheduleAtFixedRate(this::update, 0, UPDATE_RATE_MILLIS, TimeUnit.MILLISECONDS);
      }
      else
      {
         LogTools.info("Start called but module is already running");
      }
   }

   public void stop()
   {
      if (isRunning.get())
      {
         LogTools.info("Stop requested. Shutting down on next update");
         stopRequested.set(true);
      }
   }

   @Override
   public void destroy()
   {
      toolboxWakerThread.destroy();
      executorService.shutdown();
   }

   private StateMachine<BuildingExplorationStateName, State> buildStateMachine()
   {
      StateMachineFactory<BuildingExplorationStateName, State> factory = new StateMachineFactory<>(BuildingExplorationStateName.class);

      factory.setNamePrefix("buildingExploration");
      factory.setRegistry(registry);
      factory.buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));

      factory.addState(BuildingExplorationStateName.TELEOP, teleopState);
      factory.addState(BuildingExplorationStateName.LOOK_AND_STEP, lookAndStepState);
      factory.addState(BuildingExplorationStateName.WALK_THROUGH_DOOR, walkThroughDoorState);
      factory.addState(BuildingExplorationStateName.TRAVERSE_STAIRS, traverseStairsState);

      factory.addDoneTransition(BuildingExplorationStateName.LOOK_AND_STEP, BuildingExplorationStateName.TELEOP);
      factory.addDoneTransition(BuildingExplorationStateName.WALK_THROUGH_DOOR, BuildingExplorationStateName.TELEOP);
      factory.addDoneTransition(BuildingExplorationStateName.TRAVERSE_STAIRS, BuildingExplorationStateName.TELEOP);

      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.LOOK_AND_STEP, requestedState);
      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.WALK_THROUGH_DOOR, requestedState);
      factory.addRequestedTransition(BuildingExplorationStateName.TELEOP, BuildingExplorationStateName.TRAVERSE_STAIRS, requestedState);

      factory.addRequestedTransition(BuildingExplorationStateName.LOOK_AND_STEP, BuildingExplorationStateName.TELEOP, requestedState);
      factory.addRequestedTransition(BuildingExplorationStateName.WALK_THROUGH_DOOR, BuildingExplorationStateName.TELEOP, requestedState);
      factory.addRequestedTransition(BuildingExplorationStateName.TRAVERSE_STAIRS, BuildingExplorationStateName.TELEOP, requestedState);

      factory.addTransition(BuildingExplorationStateName.LOOK_AND_STEP, BuildingExplorationStateName.WALK_THROUGH_DOOR, this::transitionToWalkThroughDoor);
      factory.addTransition(BuildingExplorationStateName.LOOK_AND_STEP, BuildingExplorationStateName.TELEOP, t -> lookAndStepState.getDebrisDetected());
      factory.addTransition(BuildingExplorationStateName.LOOK_AND_STEP, BuildingExplorationStateName.TRAVERSE_STAIRS, t -> lookAndStepState.getStairsDetected());

      return factory.build(BuildingExplorationStateName.TELEOP);
   }

   private void update()
   {
      if (stopRequested.getAndSet(false))
      {
         requestState(BuildingExplorationStateName.TELEOP);
         stateMachine.doActionAndTransition();
         helper.publishToController(new AbortWalkingMessage());

         new Thread(() ->
                    {
                       if (stateMachineTask != null)
                       {
                          stateMachineTask.cancel(true);
                          stateMachineTask = null;
                       }

                       isRunning.set(false);
                    }).start();
      }
      else
      {
         stateMachine.doActionAndTransition();
      }
   }

   private boolean transitionToWalkThroughDoor(double timeInState)
   {
      DoorLocationPacket doorLocationPacket = this.doorLocationPacket.get();
      RobotConfigurationData robotConfigurationData = this.robotConfigurationData.get();

      if (doorLocationPacket == null || robotConfigurationData == null)
      {
         return false;
      }

      Point3D doorPosition = doorLocationPacket.getDoorTransformToWorld().getPosition();
      Point3D robotRootJointPosition = new Point3D(robotConfigurationData.getRootTranslation());

      double xyDistanceToDoor = doorPosition.distanceXY(robotRootJointPosition);
      boolean doorDetected = xyDistanceToDoor <= xyProximityToDoorToStopWalking;
      if (doorDetected)
      {
         doorDetectedCallback.run();
      }

      return doorDetected;
   }

   public static void pitchChestToSeeDoor(RemoteSyncedRobotModel syncedRobot, BehaviorHelper helper)
   {
      syncedRobot.update();

      double desiredChestPitch = -0.017;
      double trajectoryTime = 2.0;

      FrameYawPitchRoll frameChestYawPitchRoll = new FrameYawPitchRoll(syncedRobot.getReferenceFrames().getChestFrame());
      frameChestYawPitchRoll.changeFrame(syncedRobot.getReferenceFrames().getPelvisZUpFrame());
      frameChestYawPitchRoll.setPitch(desiredChestPitch);
      frameChestYawPitchRoll.changeFrame(ReferenceFrame.getWorldFrame());

      ChestTrajectoryMessage message = new ChestTrajectoryMessage();
      message.getSo3Trajectory()
             .set(HumanoidMessageTools.createSO3TrajectoryMessage(trajectoryTime,
                                                                  frameChestYawPitchRoll,
                                                                  EuclidCoreTools.zeroVector3D,
                                                                  ReferenceFrame.getWorldFrame()));
      long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
      message.getSo3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);

      helper.publishToController(message);

      ThreadTools.sleepSeconds(trajectoryTime);
   }

   private static class WalkThroughDoorState implements State
   {
      private final BehaviorHelper helper;
      final AtomicBoolean isDone = new AtomicBoolean();
      final AtomicBoolean receivedOperatorConfirmation = new AtomicBoolean();
      final AtomicBoolean hasStartedBehavior = new AtomicBoolean();
      private final RemoteSyncedRobotModel syncedRobot;

      public WalkThroughDoorState(BehaviorHelper helper)
      {
         this.helper = helper;
         String robotName = helper.getRobotModel().getSimpleRobotName();

         syncedRobot = helper.newSyncedRobot();

         helper.subscribeToBehaviorStatusViaCallback(status ->
         {
            if (status == CurrentBehaviorStatus.NO_BEHAVIOR_RUNNING)
            {
               isDone.set(true);
            }
         });
      }

      @Override
      public void onEntry()
      {
         pitchChestToSeeDoor(syncedRobot, helper);

         receivedOperatorConfirmation.set(false);
         hasStartedBehavior.set(false);
         isDone.set(false);

         LogTools.info("Entering " + getClass().getSimpleName());
      }

      private void startBehavior()
      {
         helper.publishBehaviorType(HumanoidBehaviorType.WALK_THROUGH_DOOR);
      }

      @Override
      public void doAction(double timeInState)
      {
         if (receivedOperatorConfirmation.get() && !hasStartedBehavior.get())
         {
            startBehavior();
            hasStartedBehavior.set(true);

            LogTools.info("Sent packet to start " + HumanoidBehaviorType.WALK_THROUGH_DOOR);
         }
      }

      @Override
      public void onExit(double timeInState)
      {
         helper.publishBehaviorControlMode(BehaviorControlModeEnum.STOP);

         LogTools.info("Exiting " + getClass().getSimpleName());
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return isDone.get();
      }

      public void proceedWithDoorBehavior()
      {
         receivedOperatorConfirmation.set(true);
      }
   }

   private static class TraverseStairsState implements State
   {
      private final BehaviorHelper helper;
      private final AtomicBoolean isDone = new AtomicBoolean();
      private final Pose3DReadOnly bombPose;

      public TraverseStairsState(BehaviorHelper helper, Pose3DReadOnly bombPose)
      {
         this.helper = helper;
         this.bombPose = bombPose;

         helper.subscribeViaCallback(TraverseStairsBehaviorAPI.COMPLETED, completed -> isDone.set(true));
      }

      @Override
      public void onEntry()
      {
         LogTools.info("Entering " + getClass().getSimpleName());

         isDone.set(false);

         String behaviorName = TraverseStairsBehavior.DEFINITION.getName();
         helper.publish(BehaviorModule.API.BehaviorSelection, behaviorName);
         ThreadTools.sleep(100);

         helper.publish(TraverseStairsBehaviorAPI.GOAL_INPUT, new Pose3D(bombPose));
         ThreadTools.sleep(100);

         helper.publish(TraverseStairsBehaviorAPI.START);
      }

      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public void onExit(double timeInState)
      {
         LogTools.info("Exiting " + getClass().getSimpleName());
         helper.publish(BehaviorModule.API.BehaviorSelection, "null");
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return isDone.get();
      }
   }

   public LookAndStepBehavior getLookAndStepBehavior()
   {
      return lookAndStepBehavior;
   }
}
