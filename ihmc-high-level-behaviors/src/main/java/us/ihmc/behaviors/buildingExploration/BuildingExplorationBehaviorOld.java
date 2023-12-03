package us.ihmc.behaviors.buildingExploration;

import controller_msgs.msg.dds.AbortWalkingMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import perception_msgs.msg.dds.DoorLocationPacket;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.fiducialDetectorToolBox.FiducialDetectorToolboxModule;
import us.ihmc.avatar.networkProcessor.objectDetectorToolBox.ObjectDetectorToolboxModule;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.behaviorTree.ResettingNode;
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
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.tools.Destroyable;
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

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.REACHED_GOAL;

/**
 * This was used for the 2019 Atlas building exploration demo. It is called old because
 * an upgrade was planned but never finished.
 * @deprecated Not supported right now. Being kept for reference or revival.
 */
public class BuildingExplorationBehaviorOld extends ResettingNode implements Destroyable
{
   private static final int UPDATE_RATE_MILLIS = 50;
   private final static Pose3D NAN_POSE = new Pose3D();
   static
   {
      NAN_POSE.setToNaN();
   }
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
   private final ROS2SyncedRobotModel syncedRobot;
   private ScheduledFuture<?> stateMachineTask = null;
   private PausablePeriodicThread toolboxWakerThread;

   private Consumer<BuildingExplorationStateName> stateChangedCallback = state -> {};
   private Runnable doorDetectedCallback = () -> {};

   private final BuildingExplorationBehaviorTeleopState teleopState;
   private final BuildingExplorationBehaviorLookAndStepState lookAndStepState;
   private final BuildingExplorationBehaviorWalkThroughDoorState walkThroughDoorState;
   private final BuildingExplorationBehaviorTraverseStairsState traverseStairsState;
   private final AtomicReference<Pose3D> goal = new AtomicReference<>(NAN_POSE);

   public BuildingExplorationBehaviorOld(BehaviorHelper helper)
   {
      this.helper = helper;
      LogTools.info("Constructing");
      executorService = Executors.newSingleThreadScheduledExecutor();
      lookAndStepBehavior = new LookAndStepBehavior(helper);
      teleopState = new BuildingExplorationBehaviorTeleopState(helper);
      lookAndStepState = new BuildingExplorationBehaviorLookAndStepState(this, helper, bombPose);
      walkThroughDoorState = new BuildingExplorationBehaviorWalkThroughDoorState(helper);
      traverseStairsState = new BuildingExplorationBehaviorTraverseStairsState(helper, bombPose);

      getChildren().add(lookAndStepBehavior);

      syncedRobot = helper.newSyncedRobot();
//      helper.subscribeViaCallback(Goal, this::setGoal);
      helper.subscribeViaCallback(REACHED_GOAL, () -> setGoal(NAN_POSE));
//      helper.subscribeViaCallback(RequestedState, this::requestState);
//      AtomicReference<BuildingExplorationStateName> requestedState = helper.subscribeViaReference(RequestedState, BuildingExplorationStateName.TELEOP);

//      helper.subscribeViaCallback(Start, start ->
//      {
//         LogTools.info("Starting");
//         setBombPose(goal.get());
//         requestState(requestedState.get());
//         start();
//      });
//      helper.subscribeViaCallback(Stop, s -> stop());
//      setStateChangedCallback(newState -> helper.publish(CurrentState, newState));
//      setDebrisDetectedCallback(() -> helper.publish(DebrisDetected, true));
//      setStairsDetectedCallback(() -> helper.publish(StairsDetected, true));
//      setDoorDetectedCallback(() -> helper.publish(DoorDetected, true));
//      helper.subscribeViaCallback(IgnoreDebris, ignore -> ignoreDebris());
//      helper.subscribeViaCallback(ConfirmDoor, confirm -> proceedWithDoorBehavior());
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

   private void setGoal(Pose3D newGoal)
   {
      goal.set(newGoal);
      if (!newGoal.containsNaN())
         lookAndStepBehavior.acceptGoal(newGoal);
//      helper.publish(GoalForUI, goal.get());
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
   public BehaviorTreeNodeStatus determineStatus()
   {
      syncedRobot.update();

      if (goal.get() != null)
      {
         if (lookAndStepBehavior.isReset())
            lookAndStepBehavior.acceptGoal(goal.get());
         return lookAndStepBehavior.tickAndGetStatus();
      }

      return BehaviorTreeNodeStatus.RUNNING;
   }

   @Override
   public void reset()
   {

   }

   public String getName()
   {
      return "Building Exploration";
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

   public void update()
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
      Point3D robotRootJointPosition = new Point3D(robotConfigurationData.getRootPosition());

      double xyDistanceToDoor = doorPosition.distanceXY(robotRootJointPosition);
      boolean doorDetected = xyDistanceToDoor <= xyProximityToDoorToStopWalking;
      if (doorDetected)
      {
         doorDetectedCallback.run();
      }

      return doorDetected;
   }

   public static void pitchChestToSeeDoor(ROS2SyncedRobotModel syncedRobot, BehaviorHelper helper)
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

   public LookAndStepBehavior getLookAndStepBehavior()
   {
      return lookAndStepBehavior;
   }
}
