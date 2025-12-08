package us.ihmc.behaviors.activeMapping;

import behavior_msgs.msg.dds.ContinuousHikingCommandMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine.*;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.communication.ContinuousHikingAPI;
import us.ihmc.footstepPlanning.graphSearch.EnvironmentHandler;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * This class is responsible for scheduling the continuous hiking state machine. It is responsible for handling the state transitions and the logic of the
 * state machine.
 */
public class ContinuousPlanningStateMachine
{
   /**
    * This is the delay between each tick of the state machine. Set based on perception update rate.
    */
   private final static long CONTINUOUS_PLANNING_DELAY_MS = 16;
   protected final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                          getClass(),
                                                                                                          ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);
   public StateMachine<ContinuousHikingState, State> stateMachine;
   private EnvironmentHandler environmentHandler;

   private final AtomicBoolean resetStateMachine = new AtomicBoolean(false);

   public ContinuousPlanningStateMachine(DRCRobotModel robotModel,
                                         ROS2Node ros2Node,
                                         ROS2SyncedRobotModel syncedRobotModel,
                                         HumanoidReferenceFrames referenceFrames,
                                         ControllerFootstepQueueMonitor controllerFootstepQueueMonitor,
                                         ActiveMappingParameterToolBox activeMappingParameterObject)
   {
      String simpleRobotName = robotModel.getSimpleRobotName();

      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

      ros2Helper.subscribeViaCallback(ContinuousHikingAPI.RESET_STATE_MACHINE, this::resetStateMachine);

      AtomicReference<ContinuousHikingCommandMessage> commandMessage = new AtomicReference<>(new ContinuousHikingCommandMessage());
      ros2Helper.subscribeViaCallback(ContinuousHikingAPI.CONTINUOUS_HIKING_COMMAND, commandMessage::set);

      ContinuousHikingLogger continuousHikingLogger = new ContinuousHikingLogger();
      TerrainPlanningDebugger debugger = new TerrainPlanningDebugger(ros2Node, activeMappingParameterObject.getMonteCarloPlannerParameters());
      ContinuousPlanner continuousPlanner = new ContinuousPlanner(robotModel,
                                                                  referenceFrames,
                                                                  commandMessage,
                                                                  activeMappingParameterObject,
                                                                  debugger,
                                                                  continuousHikingLogger);

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());

      StateMachineFactory<ContinuousHikingState, State> stateMachineFactory = new StateMachineFactory<>(ContinuousHikingState.class);
      stateMachineFactory.setNamePrefix("ContinuousHikingStateMachine");
      stateMachineFactory.setRegistry(registry);

      // Create the different states
      State notStartedState = new DoNothingState(ros2Helper, syncedRobotModel, simpleRobotName, continuousPlanner, debugger);
      State readyToPlanState = new ReadyToPlanState(ros2Helper,
                                                    referenceFrames,
                                                    commandMessage,
                                                    continuousPlanner,
                                                    controllerFootstepQueueMonitor,
                                                    activeMappingParameterObject.getContinuousHikingParameters(),
                                                    this::getEnvironmentHandler,
                                                    debugger,
                                                    continuousHikingLogger);
      State waitingtoLandState = new WaitingToLandState(ros2Helper,
                                                        simpleRobotName,
                                                        continuousPlanner,
                                                        controllerFootstepQueueMonitor,
                                                        activeMappingParameterObject.getContinuousHikingParameters(),
                                                        continuousHikingLogger);
      State justWaitState = new JustWaitState(robotModel,
                                              ros2Helper,
                                              syncedRobotModel,
                                              commandMessage,
                                              controllerFootstepQueueMonitor,
                                              debugger,
                                              activeMappingParameterObject,
                                              this::getEnvironmentHandler);

      // Adding the different states
      stateMachineFactory.addState(ContinuousHikingState.DO_NOTHING, notStartedState);
      stateMachineFactory.addState(ContinuousHikingState.READY_TO_PLAN, readyToPlanState);
      stateMachineFactory.addState(ContinuousHikingState.WAITING_TO_LAND, waitingtoLandState);
      stateMachineFactory.addState(ContinuousHikingState.JUST_WAIT, justWaitState);

      // Create different conditions
      StartContinuousHikingTransitionCondition startContinuousHikingTransitionCondition = new StartContinuousHikingTransitionCondition(commandMessage);
      StopContinuousHikingTransitionCondition stopContinuousHikingTransitionCondition = new StopContinuousHikingTransitionCondition(commandMessage);
      PlanAgainTransitionCondition planAgainTransitionCondition = new PlanAgainTransitionCondition(continuousPlanner,
                                                                                                   activeMappingParameterObject.getContinuousHikingParameters());
      SquareUpTransitionCondition squareUpTransitionCondition = new SquareUpTransitionCondition(commandMessage);

      //NOTE: The transitions for the state machine are checked in the order they are added.
      // And once one condition is true, the others don't get checked.
      // To be able to always stop the state machine, we add the stop condition first

      // From any given state we can go back to DO_NOTHING and stop ContinuousHiking
      stateMachineFactory.addTransition(ContinuousHikingState.WAITING_TO_LAND, ContinuousHikingState.DO_NOTHING, stopContinuousHikingTransitionCondition);
      stateMachineFactory.addTransition(ContinuousHikingState.READY_TO_PLAN, ContinuousHikingState.DO_NOTHING, stopContinuousHikingTransitionCondition);

      // Add condition, this triggers the state machine to start Continuous Hiking
      stateMachineFactory.addTransition(ContinuousHikingState.DO_NOTHING, ContinuousHikingState.READY_TO_PLAN, startContinuousHikingTransitionCondition);

      // Add condition, this allows us to plan over and over again without sending any footsteps to the controller
      stateMachineFactory.addTransition(ContinuousHikingState.READY_TO_PLAN, ContinuousHikingState.READY_TO_PLAN, planAgainTransitionCondition);

      // Add condition, this allows us to prepare to square the robot up to the goal
      stateMachineFactory.addTransition(ContinuousHikingState.READY_TO_PLAN, ContinuousHikingState.JUST_WAIT, squareUpTransitionCondition);

      // Add done conditions to go into the next state
      stateMachineFactory.addDoneTransition(ContinuousHikingState.READY_TO_PLAN, ContinuousHikingState.WAITING_TO_LAND);
      stateMachineFactory.addDoneTransition(ContinuousHikingState.WAITING_TO_LAND, ContinuousHikingState.READY_TO_PLAN);
      stateMachineFactory.addDoneTransition(ContinuousHikingState.JUST_WAIT, ContinuousHikingState.DO_NOTHING);

      // Added a couple listeners to help when jumping between states
      stateMachine = stateMachineFactory.build(ContinuousHikingState.DO_NOTHING);

      stateMachine.addPreTransitionCallback(() ->
                                            {
                                               if (controllerFootstepQueueMonitor.pollRobotFalling() || resetStateMachine.get())
                                               {
                                                  LogTools.info("---- Resetting State Machine for Continuous Hiking ----");
                                                  stateMachine.resetToInitialState();
                                                  resetStateMachine.set(false);
                                                  commandMessage.get().setEnableContinuousHiking(false);
                                               }
                                            });

      stateMachineFactory.addStateChangedListener((from, to) ->
                                                  {
                                                     if (from == null)
                                                     {
                                                        // This means the state machine has just started up, for the visuals put them under the feet till we start walking
                                                        SideDependentList<FramePose3D> robotFeet = new SideDependentList<>(new FramePose3D(),
                                                                                                                           new FramePose3D());
                                                        robotFeet.get(RobotSide.LEFT)
                                                                 .set(referenceFrames.getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame());
                                                        robotFeet.get(RobotSide.RIGHT)
                                                                 .set(referenceFrames.getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame());
                                                        debugger.publishStartAndGoalForVisualization(robotFeet, robotFeet);
                                                     }

                                                     String message = "STATE CHANGED: (" + from + " -> " + to + ")";
                                                     LogTools.warn(message);
                                                     continuousHikingLogger.appendString(message);
                                                  });
      stateMachineFactory.addStateChangedListener((from, to) -> continuousHikingLogger.logToFile(true, false));

      executorService.scheduleWithFixedDelay(this::tickStateMachine, 1500, CONTINUOUS_PLANNING_DELAY_MS, TimeUnit.MILLISECONDS);
   }

   private void resetStateMachine()
   {
      resetStateMachine.set(true);
   }

   public EnvironmentHandler getEnvironmentHandler()
   {
      return environmentHandler;
   }

   /**
    * Runs the continuous hiking state machine every {@link #CONTINUOUS_PLANNING_DELAY_MS} milliseconds.
    * The state is stored in the {@link ContinuousHikingState}
    */
   private void tickStateMachine()
   {
      stateMachine.doActionAndTransition();
   }

   public void setLatestEnvironmentHandler(EnvironmentHandler environmentHandler)
   {
      this.environmentHandler = environmentHandler;
   }

   public void destroy()
   {
      executorService.shutdown();
   }
}