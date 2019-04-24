package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningRequestPacketPubSubType;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatusPubSubType;
import controller_msgs.msg.dds.ToolboxStateMessage;
import controller_msgs.msg.dds.ToolboxStateMessagePubSubType;
import controller_msgs.msg.dds.WalkToGoalBehaviorPacket;
import controller_msgs.msg.dds.WalkToGoalBehaviorPacketPubSubType;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.tools.FootstepPlannerMessageTools;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.FootstepListBehavior;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkToGoalBehavior extends AbstractBehavior
{
   enum WalkToGoalBehaviorStates
   {
      WAITING_FOR_REQUEST, PLANNING, EXECUTING_PLAN
   }

   private final ConcurrentListeningQueue<WalkToGoalBehaviorPacket> walkToGoalPacketQueue = new ConcurrentListeningQueue<>(20);
   private final ConcurrentListeningQueue<FootstepPlanningToolboxOutputStatus> planningOutputStatusQueue = new ConcurrentListeningQueue<>(5);

   private final StateMachine<WalkToGoalBehaviorStates, State> stateMachine;

   private final HumanoidReferenceFrames referenceFrames;

   private final FootstepListBehavior footstepListBehavior;

   private final YoBoolean isDone;
   private final YoBoolean havePlanToExecute;
   private final YoBoolean transitionBackToWaitingState;

   private FootstepDataListMessage planToExecute;
   private final IHMCROS2Publisher<ToolboxStateMessage> toolboxStatePublisher;
   private final IHMCROS2Publisher<FootstepPlanningRequestPacket> planningRequestPublisher;

   public WalkToGoalBehavior(String robotName, Ros2Node ros2Node, HumanoidReferenceFrames referenceFrames,
                             WalkingControllerParameters walkingControllerParameters, YoDouble yoTime)
   {
      super(robotName, ros2Node);

      this.referenceFrames = referenceFrames;

      footstepListBehavior = new FootstepListBehavior(robotName, ros2Node, walkingControllerParameters);
      registry.addChild(footstepListBehavior.getYoVariableRegistry());

      isDone = new YoBoolean("isDone", registry);
      havePlanToExecute = new YoBoolean("havePlanToExecute", registry);
      transitionBackToWaitingState = new YoBoolean("transitionBackToWaitingState", registry);

      createBehaviorInputSubscriber(WalkToGoalBehaviorPacket.class, walkToGoalPacketQueue::put);
      createSubscriber(FootstepPlanningToolboxOutputStatus.class, footstepPlanningToolboxPubGenerator, planningOutputStatusQueue::put);

      toolboxStatePublisher = createPublisher(ToolboxStateMessage.class, footstepPlanningToolboxSubGenerator);
      planningRequestPublisher = createPublisher(FootstepPlanningRequestPacket.class, footstepPlanningToolboxSubGenerator);

      stateMachine = setupStateMachine(yoTime);
   }

   private StateMachine<WalkToGoalBehaviorStates, State> setupStateMachine(DoubleProvider timeProvider)
   {
      StateMachineFactory<WalkToGoalBehaviorStates, State> factory = new StateMachineFactory<>(WalkToGoalBehaviorStates.class);
      factory.setNamePrefix("walkToGoalBehaviorStateMachine").setRegistry(registry).buildYoClock(timeProvider);

      factory.addState(WalkToGoalBehaviorStates.WAITING_FOR_REQUEST, new WaitingForRequestState());
      factory.addState(WalkToGoalBehaviorStates.PLANNING, new PlanningState());
      factory.addState(WalkToGoalBehaviorStates.EXECUTING_PLAN, new ExecutingPlanState());

      factory.addTransition(WalkToGoalBehaviorStates.WAITING_FOR_REQUEST, WalkToGoalBehaviorStates.PLANNING, t -> walkToGoalPacketQueue.isNewPacketAvailable());
      factory.addTransition(WalkToGoalBehaviorStates.PLANNING, WalkToGoalBehaviorStates.EXECUTING_PLAN, t -> havePlanToExecute.getBooleanValue());
      factory.addTransition(WalkToGoalBehaviorStates.PLANNING, WalkToGoalBehaviorStates.WAITING_FOR_REQUEST,
                            t -> transitionBackToWaitingState.getBooleanValue());
      factory.addTransition(WalkToGoalBehaviorStates.EXECUTING_PLAN, WalkToGoalBehaviorStates.WAITING_FOR_REQUEST, t -> footstepListBehavior.isDone());

      return factory.build(WalkToGoalBehaviorStates.WAITING_FOR_REQUEST);
   }

   @Override
   public void doControl()
   {
      stateMachine.doActionAndTransition();
   }

   @Override
   public void onBehaviorEntered()
   {
      walkToGoalPacketQueue.clear();
      planningOutputStatusQueue.clear();
      havePlanToExecute.set(false);
   }

   @Override
   public void onBehaviorAborted()
   {

   }

   @Override
   public void onBehaviorPaused()
   {
      if (stateMachine.getCurrentStateKey().equals(WalkToGoalBehaviorStates.EXECUTING_PLAN))
         footstepListBehavior.pause();
   }

   @Override
   public void onBehaviorResumed()
   {
      if (stateMachine.getCurrentStateKey().equals(WalkToGoalBehaviorStates.EXECUTING_PLAN))
         footstepListBehavior.resume();
   }

   @Override
   public void onBehaviorExited()
   {

   }

   @Override
   public boolean isDone(double timeinState)
   {
      return isDone.getBooleanValue();
   }

   class WaitingForRequestState implements State
   {
      @Override
      public void doAction(double timeInState)
      {
         // Waiting for plan request
      }

      @Override
      public void onEntry()
      {
         // Make sure there aren't any old plan requests hanging around
         walkToGoalPacketQueue.clear();
         isDone.set(true);
         transitionBackToWaitingState.set(false);
         havePlanToExecute.set(false);
      }

      @Override
      public void onExit()
      {
      }
   }

   class PlanningState implements State
   {
      @Override
      public void doAction(double timeInState)
      {
         // Wait for plan
         boolean newPacketAvailable = planningOutputStatusQueue.isNewPacketAvailable();

         if (newPacketAvailable)
         {
            FootstepPlanningToolboxOutputStatus latestPacket = planningOutputStatusQueue.getLatestPacket();
            boolean validForExecution = FootstepPlanningResult.fromByte(latestPacket.getFootstepPlanningResult()).validForExecution();
            if (validForExecution)
            {
               planToExecute = latestPacket.getFootstepDataList();
               havePlanToExecute.set(true);
            }
            else
            {
               transitionBackToWaitingState.set(true);
            }
         }
      }

      private final Pose3D tempFinalPose = new Pose3D();

      @Override
      public void onEntry()
      {
         planningOutputStatusQueue.clear();
         isDone.set(false);

         toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));
         toolboxStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.REINITIALIZE));

         WalkToGoalBehaviorPacket walkToGoalBehaviorPacket = walkToGoalPacketQueue.poll();
         referenceFrames.updateFrames();
         RobotSide goalSide = RobotSide.fromByte(walkToGoalBehaviorPacket.getGoalRobotSide());
         FramePose3D initialPose = new FramePose3D(referenceFrames.getSoleFrame(goalSide));
         tempFinalPose.setToZero();
         tempFinalPose.setX(walkToGoalBehaviorPacket.getXGoal());
         tempFinalPose.setY(walkToGoalBehaviorPacket.getYGoal());
         tempFinalPose.setOrientationYawPitchRoll(walkToGoalBehaviorPacket.getThetaGoal(), 0.0, 0.0);
         FramePose3D finalPose = new FramePose3D(ReferenceFrame.getWorldFrame(), tempFinalPose);
         FootstepPlanningRequestPacket tempPlanningRequestPacket = FootstepPlannerMessageTools.createFootstepPlanningRequestPacket(initialPose, goalSide, finalPose);
         tempPlanningRequestPacket.setTimeout(3.0);
         tempPlanningRequestPacket.setDestination(PacketDestination.FOOTSTEP_PLANNING_TOOLBOX_MODULE.ordinal());

         planningRequestPublisher.publish(tempPlanningRequestPacket);
      }

      @Override
      public void onExit()
      {

      }
   }

   class ExecutingPlanState implements State
   {
      @Override
      public void doAction(double timeInState)
      {
         footstepListBehavior.doControl();
      }

      @Override
      public void onEntry()
      {
         footstepListBehavior.initialize();
         footstepListBehavior.set(planToExecute);
      }

      @Override
      public void onExit()
      {
         footstepListBehavior.doPostBehaviorCleanup();
         isDone.set(true);
      }
   }
}
