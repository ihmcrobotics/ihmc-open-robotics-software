package us.ihmc.avatar.networkProcessor.continuousPlanningToolboxModule;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.BipedContinuousPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.ToolboxStateMessage;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class BipedContinuousPlanningToolboxController extends ToolboxController
{
   private static final boolean debug = false;
   private static final boolean verbose = false;

   private static final double defaultTransferDuration = 0.6;
   private static final double defaultSwingDuration = 1.0;

   private static final RobotSide defaultInitialSide = RobotSide.LEFT;

   private static final double proximityGoal = 1.0e-2;
   private static final int defaultStartPlanId = 13;

   private static final int defaultNumberOfStepsToLeaveUnchanged = 3;

   private final AtomicReference<FootstepPlanningToolboxOutputStatus> latestPlannerOutput = new AtomicReference<>();
   private final AtomicReference<FootstepPlanningToolboxOutputStatus> currentPlannerOutput = new AtomicReference<>();
   private final AtomicReference<BipedContinuousPlanningRequestPacket> planningRequestPacket = new AtomicReference<>();
   private final AtomicReference<List<FootstepStatusMessage>> footstepStatusBuffer = new AtomicReference<>(new ArrayList<>());
   private final AtomicReference<RobotSide> expectedInitialSteppingSide = new AtomicReference<>();

   private final AtomicReference<PlanarRegionsListMessage> latestPlanarRegions = new AtomicReference<>();

   private final SideDependentList<Pose3DReadOnly> currentFeetPositions = new SideDependentList<>();
   private final List<FootstepDataMessage> completedSteps = new ArrayList<>();
   private final List<FootstepDataMessage> fixedStepQueue = new ArrayList<>();
   private final List<FootstepDataMessage> stepQueue = new ArrayList<>();

   private final YoBoolean isInitialSegmentOfPlan = new YoBoolean("isInitialSegmentOfPlan", registry);
   private final YoInteger numberOfStepsToLeaveUnchanged = new YoInteger("numberOfStepsToLeaveUnchanged", registry);
   private final YoBoolean hasReachedGoal = new YoBoolean("hasReachedGoal", registry);
   private final YoBoolean planningFailed = new YoBoolean("planningFailed", registry);
   private final YoBoolean receivedPlanForLastRequest = new YoBoolean("receivedPlanForLastRequest", registry);
   private final YoBoolean waitingForPlan = new YoBoolean("waitingForPlan", registry);

   private final YoInteger broadcastPlanId = new YoInteger("broadcastPlanId", registry);
   private final YoInteger receivedPlanId = new YoInteger("receivedPlanId", registry);

   private final YoDouble transferDuration = new YoDouble("transferDuration", registry);
   private final YoDouble swingDuration = new YoDouble("swingDuration", registry);

   private final IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> planningRequestPublisher;
   private final IHMCRealtimeROS2Publisher<ToolboxStateMessage> plannerStatePublisher;
   private final FootstepPlannerParametersBasics plannerParameters;

   public BipedContinuousPlanningToolboxController(StatusMessageOutputManager statusOutputManager,
                                                   IHMCRealtimeROS2Publisher<FootstepPlanningRequestPacket> planningRequestPublisher,
                                                   IHMCRealtimeROS2Publisher<ToolboxStateMessage> plannerStatePublisher,
                                                   FootstepPlannerParametersBasics plannerParameters,
                                                   YoRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      this.planningRequestPublisher = planningRequestPublisher;
      this.plannerStatePublisher = plannerStatePublisher;
      this.plannerParameters = plannerParameters;

      broadcastPlanId.set(defaultStartPlanId);
      receivedPlanId.set(-1);

      numberOfStepsToLeaveUnchanged.set(defaultNumberOfStepsToLeaveUnchanged);

      transferDuration.set(defaultTransferDuration);
      swingDuration.set(defaultSwingDuration);
   }

   public void processFootstepPlannerOutput(FootstepPlanningToolboxOutputStatus message)
   {
      FootstepPlanningResult result = FootstepPlanningResult.fromByte(message.getFootstepPlanningResult());
      if (!result.validForExecution())
      {
         LogTools.info(("Planner result failed. Terminating because of " + result));
         resetForNewPlan();
         planningFailed.set(true);

         return;
      }
      latestPlannerOutput.set(message);
      receivedPlanId.set(message.getPlanId());
      waitingForPlan.set(false);
   }

   public void processContinuousPlanningRequest(BipedContinuousPlanningRequestPacket message)
   {
      resetForNewPlan();
      planningRequestPacket.set(message);
   }

   public void processFootstepStatusMessage(FootstepStatusMessage message)
   {
      if (FootstepStatus.fromByte(message.getFootstepStatus()) == FootstepStatus.COMPLETED)
      {
         List<FootstepStatusMessage> oldBuffer = footstepStatusBuffer.getAndSet(null);
         List<FootstepStatusMessage> newBuffer = new ArrayList<>();

         if (oldBuffer != null)
            newBuffer.addAll(oldBuffer);
         newBuffer.add(message);

         footstepStatusBuffer.set(newBuffer);

         Pose3D footPose = new Pose3D();
         footPose.getOrientation().set(message.getActualFootOrientationInWorld());
         footPose.getPosition().set(message.getActualFootPositionInWorld());
         currentFeetPositions.put(RobotSide.fromByte(message.getRobotSide()), footPose);

         if (verbose)
         {
            LogTools.info("Received completed step: " + RobotSide.fromByte(message.getRobotSide()) + " at " + message.getActualFootPositionInWorld());
         }
      }
   }

   public void processPlanarRegionListMessage(PlanarRegionsListMessage message)
   {
      latestPlanarRegions.set(message);
   }

   @Override
   public boolean initialize()
   {
      resetForNewPlan();

      computeAndSendLatestPlanningRequest();

      return true;
   }

   @Override
   public void updateInternal()
   {
      if (!clearCompletedSteps())
         return;

      checkIfLastPlanFulfilledRequest();

      if (!waitingForPlan.getBooleanValue() && !hasReachedGoal.getBooleanValue() && fixedStepQueue.size() < numberOfStepsToLeaveUnchanged.getValue())
      {
         if (!updateStepQueueFromNewPlan())
            return;

         if (!updateFixedStepQueue())
            return;

         updateHasReachedGoal();

         if (!hasReachedGoal.getBooleanValue())
         {
            broadcastLatestPlan();

            computeAndSendLatestPlanningRequest();
         }
      }
   }

   @Override
   public boolean isDone()
   {
      return hasReachedGoal.getBooleanValue() || planningFailed.getBooleanValue();
   }

   private void resetForNewPlan()
   {
      waitingForPlan.set(true);
      isInitialSegmentOfPlan.set(true);
      completedSteps.clear();

      planningFailed.set(false);
      hasReachedGoal.set(false);
      // TODO don't clear the fixed step queue.
      fixedStepQueue.clear();
      currentFeetPositions.clear();
      stepQueue.clear();
      footstepStatusBuffer.set(null);
      latestPlannerOutput.set(null);
      expectedInitialSteppingSide.set(defaultInitialSide);

      broadcastPlanId.set(defaultStartPlanId);
      receivedPlanId.set(-1);
   }

   private void computeAndSendLatestPlanningRequest()
   {
      FootstepPlanningRequestPacket planningRequestPacket = new FootstepPlanningRequestPacket();
      BipedContinuousPlanningRequestPacket continuousPlanningRequestPacket = this.planningRequestPacket.get();

      plannerStatePublisher.publish(MessageTools.createToolboxStateMessage(ToolboxState.WAKE_UP));

      // sleep for a minute to let the toolbox wake up.
      ThreadTools.sleep(100);

      SideDependentList<Pose3DReadOnly> startPositions = new SideDependentList<>();
      if (isInitialSegmentOfPlan.getBooleanValue())
      {
         /*
         if (robotDataReceiver != null)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               FramePose3D position = new FramePoint3D(robotDataReceiver.getReferenceFrames().getSoleFrame(robotSide));
               position.changeFrame(ReferenceFrame.getWorldFrame());

               startPositions.put(robotSide, position);
            }
         }
         else
         {
         */
            Pose3D leftFoot = new Pose3D();
            leftFoot.getPosition().set(continuousPlanningRequestPacket.getLeftStartPositionInWorld());
            leftFoot.getOrientation().set(continuousPlanningRequestPacket.getLeftStartOrientationInWorld());
            Pose3D rightFoot = new Pose3D();
            rightFoot.getPosition().set(continuousPlanningRequestPacket.getRightStartPositionInWorld());
            rightFoot.getOrientation().set(continuousPlanningRequestPacket.getRightStartOrientationInWorld());

            startPositions.put(RobotSide.LEFT, leftFoot);
            startPositions.put(RobotSide.RIGHT, rightFoot);
//         }

         for (RobotSide robotSide : RobotSide.values)
            currentFeetPositions.put(robotSide, startPositions.get(robotSide));
      }
      else
      {
         int numberOfFixedSteps = fixedStepQueue.size();
         if (fixedStepQueue.size() > 2)
         {
            Pose3D footPose = new Pose3D();
            FootstepDataMessage footstep = fixedStepQueue.get(numberOfFixedSteps - 2);
            footPose.getPosition().set(footstep.getLocation());
            footPose.getOrientation().set(footstep.getOrientation());
            currentFeetPositions.put(RobotSide.fromByte(footstep.getRobotSide()), footPose);
         }

         if (fixedStepQueue.size() > 1)
         {
            Pose3D footPose = new Pose3D();
            FootstepDataMessage footstep = fixedStepQueue.get(numberOfFixedSteps - 1);
            footPose.getPosition().set(footstep.getLocation());
            footPose.getOrientation().set(footstep.getOrientation());
            currentFeetPositions.put(RobotSide.fromByte(footstep.getRobotSide()), footPose);
         }
      }

      if (fixedStepQueue.size() > 0)
      {
         FootstepDataMessage finalStep = fixedStepQueue.get(fixedStepQueue.size() - 1);
         expectedInitialSteppingSide.set(RobotSide.fromByte(finalStep.getRobotSide()).getOppositeSide());
      }

      RobotSide initialSupportSide = expectedInitialSteppingSide.get().getOppositeSide();

      planningRequestPacket.setRequestedInitialStanceSide(initialSupportSide.toByte());
      planningRequestPacket.setHorizonLength(continuousPlanningRequestPacket.getHorizonLength());
      planningRequestPacket.setPlanBodyPath(true);
      planningRequestPacket.setPerformAStarSearch(true);
      planningRequestPacket.getStartLeftFootPose().set(currentFeetPositions.get(RobotSide.LEFT));
      planningRequestPacket.getStartRightFootPose().set(currentFeetPositions.get(RobotSide.RIGHT));
      planningRequestPacket.setTimeout(continuousPlanningRequestPacket.getTimeout());
      planningRequestPacket.getPlanarRegionsListMessage().set(latestPlanarRegions.get());

      planningRequestPacket.getGoalLeftFootPose().set(continuousPlanningRequestPacket.getGoalPositionInWorld(), continuousPlanningRequestPacket.getGoalOrientationInWorld());
      planningRequestPacket.getGoalRightFootPose().set(continuousPlanningRequestPacket.getGoalPositionInWorld(), continuousPlanningRequestPacket.getGoalOrientationInWorld());
      planningRequestPacket.getGoalLeftFootPose().appendTranslation(0.0, 0.5 * plannerParameters.getIdealFootstepWidth(), 0.0);
      planningRequestPacket.getGoalRightFootPose().appendTranslation(0.0, - 0.5 * plannerParameters.getIdealFootstepWidth(), 0.0);

      broadcastPlanId.increment();
      planningRequestPacket.setPlannerRequestId(broadcastPlanId.getIntegerValue());

      waitingForPlan.set(true);
      planningRequestPublisher.publish(planningRequestPacket);

      if (debug || verbose)
      {
         LogTools.info("Planning #" + broadcastPlanId.getValue() + " to " + continuousPlanningRequestPacket.getGoalPositionInWorld() + ", starting with " + expectedInitialSteppingSide
               .get());
      }
   }


   private boolean clearCompletedSteps()
   {
      List<FootstepStatusMessage> statusMessages = this.footstepStatusBuffer.getAndSet(null);
      if (statusMessages == null)
         return true;

      for (FootstepStatusMessage footstepStatusMessage : statusMessages)
      {
         if (FootstepStatus.fromByte(footstepStatusMessage.getFootstepStatus()) == FootstepStatus.COMPLETED)
         {
            FootstepDataMessage stepToRemove = getStepFromQueue(footstepStatusMessage, fixedStepQueue);
            if (stepToRemove == null)
            {
               stepToRemove = getStepFromQueue(footstepStatusMessage, stepQueue);
            }
            else
            {
               fixedStepQueue.remove(stepToRemove);
               completedSteps.add(stepToRemove);

               continue;
            }


            if (stepToRemove == null)
            {
               LogTools.warn("The completed step was not in the queue. Killing the module.");
               resetForNewPlan();
               planningFailed.set(true);

               return false;
            }

            stepQueue.remove(stepToRemove);
            completedSteps.add(stepToRemove);
         }
      }

      return true;
   }

   private FootstepDataMessage getStepFromQueue(FootstepStatusMessage statusMessage, List<FootstepDataMessage> stepList)
   {
      for (FootstepDataMessage step : stepList)
      {
         if (step.getRobotSide() != statusMessage.getRobotSide())
            continue;

         if (!step.getLocation().epsilonEquals(statusMessage.getDesiredFootPositionInWorld(), 1e-2))
            continue;

         if (!step.getOrientation().epsilonEquals(statusMessage.getDesiredFootOrientationInWorld(), 1e-2))
            continue;

         return step;
      }

      return null;
   }

   private void checkIfLastPlanFulfilledRequest()
   {
      FootstepPlanningToolboxOutputStatus plannerOutput = currentPlannerOutput.get();
      if (plannerOutput == null)
      {
         receivedPlanForLastRequest.set(false);
         return;
      }

      receivedPlanForLastRequest.set(broadcastPlanId.getIntegerValue() == receivedPlanId.getIntegerValue());
   }

   private boolean updateStepQueueFromNewPlan()
   {
      FootstepPlanningToolboxOutputStatus plannerOutput = latestPlannerOutput.getAndSet(null);
      if (plannerOutput == null)
         return false;

      // ignore this plan, it's out of date
      if (plannerOutput.getPlanId() != broadcastPlanId.getValue())
         return false;

      currentPlannerOutput.set(plannerOutput);
      List<FootstepDataMessage> stepList = plannerOutput.getFootstepDataList().getFootstepDataList();
      stepQueue.clear();
      for (int i = 0; i < stepList.size(); i++)
      {
         FootstepDataMessage step = new FootstepDataMessage(stepList.get(i));
         step.setTransferDuration(transferDuration.getDoubleValue());
         step.setSwingDuration(swingDuration.getDoubleValue());
         stepQueue.add(step);
      }

      if (stepQueue.get(0).getRobotSide() != expectedInitialSteppingSide.get().toByte())
      {
         LogTools.error("Got a plan back starting with the wrong first step. Should start with " + expectedInitialSteppingSide.get() + ", but actually starts with " +
                              RobotSide.fromByte(stepQueue.get(0).getRobotSide()));
         resetForNewPlan();
         planningFailed.set(true);

         return false;
      }

      isInitialSegmentOfPlan.set(false);

      return true;
   }

   private boolean updateFixedStepQueue()
   {
      if (fixedStepQueue.size() > numberOfStepsToLeaveUnchanged.getValue() || stepQueue.size() == 0)
         return false;

      while (fixedStepQueue.size() < numberOfStepsToLeaveUnchanged.getIntegerValue() && stepQueue.size() > 0)
      {
         FootstepDataMessage footstepToAdd = stepQueue.remove(0);
         if (fixedStepQueue.size() > 0)
         {
            byte lastSide = fixedStepQueue.get(fixedStepQueue.size() - 1).getRobotSide();
            if (lastSide == footstepToAdd.getRobotSide())
            {
               LogTools.error("The next side when appending would be out of order. The fixed queue size " + fixedStepQueue.size() +
                                    " ends with a step on the " + RobotSide.fromByte(lastSide) + ", meaning it should start with " +
                                    RobotSide.fromByte(lastSide).getOppositeSide() + ". \nIt actually starts with " +
                                    RobotSide.fromByte(footstepToAdd.getRobotSide()) + ", but " + expectedInitialSteppingSide.get() + " was requested for starting.");
               resetForNewPlan();
               planningFailed.set(true);

               return false;
            }
         }
         fixedStepQueue.add(footstepToAdd);
      }

      return true;
   }

   private void broadcastLatestPlan()
   {
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      // TODO make an output status thing and report it
      for (int i = 0; i < fixedStepQueue.size(); i++)
         footstepDataListMessage.getFootstepDataList().add().set(fixedStepQueue.get(i));
      for (int i = 0; i < stepQueue.size(); i++)
         footstepDataListMessage.getFootstepDataList().add().set(stepQueue.get(i));

      reportMessage(footstepDataListMessage);
   }

   private void updateHasReachedGoal()
   {
      boolean isPlannedGoalTheFinalGoal = false;
      boolean doesThePlanReachTheFinalGoal = false;
      if (currentPlannerOutput.get() != null)
      {
         isPlannedGoalTheFinalGoal = currentPlannerOutput.get().getGoalPose().getPosition().distanceXY(planningRequestPacket.get().getGoalPositionInWorld()) < proximityGoal;
         doesThePlanReachTheFinalGoal = FootstepPlanningResult.fromByte(currentPlannerOutput.get().getFootstepPlanningResult()) != FootstepPlanningResult.PLANNING;
      }
      if (!isInitialSegmentOfPlan.getBooleanValue() && isPlannedGoalTheFinalGoal && doesThePlanReachTheFinalGoal && stepQueue.isEmpty())
         hasReachedGoal.set(true);
      else
         hasReachedGoal.set(false);
   }
}
