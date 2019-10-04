package us.ihmc.avatar.networkProcessor.continuousPlanningToolboxModule;

import controller_msgs.msg.dds.*;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class BipedContinuousPlanningToolboxController extends ToolboxController
{
   private static final RobotSide defaultInitialSide = RobotSide.LEFT;

   private static final double proximityGoal = 1.0e-2;
   private static final int defaultStartPlanId = 13;

   private static final int defaultNumberOfStepsToLeaveUnchanged = 3;

   private final AtomicReference<FootstepPlanningToolboxOutputStatus> latestPlannerOutput = new AtomicReference<>();
   private final AtomicReference<FootstepPlanningToolboxOutputStatus> currentPlannerOutput = new AtomicReference<>();
   private final AtomicReference<BipedContinuousPlanningRequestPacket> planningRequestPacket = new AtomicReference<>();
   private final AtomicReference<List<FootstepStatusMessage>> footstepStatusBuffer = new AtomicReference<>(new ArrayList<>());
   private final AtomicReference<RobotSide> expectedInitialSide= new AtomicReference<>();

   private final AtomicReference<PlanarRegionsListMessage> latestPlanarRegions = new AtomicReference<>();

   private final SideDependentList<Point3DReadOnly> currentFeetPositions = new SideDependentList<>();
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

   public BipedContinuousPlanningToolboxController(StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry)
   {
      super(statusOutputManager, parentRegistry);

      broadcastPlanId.set(defaultStartPlanId);
      receivedPlanId.set(-1);

      numberOfStepsToLeaveUnchanged.set(defaultNumberOfStepsToLeaveUnchanged);
   }

   public void processFootstepPlannerOutput(FootstepPlanningToolboxOutputStatus message)
   {
      FootstepPlanningResult result = FootstepPlanningResult.fromByte(message.getFootstepPlanningResult());
      if (!result.validForExecution())
      {
         LogTools.info(("Planner result failed. Terminating because of " + result));
         resetForNewPlan();
         planningFailed.set(true);
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
      List<FootstepStatusMessage> messages = footstepStatusBuffer.get();
      messages.add(message);
      footstepStatusBuffer.set(messages);
      if (FootstepStatus.fromByte(message.getFootstepStatus()) == FootstepStatus.COMPLETED)
      {
         currentFeetPositions.put(RobotSide.fromByte(message.getRobotSide()), message.getActualFootPositionInWorld());
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
      return false;
   }

   private void resetForNewPlan()
   {
      waitingForPlan.set(true);
      isInitialSegmentOfPlan.set(true);

      planningFailed.set(false);
      hasReachedGoal.set(false);
      // TODO don't clear the fixed step queue.
      fixedStepQueue.clear();
      currentFeetPositions.clear();
      stepQueue.clear();
      footstepStatusBuffer.set(new ArrayList<>());
      latestPlanarRegions.set(null);
      expectedInitialSide.set(defaultInitialSide);

      broadcastPlanId.set(defaultStartPlanId);
      receivedPlanId.set(-1);
   }

   private void computeAndSendLatestPlanningRequest()
   {
      // TODO
   }


   private boolean clearCompletedSteps()
   {
      List<FootstepStatusMessage> footstepStatusBuffer = this.footstepStatusBuffer.get();

      for (FootstepStatusMessage footstepStatusMessage : footstepStatusBuffer)
      {
         if (footstepStatusMessage.getFootstepStatus() == FootstepStatusMessage.FOOTSTEP_STATUS_COMPLETED)
         {
            FootstepDataMessage stepToRemove = fixedStepQueue.remove(0);
            if (stepToRemove.getRobotSide() != footstepStatusMessage.getRobotSide())
            {
               LogTools.warn("The completed step was not the next step in the queue. Killing the module.");
               resetForNewPlan();
               planningFailed.set(true);

               return false;
            }
         }
      }

      footstepStatusBuffer.clear();
      this.footstepStatusBuffer.set(footstepStatusBuffer);

      return true;
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
         stepQueue.add(new FootstepDataMessage(stepList.get(i)));

      if (stepQueue.get(0).getRobotSide() != expectedInitialSide.get().toByte())
      {
         LogTools.error("Got a plan back starting with the wrong first step. Should start with " + expectedInitialSide.get() + ", but actually starts with " +
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

      while (fixedStepQueue.size() < numberOfStepsToLeaveUnchanged.getIntegerValue())
      {
         FootstepDataMessage footstepToAdd = stepQueue.remove(0);
         if (fixedStepQueue.size() > 0)
         {
            byte lastSide = fixedStepQueue.get(fixedStepQueue.size() - 1).getRobotSide();
            if (lastSide == footstepToAdd.getRobotSide())
            {
               LogTools.error("The next quadrant when appending would be out of order. The fixed queue size " + fixedStepQueue.size() +
                                    " ends with a step on the " + RobotSide.fromByte(lastSide) + ", meaning it should start with " +
                                    RobotSide.fromByte(lastSide).getOppositeSide() + ". \nIt actually starts with " +
                                    RobotSide.fromByte(footstepToAdd.getRobotSide()) + ", but " + expectedInitialSide.get() + " was requested for starting.");
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
      if (currentPlannerOutput.get() != null)
      {
         isPlannedGoalTheFinalGoal = currentPlannerOutput.get().getLowLevelPlannerGoal().getPosition().distanceXY(planningRequestPacket.get().getGoalPositionInWorld()) < proximityGoal;
      }
      hasReachedGoal.set(!isInitialSegmentOfPlan.getBooleanValue() && isPlannedGoalTheFinalGoal && stepQueue.isEmpty());
   }
}
