package us.ihmc.quadrupedCommunication.networkProcessing.continuousPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedCommunication.networkProcessing.OutputManager;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerType;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlanningResult;
import us.ihmc.quadrupedPlanning.QuadrupedGait;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedContinuousPlanningController extends QuadrupedToolboxController
{
   private static final boolean debug = false;
   private static final boolean verbose = true;

   private static final double proximityGoal = 1.0e-2;
   private static final int defaultStartPlanId = 13;

   private static final int defaultNumberOfStepsToLeaveUnchanged = 6;

   private final AtomicReference<PawStepPlanningToolboxOutputStatus> latestPlannerOutput = new AtomicReference<>();
   private final AtomicReference<PawStepPlanningToolboxOutputStatus> currentPlannerOutput = new AtomicReference<>();
   private final AtomicReference<QuadrupedContinuousPlanningRequestPacket> planningRequestPacket = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> latestPlanarRegions = new AtomicReference<>();
   private final AtomicReference<List<QuadrupedFootstepStatusMessage>> footstepStatusBuffer = new AtomicReference<>(new ArrayList<>());
   private final AtomicReference<RobotQuadrant> expectedInitialQuadrant = new AtomicReference<>();

   private final QuadrantDependentList<Point3DReadOnly> currentFeetPositions = new QuadrantDependentList<>();
   private final List<QuadrupedTimedStep> fixedStepQueue = new ArrayList<>();
   private final List<QuadrupedTimedStep> stepQueue = new ArrayList<>();

   private final YoBoolean isInitialSegmentOfPlan = new YoBoolean("isInitialSegmentOfPlan", registry);
   private final YoInteger numberOfStepsToLeaveUnchanged = new YoInteger("numberOfStepsToLeaveUnchanged", registry);
   private final YoBoolean hasReachedGoal = new YoBoolean("hasReachedGoal", registry);
   private final YoBoolean planningFailed = new YoBoolean("planningFailed", registry);
   private final YoBoolean receivedPlanForLastRequest = new YoBoolean("receivedPlanForLastRequest", registry);
   private final YoBoolean waitingForPlan = new YoBoolean("waitingForPlan", registry);

   private final YoInteger broadcastPlanId = new YoInteger("broadcastPlanId", registry);
   private final YoInteger receivedPlanId = new YoInteger("receivedPlanId", registry);

   private final YoQuadrupedXGaitSettings xGaitSettings;

   static Comparator<RobotEnd> robotEndComparator = (RobotEnd a, RobotEnd b) -> {
      if (a == b)
         return 0;
      else if (a == RobotEnd.FRONT)
         return -1;
      else
         return 1;
   };

   static Comparator<QuadrupedTimedStep> startTimeComparator = (QuadrupedTimedStep a, QuadrupedTimedStep b) -> {
      double startTimeA = a.getTimeInterval().getStartTime();
      double startTimeB = b.getTimeInterval().getStartTime();
      int result = Double.compare(startTimeA, startTimeB);
      if (result == 0)
      {
         RobotEnd endA = a.getRobotQuadrant().getEnd();
         RobotEnd endB = b.getRobotQuadrant().getEnd();

         return robotEndComparator.compare(endA, endB);
      }
      else
      {
         return result;
      }
   };


   public QuadrupedContinuousPlanningController(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                                OutputManager statusOutputManager, QuadrupedRobotDataReceiver robotDataReceiver,
                                                YoVariableRegistry parentRegistry)
   {
      super(robotDataReceiver, statusOutputManager, parentRegistry);

      broadcastPlanId.set(defaultStartPlanId);
      receivedPlanId.set(-1);

      xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);

      numberOfStepsToLeaveUnchanged.set(defaultNumberOfStepsToLeaveUnchanged);
   }

   public void processFootstepPlannerOutput(PawStepPlanningToolboxOutputStatus message)
   {
      PawStepPlanningResult result = PawStepPlanningResult.fromByte(message.getFootstepPlanningResult());
      if (!result.validForExecution())
      {
         LogTools.info("Planner result failed. Terminating because of " + result);
         resetForNewPlan();
         planningFailed.set(true);
      }
      latestPlannerOutput.set(message);
      receivedPlanId.set(message.getPlanId());
      waitingForPlan.set(false);

      if (debug)
         LogTools.info("Received result for plan #" + message.getPlanId());
   }

   public void processContinuousPlanningRequest(QuadrupedContinuousPlanningRequestPacket message)
   {
      resetForNewPlan();

      if (debug || verbose)
      {
         LogTools.info("Received new plan request. Going to " + message.getGoalPositionInWorld());
      }

      planningRequestPacket.set(message);
   }

   public void processFootstepStatusMessage(QuadrupedFootstepStatusMessage message)
   {
      List<QuadrupedFootstepStatusMessage> messages = footstepStatusBuffer.get();
      messages.add(message);
      footstepStatusBuffer.set(messages);
      currentFeetPositions.put(RobotQuadrant.fromByte(message.getRobotQuadrant()), message.getActualTouchdownPositionInWorld());
   }

   public void processPlanarRegionListMessage(PlanarRegionsListMessage message)
   {
      latestPlanarRegions.set(message);
   }

   public void processQuadrupedXGaitSettings(QuadrupedXGaitSettingsPacket packet)
   {
      xGaitSettings.set(packet);
   }

   private void resetForNewPlan()
   {
      waitingForPlan.set(true);
      isInitialSegmentOfPlan.set(true);

      planningFailed.set(false);
      hasReachedGoal.set(false);
      fixedStepQueue.clear();
      currentFeetPositions.clear();
      stepQueue.clear();
      footstepStatusBuffer.set(new ArrayList<>());
      latestPlannerOutput.set(null);
      expectedInitialQuadrant.set(null);

      broadcastPlanId.set(defaultStartPlanId);
      receivedPlanId.set(-1);
   }

   @Override
   public boolean initializeInternal()
   {
      resetForNewPlan();

      computeAndSendLatestPlanningRequest();

      return true;
   }

   @Override
   public void updateInternal()
   {
      clearCompletedSteps();
//      updateFixedStepQueue();

      checkIfLastPlanFulfilledRequest();

      if (!waitingForPlan.getBooleanValue())// && receivedPlanForLastRequest.getBooleanValue())
      {
         updateStepQueueFromNewPlan();

         boolean fixedStepsWereUpdated = updateFixedStepQueue();

         if (fixedStepsWereUpdated)
         {
            updateHasReachedGoal();

            if (!hasReachedGoal.getBooleanValue())
            {
               broadcastLatestPlan();

               if (debug)
                  LogTools.info("Sending a new planning request.");
               computeAndSendLatestPlanningRequest();
            }
         }
      }
   }


   @Override
   public boolean isDone()
   {
      return hasReachedGoal.getBooleanValue() || planningFailed.getBooleanValue();
   }


   private void checkIfLastPlanFulfilledRequest()
   {
      PawStepPlanningToolboxOutputStatus plannerOutput = currentPlannerOutput.get();
      if (plannerOutput == null)
      {
         receivedPlanForLastRequest.set(false);
         return;
      }

      receivedPlanForLastRequest.set(broadcastPlanId.getIntegerValue() == receivedPlanId.getIntegerValue());
   }

   private void updateStepQueueFromNewPlan()
   {
      PawStepPlanningToolboxOutputStatus plannerOutput = latestPlannerOutput.getAndSet(null);
      if (plannerOutput == null)
         return;

      currentPlannerOutput.set(plannerOutput);
      IDLSequence.Object<QuadrupedTimedStepMessage> stepList = plannerOutput.getFootstepDataList().getQuadrupedStepList();
      stepQueue.clear();
      for (int i = 0; i < stepList.size(); i++)
         stepQueue.add(new QuadrupedTimedStep(stepList.get(i)));

      shiftStepTimesToAppendToList(fixedStepQueue, stepQueue, xGaitSettings);

      isInitialSegmentOfPlan.set(false);
   }

   private static void shiftStepTimesToAppendToList(List<QuadrupedTimedStep> rootSteps, List<QuadrupedTimedStep> stepsToAppend,
                                                    QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      TimeIntervalTools.sort(stepsToAppend, startTimeComparator);

      QuadrupedTimedStep lastFromFixedQueue = getLastItemInList(rootSteps);
      if (lastFromFixedQueue != null)
      {
         QuadrupedTimedStep firstFromNewQueue = stepsToAppend.get(0);
         double desiredTimeDifference = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(lastFromFixedQueue.getRobotQuadrant(), xGaitSettings);
         double actualTimeDifference = firstFromNewQueue.getTimeInterval().getStartTime() - lastFromFixedQueue.getTimeInterval().getStartTime();
         double timeShift = desiredTimeDifference - actualTimeDifference;
         shiftStepsInTime(timeShift, stepsToAppend);
      }
   }

   private static <T> T getLastItemInList(List<T> list)
   {
      if (list.size() == 0)
         return null;

      return list.get(list.size() - 1);
   }

   private void clearCompletedSteps()
   {
      List<QuadrupedFootstepStatusMessage> footstepStatusBuffer = this.footstepStatusBuffer.get();

      for (QuadrupedFootstepStatusMessage footstepStatusMessage : footstepStatusBuffer)
      {
         if (footstepStatusMessage.getFootstepStatus() == QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
         {
            removeQuadrantFromFixedStepQueue(RobotQuadrant.fromByte(footstepStatusMessage.getRobotQuadrant()));
         }
      }

      footstepStatusBuffer.clear();
      this.footstepStatusBuffer.set(footstepStatusBuffer);
   }

   private boolean updateFixedStepQueue()
   {
      if (fixedStepQueue.size() >= numberOfStepsToLeaveUnchanged.getValue() || stepQueue.size() == 0)
         return false;

      TimeIntervalTools.sort(fixedStepQueue, startTimeComparator);
      shiftStepTimesToAppendToList(fixedStepQueue, stepQueue, xGaitSettings);

      while (fixedStepQueue.size() < numberOfStepsToLeaveUnchanged.getIntegerValue() && stepQueue.size() > 0)
      {
         QuadrupedTimedStep stepToAdd = stepQueue.remove(0);
         if (fixedStepQueue.size() > 0)
         {
            RobotQuadrant lastQuadrant = getLastItemInList(fixedStepQueue).getRobotQuadrant();
            RobotQuadrant nextQuadrant = stepToAdd.getRobotQuadrant();

            if (lastQuadrant.getNextRegularGaitSwingQuadrant() != nextQuadrant)
            {
               LogTools.error("The next quadrant when appending would be out of order. The fixed queue size " + fixedStepQueue.size() +
                                    " ends with a step on the " + lastQuadrant + " quadrant, meaning it should start with " +
                                    lastQuadrant.getNextRegularGaitSwingQuadrant() + ". It actually starts with " + nextQuadrant + ", but " +
                                    expectedInitialQuadrant.get() + " was requested for starting.");
               resetForNewPlan();
               planningFailed.set(true);
               throw new RuntimeException("This is out of order.");
            }
         }
         fixedStepQueue.add(stepToAdd);
      }

      checkStepOrders(fixedStepQueue);
      checkStepOrders(stepQueue);

      QuadrupedTimedStep firstStep = fixedStepQueue.get(0);
      double timeShift = -firstStep.getTimeInterval().getStartTime();
      shiftStepsInTime(timeShift, fixedStepQueue);
      shiftStepsInTime(timeShift, stepQueue);

      if (firstStep.getTimeInterval().getStartTime() != 0.0)
      {
         LogTools.error("The first step is not at time 0.0.");
         resetForNewPlan();
         planningFailed.set(true);
         throw new RuntimeException("This steps aren't at the correct times.");
      }

      checkStepTimes("Fixed queue", fixedStepQueue, xGaitSettings);
      checkStepTimes("Step queue", stepQueue, xGaitSettings);

      return true;
   }

   private void checkStepOrders(List<QuadrupedTimedStep> stepList)
   {
      for (int i = 0; i < stepList.size() - 1; i++)
      {
         RobotQuadrant lastQuadrant = stepList.get(i).getRobotQuadrant();
         RobotQuadrant nextQuadrant = stepList.get(i + 1).getRobotQuadrant();

         if (lastQuadrant.getNextRegularGaitSwingQuadrant() != nextQuadrant)
         {
            LogTools.error("The steps are out of order, and the bug may be getting swallowed.");
            resetForNewPlan();
            planningFailed.set(true);
            throw new RuntimeException("This is out of order.");
         }
      }
   }

   private void checkStepTimes(String prefix, List<QuadrupedTimedStep> stepList, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      if (stepList.size() < 1)
         return;

      RobotQuadrant previousMovingQuadrant = stepList.get(0).getRobotQuadrant();
      double previousStartTime = stepList.get(0).getTimeInterval().getStartTime();
      for (int i = 1; i < stepList.size(); i++)
      {
         RobotQuadrant movingQuadrant = stepList.get(i).getRobotQuadrant();
         double startTime = stepList.get(i).getTimeInterval().getStartTime();
         double expectedTimeBetweenSteps = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(previousMovingQuadrant, xGaitSettings);
         if (!MathTools.epsilonEquals(startTime - previousStartTime, expectedTimeBetweenSteps, 1e-3))
         {
            LogTools.error("The " + prefix + " steps are improperly timed.");
            resetForNewPlan();
            planningFailed.set(true);
            throw new RuntimeException("This is poorly timed.");
         }
         previousMovingQuadrant = movingQuadrant;
         previousStartTime = startTime;
      }
   }

   private static void shiftStepsInTime(double timeShift, List<QuadrupedTimedStep> stepList)
   {
      for (QuadrupedTimedStep step : stepList)
         step.getTimeInterval().shiftInterval(timeShift);
   }

   private void updateHasReachedGoal()
   {
      boolean isGoalTheFinalGoal = false;
      if (currentPlannerOutput.get() != null)
      {
         isGoalTheFinalGoal = currentPlannerOutput.get().getLowLevelPlannerGoal().getPosition().distanceXY(planningRequestPacket.get().getGoalPositionInWorld()) < proximityGoal;
      }
      hasReachedGoal.set(!isInitialSegmentOfPlan.getBooleanValue() && isGoalTheFinalGoal && stepQueue.isEmpty());
   }

   private void broadcastLatestPlan()
   {
      // TODO make an output status thing and report it
      QuadrupedTimedStepListMessage stepListMessage = new QuadrupedTimedStepListMessage();
      for (int i = 0; i < fixedStepQueue.size(); i++)
         stepListMessage.getQuadrupedStepList().add().set(QuadrupedMessageTools.createQuadrupedTimedStepMessage(fixedStepQueue.get(i)));
      for (int i = 0; i < stepQueue.size(); i++)
         stepListMessage.getQuadrupedStepList().add().set(QuadrupedMessageTools.createQuadrupedTimedStepMessage(stepQueue.get(i)));

      stepListMessage.setIsExpressedInAbsoluteTime(false);

      reportMessage(stepListMessage);
   }

   private void computeAndSendLatestPlanningRequest()
   {
      PawStepPlanningRequestPacket planningRequestPacket = new PawStepPlanningRequestPacket();
      QuadrupedContinuousPlanningRequestPacket continuousPlanningRequestPacket = this.planningRequestPacket.get();

      ToolboxStateMessage plannerState = new ToolboxStateMessage();
      plannerState.setRequestedToolboxState(ToolboxStateMessage.WAKE_UP);
      reportMessage(plannerState);

      QuadrantDependentList<Point3DReadOnly> startPositions = new QuadrantDependentList<>();
      if (isInitialSegmentOfPlan.getBooleanValue())
      {
         if (robotDataReceiver != null)
         {
            for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
            {
               FramePoint3D position = new FramePoint3D(robotDataReceiver.getReferenceFrames().getSoleFrame(robotQuadrant));
               position.changeFrame(ReferenceFrame.getWorldFrame());

               startPositions.put(robotQuadrant, position);
            }
         }
         else
         {
            startPositions.put(RobotQuadrant.FRONT_LEFT, continuousPlanningRequestPacket.getFrontLeftStartPositionInWorld());
            startPositions.put(RobotQuadrant.FRONT_RIGHT, continuousPlanningRequestPacket.getFrontLeftStartPositionInWorld());
            startPositions.put(RobotQuadrant.HIND_LEFT, continuousPlanningRequestPacket.getHindLeftStartPositionInWorld());
            startPositions.put(RobotQuadrant.HIND_RIGHT, continuousPlanningRequestPacket.getHindRightStartPositionInWorld());
         }

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
            currentFeetPositions.put(robotQuadrant, startPositions.get(robotQuadrant));

         if (debug || verbose)
         {
            Point3D averageStart = new Point3D();
            for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
            {
               averageStart.add(startPositions.get(robotQuadrant));
            }
            averageStart.scale(0.25);

            LogTools.info("Starting from " + averageStart);
         }
      }
      else
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            QuadrupedTimedStep step = getLastStepInQuadrantFromList(robotQuadrant, fixedStepQueue);
            if (step != null)
               startPositions.put(robotQuadrant, step.getGoalPosition());
            else
               startPositions.put(robotQuadrant, currentFeetPositions.get(robotQuadrant));
         }
      }

      planningRequestPacket.getFrontLeftPositionInWorld().set(startPositions.get(RobotQuadrant.FRONT_LEFT));
      planningRequestPacket.getFrontRightPositionInWorld().set(startPositions.get(RobotQuadrant.FRONT_RIGHT));
      planningRequestPacket.getHindLeftPositionInWorld().set(startPositions.get(RobotQuadrant.HIND_LEFT));
      planningRequestPacket.getHindRightPositionInWorld().set(startPositions.get(RobotQuadrant.HIND_RIGHT));


      QuadrupedTimedStep finalStep = getLastItemInList(fixedStepQueue);
      if (finalStep != null)
      {
         if (MathTools.epsilonEquals(xGaitSettings.getEndPhaseShift(), QuadrupedGait.AMBLE.getEndPhaseShift(), 1e-5))
         {
            RobotQuadrant nextQuadrant = finalStep.getRobotQuadrant().getNextRegularGaitSwingQuadrant();
            planningRequestPacket.setInitialStepRobotQuadrant(nextQuadrant.toByte());
         }
         else if (MathTools.epsilonEquals(xGaitSettings.getEndPhaseShift(), QuadrupedGait.TROT.getEndPhaseShift(), 1e-5))
         {
            RobotQuadrant quadrant = finalStep.getRobotQuadrant();
            if (quadrant.getEnd() == RobotEnd.FRONT)
               planningRequestPacket.setInitialStepRobotQuadrant(quadrant.getAcrossBodyQuadrant().toByte());
            else
               planningRequestPacket.setInitialStepRobotQuadrant(quadrant.getDiagonalOppositeQuadrant().getAcrossBodyQuadrant().toByte());
         }
         else
         {
            resetForNewPlan();
            planningFailed.set(true);
            throw new RuntimeException("This is not supported.");
         }
      }

      planningRequestPacket.setHorizonLength(continuousPlanningRequestPacket.getHorizonLength());
      planningRequestPacket.setRequestedPawPlannerType(PawStepPlannerType.VIS_GRAPH_WITH_A_STAR.toByte());
      planningRequestPacket.setStartTargetType(PawStepPlanningRequestPacket.PAW_PLANNER_TARGET_TYPE_FOOTSTEPS);
      planningRequestPacket.setTimeout(continuousPlanningRequestPacket.getTimeout());
      planningRequestPacket.setBestEffortTimeout(continuousPlanningRequestPacket.getBestEffortTimeout());
      planningRequestPacket.getGoalPositionInWorld().set(continuousPlanningRequestPacket.getGoalPositionInWorld());
      planningRequestPacket.getGoalOrientationInWorld().set(continuousPlanningRequestPacket.getGoalOrientationInWorld());
      planningRequestPacket.getPlanarRegionsListMessage().set(latestPlanarRegions.get());

      expectedInitialQuadrant.set(RobotQuadrant.fromByte(planningRequestPacket.getInitialStepRobotQuadrant()));

      broadcastPlanId.increment();
      planningRequestPacket.setPlannerRequestId(broadcastPlanId.getIntegerValue());

      if (debug || verbose)
      {
         LogTools.info("Planning #" + broadcastPlanId.getValue() + " to " + continuousPlanningRequestPacket.getGoalPositionInWorld() + ", starting with " + expectedInitialQuadrant.get());
      }
      waitingForPlan.set(true);
      reportMessage(planningRequestPacket);
   }

   private boolean removeQuadrantFromFixedStepQueue(RobotQuadrant quadrant)
   {
      QuadrupedTimedStep messageToRemove = getFirstStepInQuadrantFromList(quadrant, fixedStepQueue);
      if (messageToRemove != null)
      {
         return fixedStepQueue.remove(messageToRemove);
      }
      {
         return false;
      }
   }

   private static QuadrupedTimedStep getFirstStepInQuadrantFromList(RobotQuadrant robotQuadrant, List<QuadrupedTimedStep> stepList)
   {
      for (int i = 0; i < stepList.size(); i++)
      {
         if (stepList.get(i).getRobotQuadrant() == robotQuadrant)
         {
            return stepList.get(i);
         }
      }

      return null;
   }

   static QuadrupedTimedStep getLastStepInQuadrantFromList(RobotQuadrant robotQuadrant, List<QuadrupedTimedStep> stepList)
   {
      for (int i = stepList.size() - 1; i >= 0; i--)
      {
         if (stepList.get(i).getRobotQuadrant() == robotQuadrant)
         {
            return stepList.get(i);
         }
      }

      return null;
   }
}
