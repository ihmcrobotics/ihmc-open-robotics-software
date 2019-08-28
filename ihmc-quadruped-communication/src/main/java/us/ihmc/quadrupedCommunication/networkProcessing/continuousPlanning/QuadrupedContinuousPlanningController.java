package us.ihmc.quadrupedCommunication.networkProcessing.continuousPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
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
import us.ihmc.robotics.time.TimeIntervalProvider;
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

   private static final double proximityGoal = 1.0e-2;

   private static final int defaultNumberOfStepsToLeaveUnchanged = 4;

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<PawStepPlanningToolboxOutputStatus> footstepPlannerOutput = new AtomicReference<>();
   private final AtomicReference<QuadrupedContinuousPlanningRequestPacket> planningRequestPacket = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> latestPlanarRegions = new AtomicReference<>();
   private final AtomicReference<List<QuadrupedFootstepStatusMessage>> footstepStatusBuffer = new AtomicReference<>(new ArrayList<>());

   private final List<QuadrupedTimedStepMessage> fixedStepQueue = new ArrayList<>();
   private final List<QuadrupedTimedStepMessage> stepQueue = new ArrayList<>();

   private final YoBoolean isInitialSegmentOfPlan = new YoBoolean("isInitialSegmentOfPlan", registry);
   private final YoInteger numberOfStepsToLeaveUnchanged = new YoInteger("numberOfStepsToLeaveUnchanged", registry);
   private final YoBoolean hasReachedGoal = new YoBoolean("hasReachedGoal", registry);
   private final YoBoolean planningFailed = new YoBoolean("planningFailed", registry);
   private final YoBoolean receivedPlanForLastRequest = new YoBoolean("receivedPlanForLastRequest", registry);

   private final YoInteger broadcastPlanId = new YoInteger("broadcastPlanId", registry);
   private final YoInteger receivedPlanId = new YoInteger("receivedPlanId", registry);

   private final YoQuadrupedXGaitSettings xGaitSettings;

   private static Comparator<RobotEnd> robotEndComparator = (RobotEnd a, RobotEnd b) -> {
      if (a == b)
         return 0;
      else if (a == RobotEnd.FRONT)
         return -1;
      else
         return 1;
   };

   private static Comparator<QuadrupedTimedStepMessage> startTimeComparator = (QuadrupedTimedStepMessage a, QuadrupedTimedStepMessage b) -> {
      double startTimeA = a.getTimeInterval().getStartTime();
      double startTimeB = b.getTimeInterval().getStartTime();
      int result = Double.compare(startTimeA, startTimeB);
      if (result == 0)
      {
         RobotEnd endA = getQuadrant(a).getEnd();
         RobotEnd endB = getQuadrant(b).getEnd();

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

      broadcastPlanId.set(13);
      receivedPlanId.set(-1);

      xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);

      numberOfStepsToLeaveUnchanged.set(defaultNumberOfStepsToLeaveUnchanged);
   }

   public void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      controllerStateChangeMessage.set(message);
   }

   public void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      steppingStateChangeMessage.set(message);
   }

   public void processFootstepPlannerOutput(PawStepPlanningToolboxOutputStatus message)
   {
      PawStepPlanningResult result = PawStepPlanningResult.fromByte(message.getFootstepPlanningResult());
      if (!result.validForExecution())
      {
         LogTools.info("Planner result failed. Terminating because of " + result);
         planningFailed.getBooleanValue();
      }
      footstepPlannerOutput.set(message);
      receivedPlanId.set(message.getPlanId());

      if (debug)
         LogTools.info("Received result for plan #" + message.getPlanId());
   }

   public void processContinuousPlanningRequest(QuadrupedContinuousPlanningRequestPacket message)
   {
      planningRequestPacket.set(message);
      planningFailed.set(false);
      hasReachedGoal.set(false);
   }

   public void processFootstepStatusMessage(QuadrupedFootstepStatusMessage message)
   {
      footstepStatusBuffer.get().add(message);
   }

   public void processPlanarRegionListMessage(PlanarRegionsListMessage message)
   {
      latestPlanarRegions.set(message);
   }

   public void processQuadrupedXGaitSettings(QuadrupedXGaitSettingsPacket packet)
   {
      xGaitSettings.set(packet);
   }

   @Override
   public boolean initializeInternal()
   {
      isInitialSegmentOfPlan.set(true);

      computeAndSendLatestPlanningRequest();

      return true;
   }

   @Override
   public void updateInternal()
   {
      clearCompletedSteps();
//      updateFixedStepQueue();

      checkIfLastPlanFulfilledRequest();

      if (receivedPlanForLastRequest.getBooleanValue())
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
      PawStepPlanningToolboxOutputStatus plannerOutput = footstepPlannerOutput.get();
      if (plannerOutput == null)
      {
         receivedPlanForLastRequest.set(false);
         return;
      }

      receivedPlanForLastRequest.set(broadcastPlanId.getIntegerValue() == receivedPlanId.getIntegerValue());
   }

   private void updateStepQueueFromNewPlan()
   {
      PawStepPlanningToolboxOutputStatus plannerOutput = footstepPlannerOutput.get();
      IDLSequence.Object<QuadrupedTimedStepMessage> stepList = plannerOutput.getFootstepDataList().getQuadrupedStepList();
      stepQueue.clear();
      stepQueue.addAll(stepList);
      TimeIntervalTools.sort(stepQueue, startTimeComparator);


      QuadrupedTimedStepMessage lastFromFixedQueue = getFinalStepFromFixedQueue();
      if (lastFromFixedQueue != null)
      {
         QuadrupedTimedStepMessage firstFromNewQueue = stepQueue.get(0);
         double desiredTimeDifference = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(getQuadrant(lastFromFixedQueue), xGaitSettings);
         double actualTimeDifference = firstFromNewQueue.getTimeInterval().getStartTime() - lastFromFixedQueue.getTimeInterval().getStartTime();
         double timeShift = desiredTimeDifference - actualTimeDifference;
         for (int i = 0; i < stepQueue.size(); i++)
         {
            QuadrupedTimedStepMessage step = stepQueue.get(i);
            double startTime = step.getTimeInterval().getStartTime() + timeShift;
            double endTime = step.getTimeInterval().getEndTime() + timeShift;
            step.getTimeInterval().setStartTime(startTime);
            step.getTimeInterval().setEndTime(endTime);
         }
      }


      isInitialSegmentOfPlan.set(false);
   }

   private final QuadrupedTimedStepMessage getFinalStepFromFixedQueue()
   {
      if (fixedStepQueue.size() == 0)
         return null;

      return fixedStepQueue.get(fixedStepQueue.size() -1 );
   }

   private void clearCompletedSteps()
   {
      List<QuadrupedFootstepStatusMessage> footstepStatusBuffer = this.footstepStatusBuffer.get();

      for (QuadrupedFootstepStatusMessage footstepStatusMessage : footstepStatusBuffer)
      {
         if (footstepStatusMessage.getFootstepStatus() == QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
         {
            boolean success = removeQuadrantFromFixedStepQueue(RobotQuadrant.fromByte(footstepStatusMessage.getRobotQuadrant()));

            if (debug)
            {
               LogTools.info("Started step " + RobotQuadrant.fromByte(footstepStatusMessage.getRobotQuadrant()) + " : " + footstepStatusMessage.getActualTouchdownPositionInWorld());
               if (success)
                  LogTools.info("Successfully removed.");
               else
                  LogTools.info("Failed to remove.");
            }
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
      TimeIntervalTools.sort(stepQueue, startTimeComparator);

      // TODO add checks on timing and quadrant
      int numberAdded = 0;
      while (fixedStepQueue.size() < numberOfStepsToLeaveUnchanged.getIntegerValue() && stepQueue.size() > 0)
      {
         numberAdded++;

         QuadrupedTimedStepMessage stepToAdd = stepQueue.remove(0);
         if (fixedStepQueue.size() > 0)
         {
            RobotQuadrant lastQuadrant = getQuadrant(fixedStepQueue.get(fixedStepQueue.size() - 1));
            RobotQuadrant nextQuadrant = getQuadrant(stepToAdd);

            if (lastQuadrant.getNextRegularGaitSwingQuadrant() != nextQuadrant)
            {
               LogTools.error("Had a bug and the exception gets swallowed.");
               throw new RuntimeException("This is out of order.");
            }
         }
         fixedStepQueue.add(stepToAdd);
      }

      checkStepOrders(fixedStepQueue);
      checkStepOrders(stepQueue);

      QuadrupedTimedStepMessage firstStep = fixedStepQueue.get(0);
      double timeShift = -firstStep.getTimeInterval().getStartTime();
      shiftStepsInTime(timeShift, fixedStepQueue);
      shiftStepsInTime(timeShift, stepQueue);

      checkStepTimes(fixedStepQueue, xGaitSettings);
      checkStepTimes(stepQueue, xGaitSettings);


      if (debug)
      {
         LogTools.info("Just added " + numberAdded + " to the fixed queue, making the size " + fixedStepQueue.size());
         for (int i = 0; i < fixedStepQueue.size(); i++)
         {
            System.out.println("\tstep " + i + " = " + getQuadrant(fixedStepQueue.get(i)) + " : " + fixedStepQueue.get(i).getQuadrupedStepMessage().getGoalPosition() + " : Time : " + fixedStepQueue.get(i).getTimeInterval());
         }
      }

      return true;
   }

   private static void checkStepOrders(List<QuadrupedTimedStepMessage> stepList)
   {
      for (int i = 0; i < stepList.size() - 1; i++)
      {
         RobotQuadrant lastQuadrant = getQuadrant(stepList.get(i));
         RobotQuadrant nextQuadrant = getQuadrant(stepList.get(i + 1));

         if (lastQuadrant.getNextRegularGaitSwingQuadrant() != nextQuadrant)
         {
            LogTools.error("Had a bug and the exception gets swallowed.");
            throw new RuntimeException("This is out of order.");
         }
      }
   }

   private static void checkStepTimes(List<QuadrupedTimedStepMessage> stepList, QuadrupedXGaitSettingsReadOnly xGaitSettings)
   {
      if (stepList.size() < 1)
         return;

      RobotQuadrant previousMovingQuadrant = getQuadrant(stepList.get(0));
      double previousStartTime = stepList.get(0).getTimeInterval().getStartTime();
      for (int i = 1; i < stepList.size(); i++)
      {
         RobotQuadrant movingQuadrant = getQuadrant(stepList.get(i));
         double startTime = stepList.get(i).getTimeInterval().getStartTime();
         double expectedTimeBetweenSteps = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(previousMovingQuadrant, xGaitSettings);
         if (!MathTools.epsilonEquals(startTime - previousStartTime, expectedTimeBetweenSteps, 1e-3))
         {
            LogTools.error("Had a bug and the exception gets swallowed.");
            throw new RuntimeException("This is poorly timed..");
         }
         previousMovingQuadrant = movingQuadrant;
         previousStartTime = startTime;
      }
   }

   private static void shiftStepsInTime(double timeShift, List<QuadrupedTimedStepMessage> stepList)
   {
      for (QuadrupedTimedStepMessage step : stepList)
      {
         TimeIntervalMessage timeInterval = step.getTimeInterval();
         timeInterval.setStartTime(timeInterval.getStartTime() + timeShift);
         timeInterval.setEndTime(timeInterval.getEndTime() + timeShift);
      }
   }

   private void updateHasReachedGoal()
   {
      boolean isGoalTheFinalGoal = false;
      if (footstepPlannerOutput.get() != null)
      {
         isGoalTheFinalGoal = footstepPlannerOutput.get().getLowLevelPlannerGoal().getPosition().epsilonEquals(planningRequestPacket.get().getGoalPositionInWorld(), proximityGoal);
      }
      hasReachedGoal.set(!isInitialSegmentOfPlan.getBooleanValue() && isGoalTheFinalGoal && stepQueue.isEmpty());
   }

   private void broadcastLatestPlan()
   {
      // TODO make an output status thing and report it
      QuadrupedTimedStepListMessage stepListMessage = new QuadrupedTimedStepListMessage();
      for (int i = 0; i < fixedStepQueue.size(); i++)
         stepListMessage.getQuadrupedStepList().add().set(fixedStepQueue.get(i));
      for (int i = 0; i < stepQueue.size(); i++)
         stepListMessage.getQuadrupedStepList().add().set(stepQueue.get(i));

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
      }
      else
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            QuadrupedTimedStepMessage timedStepMessage = getFirstStepInQuadrantFromList(robotQuadrant, fixedStepQueue);
            if (timedStepMessage != null)
            {
               startPositions.put(robotQuadrant, timedStepMessage.getQuadrupedStepMessage().getGoalPosition());
            }
            else
            {
               LogTools.error("Had a bug and the exception gets swallowed. Fixed step queue = " );
               for (int i = 0; i < fixedStepQueue.size(); i++)
               {
                  System.out.println("\tstep " + i + " = " + getQuadrant(fixedStepQueue.get(i)));
               }
               throw new RuntimeException("bug ");
            }
         }
      }

      planningRequestPacket.getFrontLeftPositionInWorld().set(startPositions.get(RobotQuadrant.FRONT_LEFT));
      planningRequestPacket.getFrontRightPositionInWorld().set(startPositions.get(RobotQuadrant.FRONT_RIGHT));
      planningRequestPacket.getHindLeftPositionInWorld().set(startPositions.get(RobotQuadrant.HIND_LEFT));
      planningRequestPacket.getHindRightPositionInWorld().set(startPositions.get(RobotQuadrant.HIND_RIGHT));


      QuadrupedTimedStepMessage finalStep = getFinalStepFromFixedQueue();
      if (finalStep != null)
      {
         if (MathTools.epsilonEquals(xGaitSettings.getEndPhaseShift(), QuadrupedGait.AMBLE.getEndPhaseShift(), 1e-5))
         {
            RobotQuadrant nextQuadrant = getQuadrant(finalStep).getNextRegularGaitSwingQuadrant();
            planningRequestPacket.setInitialStepRobotQuadrant(nextQuadrant.toByte());
         }
         else if (MathTools.epsilonEquals(xGaitSettings.getEndPhaseShift(), QuadrupedGait.TROT.getEndPhaseShift(), 1e-5))
         {
            RobotQuadrant quadrant = getQuadrant(finalStep);
            if (quadrant.getEnd() == RobotEnd.FRONT)
               planningRequestPacket.setInitialStepRobotQuadrant(quadrant.getAcrossBodyQuadrant().toByte());
            else
               planningRequestPacket.setInitialStepRobotQuadrant(quadrant.getDiagonalOppositeQuadrant().getAcrossBodyQuadrant().toByte());
         }
         else
         {
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

      broadcastPlanId.increment();
      planningRequestPacket.setPlannerRequestId(broadcastPlanId.getIntegerValue());

      if (debug)
      {
         LogTools.info("Planning #" + broadcastPlanId.getValue() + " to " + continuousPlanningRequestPacket.getGoalPositionInWorld());
         LogTools.info("Starting from:");
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            System.out.println("\t" + robotQuadrant + " : " + startPositions.get(robotQuadrant));
         }
      }
      reportMessage(planningRequestPacket);
   }

   private boolean removeQuadrantFromFixedStepQueue(RobotQuadrant quadrant)
   {
      QuadrupedTimedStepMessage messageToRemove = getFirstStepInQuadrantFromList(quadrant, fixedStepQueue);
      if (messageToRemove != null)
      {
         return fixedStepQueue.remove(messageToRemove);
      }
      {
         return false;
      }
   }

   private static QuadrupedTimedStepMessage getFirstStepInQuadrantFromList(RobotQuadrant robotQuadrant, List<QuadrupedTimedStepMessage> stepList)
   {
      for (int i = 0; i < stepList.size(); i++)
      {
         if (stepList.get(i).getQuadrupedStepMessage().getRobotQuadrant() == robotQuadrant.toByte())
         {
            return stepList.get(i);
         }
      }

      return null;
   }

   private static RobotQuadrant getQuadrant(QuadrupedTimedStepMessage message)
   {
      return RobotQuadrant.fromByte(message.getQuadrupedStepMessage().getRobotQuadrant());
   }
}
