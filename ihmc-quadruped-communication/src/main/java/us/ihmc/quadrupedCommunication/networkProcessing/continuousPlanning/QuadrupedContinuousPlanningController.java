package us.ihmc.quadrupedCommunication.networkProcessing.continuousPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.quadrupedCommunication.networkProcessing.OutputManager;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerType;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedContinuousPlanningController extends QuadrupedToolboxController
{
   private static final boolean debug = true;

   private static final int defaultNumberOfStepsToLeaveUnchanged = 4;
   private static final double defaultHorizonLength = 1.0;
   private static final double defaultTimeout = 1.0;

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
   private final YoBoolean receivedPlanForLastRequest = new YoBoolean("receivedPlanForLastRequest", registry);

   private final YoDouble horizonLength = new YoDouble("horizonLength", registry);
   private final YoDouble timeout = new YoDouble("timeout", registry);

   private final YoInteger broadcastPlanId = new YoInteger("broadcastPlanId", registry);
   private final YoInteger receivedPlanId = new YoInteger("receivedPlanId", registry);

   private final YoQuadrupedXGaitSettings xGaitSettings;

   public QuadrupedContinuousPlanningController(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings,
                                                OutputManager statusOutputManager, QuadrupedRobotDataReceiver robotDataReceiver,
                                                YoVariableRegistry parentRegistry)
   {
      super(robotDataReceiver, statusOutputManager, parentRegistry);

      broadcastPlanId.set(13);
      receivedPlanId.set(-1);

      xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);

      numberOfStepsToLeaveUnchanged.set(defaultNumberOfStepsToLeaveUnchanged);
      horizonLength.set(defaultHorizonLength);
      timeout.set(defaultTimeout);
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
      footstepPlannerOutput.set(message);
      receivedPlanId.set(message.getPlanId());

      if (debug)
         LogTools.info("Received result for plan #" + message.getPlanId());
   }

   public void processContinuousPlanningRequest(QuadrupedContinuousPlanningRequestPacket message)
   {
      planningRequestPacket.set(message);
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

      updateHasReachedGoal();

      checkIfLastPlanFulfilledRequest();

      if (receivedPlanForLastRequest.getBooleanValue())
      {
         updateStepQueueFromNewPlan();

         boolean fixedStepsWereUpdated = updateFixedStepQueue();

         if (fixedStepsWereUpdated)
         {
            if (stepQueue.isEmpty())
               LogTools.info("Somehow the step queue is empty.");
            else
            {
               LogTools.info("Should be broadcasting the newer plan.");
               broadcastLatestPlan();

               LogTools.info("Sending a new planning request.");
               computeAndSendLatestPlanningRequest();
            }
         }
      }
   }


   @Override
   public boolean isDone()
   {
      return hasReachedGoal.getBooleanValue();
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
      // FIXME the timings need to be shifted to properly append to the current fixed step queue.
      PawStepPlanningToolboxOutputStatus plannerOutput = footstepPlannerOutput.get();
      IDLSequence.Object<QuadrupedTimedStepMessage> stepList = plannerOutput.getFootstepDataList().getQuadrupedStepList();
      stepQueue.clear();
      stepQueue.addAll(stepList);

      QuadrupedTimedStepMessage lastFromFixedQueue = getFinalStepFromFixedQueue();
      if (lastFromFixedQueue != null)
      {
         QuadrupedTimedStepMessage firstFromNewQueue = getFirstStepFromStepQueue();
         double desiredTimeDifference = QuadrupedXGaitTools
               .computeTimeDeltaBetweenSteps(RobotQuadrant.fromByte(lastFromFixedQueue.getQuadrupedStepMessage().getRobotQuadrant()), xGaitSettings);
         double actualTimeDifference = firstFromNewQueue.getTimeInterval().getStartTime() - lastFromFixedQueue.getTimeInterval().getEndTime();
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

      QuadrupedTimedStepMessage lastStep = fixedStepQueue.get(0);
      for (int i = 1; i < fixedStepQueue.size(); i++)
      {
         if (lastStep.getTimeInterval().getEndTime() < fixedStepQueue.get(i).getTimeInterval().getEndTime())
            lastStep = fixedStepQueue.get(i);
      }

      return lastStep;
   }

   private final QuadrupedTimedStepMessage getFirstStepFromStepQueue()
   {
      QuadrupedTimedStepMessage firstStep = stepQueue.get(0);
      for (int i = 1; i < stepQueue.size(); i++)
      {
         if (firstStep.getTimeInterval().getStartTime() > stepQueue.get(i).getTimeInterval().getStartTime())
            firstStep = stepQueue.get(i);
      }

      return firstStep;
   }


   private void clearCompletedSteps()
   {
      List<QuadrupedFootstepStatusMessage> footstepStatusBuffer = this.footstepStatusBuffer.get();

      for (QuadrupedFootstepStatusMessage footstepStatusMessage : footstepStatusBuffer)
      {
         if (footstepStatusMessage.getFootstepStatus() == QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
         {
            if (debug)
               LogTools.info("Started step " + RobotQuadrant.fromByte(footstepStatusMessage.getRobotQuadrant()) + " : " + footstepStatusMessage.getActualTouchdownPositionInWorld());
            if (removeQuadrantFromFixedStepQueue(RobotQuadrant.fromByte(footstepStatusMessage.getRobotQuadrant())))
               LogTools.info("Successfully removed.");
            else
               LogTools.info("Failed to remove.");
         }
      }

      footstepStatusBuffer.clear();
      this.footstepStatusBuffer.set(footstepStatusBuffer);
   }

   private boolean updateFixedStepQueue()
   {
      if (fixedStepQueue.size() >= numberOfStepsToLeaveUnchanged.getValue() || stepQueue.size() == 0)
         return false;

      // TODO add checks on timing and quadrant
      int numberAdded = 0;
      while (fixedStepQueue.size() < numberOfStepsToLeaveUnchanged.getIntegerValue() && stepQueue.size() > 0)
      {
         numberAdded++;
         fixedStepQueue.add(stepQueue.remove(0));
      }

      if (debug)
      {
         LogTools.info("Just added " + numberAdded + " to the fixed queue, making the size " + fixedStepQueue.size());
         for (int i = 0; i < fixedStepQueue.size(); i++)
         {
            System.out.println("\tstep " + i + " = " + RobotQuadrant.fromByte(fixedStepQueue.get(i).getQuadrupedStepMessage().getRobotQuadrant()) + " : " + fixedStepQueue.get(i).getQuadrupedStepMessage().getGoalPosition() + " : Time : " + fixedStepQueue.get(i).getTimeInterval());
         }
      }

      return true;
   }

   private void updateHasReachedGoal()
   {
      hasReachedGoal.set(!isInitialSegmentOfPlan.getBooleanValue() && fixedStepQueue.isEmpty());
   }

   private void broadcastLatestPlan()
   {
      // TODO make an output status thing and report it
      QuadrupedTimedStepListMessage stepListMessage = new QuadrupedTimedStepListMessage();
      for (int i = 0; i < fixedStepQueue.size(); i++)
         stepListMessage.getQuadrupedStepList().add().set(fixedStepQueue.get(i));
      for (int i = 0; i < stepQueue.size(); i++)
         stepListMessage.getQuadrupedStepList().add().set(stepQueue.get(i));

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
                  System.out.println("\tstep " + i + " = " + RobotQuadrant.fromByte(fixedStepQueue.get(i).getQuadrupedStepMessage().getRobotQuadrant()));
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
         RobotQuadrant nextQuadrant = RobotQuadrant.fromByte(finalStep.getQuadrupedStepMessage().getRobotQuadrant()).getNextRegularGaitSwingQuadrant();
         planningRequestPacket.setInitialStepRobotQuadrant(nextQuadrant.toByte());
      }

      planningRequestPacket.setHorizonLength(horizonLength.getDoubleValue());
      planningRequestPacket.setRequestedPawPlannerType(PawStepPlannerType.VIS_GRAPH_WITH_A_STAR.toByte());
      planningRequestPacket.setStartTargetType(PawStepPlanningRequestPacket.PAW_PLANNER_TARGET_TYPE_FOOTSTEPS);
      planningRequestPacket.setTimeout(continuousPlanningRequestPacket.getTimeout());
      planningRequestPacket.setBestEffortTimeout(continuousPlanningRequestPacket.getTimeout());
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
}
