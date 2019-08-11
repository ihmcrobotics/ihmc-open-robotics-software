package us.ihmc.quadrupedCommunication.networkProcessing.continuousPlanning;

import boofcv.struct.image.Planar;
import controller_msgs.msg.dds.*;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.idl.IDLSequence;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedCommunication.networkProcessing.OutputManager;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerType;
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
   private static final int defaultNumberOfStepsToLeaveUnchanged = 4;
   private static final double defaultHorizonLength = 1.0;
   private static final double defaultTimeout = 1.0;

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedFootstepPlanningToolboxOutputStatus> footstepPlannerOutput = new AtomicReference<>();
   private final AtomicReference<QuadrupedContinuousPlanningRequestPacket> planningRequestPacket = new AtomicReference<>();
   private final AtomicReference<QuadrupedFootstepStatusMessage> footstepStatusMessage = new AtomicReference<>();
   private final AtomicReference<PlanarRegionsListMessage> latestPlanarRegions = new AtomicReference<>();

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

   public QuadrupedContinuousPlanningController(OutputManager statusOutputManager, QuadrupedRobotDataReceiver robotDataReceiver,
                                              YoVariableRegistry parentRegistry)
   {
      super(robotDataReceiver, statusOutputManager, parentRegistry);

      broadcastPlanId.set(13);
      receivedPlanId.set(-1);

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

   public void processFootstepPlannerOutput(QuadrupedFootstepPlanningToolboxOutputStatus message)
   {
      footstepPlannerOutput.set(message);
   }

   public void processContinuousPlanningRequest(QuadrupedContinuousPlanningRequestPacket message)
   {
      planningRequestPacket.set(message);
      hasReachedGoal.set(false);
   }

   public void processFootstepStatusMessage(QuadrupedFootstepStatusMessage message)
   {
      footstepStatusMessage.set(message);

      if (message.getFootstepStatus() == QuadrupedFootstepStatusMessage.FOOTSTEP_STATUS_STARTED)
         removeQuadrantFromFixedStepQueue(RobotQuadrant.fromByte(message.getRobotQuadrant()));
   }

   public void processPlanarRegionListMessage(PlanarRegionsListMessage message)
   {
      latestPlanarRegions.set(message);
   }

   @Override
   public boolean initializeInternal()
   {
      isInitialSegmentOfPlan.set(true);

      return true;
   }

   @Override
   public void updateInternal()
   {
      updateFixedStepQueue();

      updateHasReachedGoal();

      checkIfLastPlanFulfilledRequest();

      if (receivedPlanForLastRequest.getBooleanValue())
      {
         updateStepQueueFromNewPlan();

         if (!stepQueue.isEmpty())
         {
            broadcastLatestPlan();

            computeAndSendLatestPlanningRequest();
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
      QuadrupedFootstepPlanningToolboxOutputStatus plannerOutput = footstepPlannerOutput.get();
      if (plannerOutput == null)
      {
         receivedPlanForLastRequest.set(false);
         return;
      }

      receivedPlanId.set(plannerOutput.getPlanId());

      receivedPlanForLastRequest.set(broadcastPlanId.getIntegerValue() == receivedPlanId.getIntegerValue());
   }

   private void updateStepQueueFromNewPlan()
   {
      QuadrupedFootstepPlanningToolboxOutputStatus plannerOutput = footstepPlannerOutput.get();
      IDLSequence.Object<QuadrupedTimedStepMessage> stepList = plannerOutput.getFootstepDataList().getQuadrupedStepList();
      stepQueue.clear();
      stepQueue.addAll(stepList);

      isInitialSegmentOfPlan.set(false);
   }

   private void updateFixedStepQueue()
   {
      // TODO add checks on timing and quadrant
      while (fixedStepQueue.size() < numberOfStepsToLeaveUnchanged.getIntegerValue() && stepQueue.size() > 0)
      {
         fixedStepQueue.add(stepQueue.remove(0));
      }

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
      QuadrupedFootstepPlanningRequestPacket planningRequestPacket = new QuadrupedFootstepPlanningRequestPacket();

      QuadrantDependentList<Point3DReadOnly> startPositions = new QuadrantDependentList<>();
      if (isInitialSegmentOfPlan.getBooleanValue())
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
         startPositions.put(RobotQuadrant.FRONT_LEFT, getFirstStepInQuadrantFromList(RobotQuadrant.FRONT_LEFT, fixedStepQueue).getQuadrupedStepMessage().getGoalPosition());
         startPositions.put(RobotQuadrant.FRONT_RIGHT, getFirstStepInQuadrantFromList(RobotQuadrant.FRONT_RIGHT, fixedStepQueue).getQuadrupedStepMessage().getGoalPosition());
         startPositions.put(RobotQuadrant.HIND_LEFT, getFirstStepInQuadrantFromList(RobotQuadrant.HIND_LEFT, fixedStepQueue).getQuadrupedStepMessage().getGoalPosition());
         startPositions.put(RobotQuadrant.HIND_RIGHT, getFirstStepInQuadrantFromList(RobotQuadrant.HIND_RIGHT, fixedStepQueue).getQuadrupedStepMessage().getGoalPosition());
      }

      planningRequestPacket.getFrontLeftPositionInWorld().set(startPositions.get(RobotQuadrant.FRONT_LEFT));
      planningRequestPacket.getFrontRightPositionInWorld().set(startPositions.get(RobotQuadrant.FRONT_RIGHT));
      planningRequestPacket.getHindLeftPositionInWorld().set(startPositions.get(RobotQuadrant.HIND_LEFT));
      planningRequestPacket.getHindRightPositionInWorld().set(startPositions.get(RobotQuadrant.HIND_RIGHT));

      QuadrupedContinuousPlanningRequestPacket continuousPlanningRequestPacket = this.planningRequestPacket.get();

      planningRequestPacket.setHorizonLength(horizonLength.getDoubleValue());
      planningRequestPacket.setRequestedFootstepPlannerType(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR.toByte());
      planningRequestPacket.setStartTargetType(QuadrupedFootstepPlanningRequestPacket.FOOTSTEP_PLANNER_TARGET_TYPE_FOOTSTEPS);
      planningRequestPacket.setTimeout(timeout.getDoubleValue());
      planningRequestPacket.getGoalPositionInWorld().set(continuousPlanningRequestPacket.getGoalPositionInWorld());
      planningRequestPacket.getGoalOrientationInWorld().set(continuousPlanningRequestPacket.getGoalOrientationInWorld());
      planningRequestPacket.getPlanarRegionsListMessage().set(latestPlanarRegions.get());

      reportMessage(planningRequestPacket);
   }

   private void removeQuadrantFromFixedStepQueue(RobotQuadrant quadrant)
   {
      fixedStepQueue.remove(getFirstStepInQuadrantFromList(quadrant, fixedStepQueue));
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
