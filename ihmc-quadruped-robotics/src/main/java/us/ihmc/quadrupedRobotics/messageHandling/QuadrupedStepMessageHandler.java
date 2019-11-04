package us.ihmc.quadrupedRobotics.messageHandling;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableBoolean;

import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PauseWalkingCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SoleTrajectoryCommand;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.YoQuadrupedTimedStep;
import us.ihmc.robotics.lists.YoPreallocatedList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class QuadrupedStepMessageHandler
{
   private static final double timeEpsilonForStepSelection = 0.05;
   private static final int STEP_QUEUE_SIZE = 200;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrantDependentList<RecyclingArrayDeque<SoleTrajectoryCommand>> upcomingFootTrajectoryCommandList = new QuadrantDependentList<>();

   private final YoInteger numberOfStepsToRecover = new YoInteger("numberOfStepsToRecover", registry);
   private final BooleanProvider shiftTimesBasedOnLateTouchdown = new BooleanParameter("shiftTimesBasedOnLateTouchdown", registry, true);
   private final YoDouble initialTransferDurationForShifting = new YoDouble("initialTransferDurationForShifting", registry);
   private final YoBoolean isPaused = new YoBoolean("isPaused", registry);
   private final YoDouble pauseTime = new YoDouble("pauseTime", registry);

   private final ArrayList<YoQuadrupedTimedStep> activeSteps = new ArrayList<>();
   private final YoDouble robotTimestamp;
   private final QuadrantDependentList<MutableBoolean> touchdownTrigger = new QuadrantDependentList<>(MutableBoolean::new);
   private final YoPreallocatedList<YoQuadrupedTimedStep> receivedStepSequence;
   private final QuadrantDependentList<YoQuadrupedTimedStep> mostRecentCompletedStep = new QuadrantDependentList<>();

   private final YoBoolean stepPlanIsAdjustable = new YoBoolean("stepPlanIsAdjustable", registry);

   private final YoBoolean offsettingHeightPlanWithStepError = new YoBoolean("offsettingHeightPlanWithStepError", registry);
   private final DoubleParameter offsetHeightCorrectionScale = new DoubleParameter("stepHeightCorrectionErrorScaleFactor", registry, 0.25);

   private final double controlDt;

   public QuadrupedStepMessageHandler(YoDouble robotTimestamp, double controlDt, YoVariableRegistry parentRegistry)
   {
      this.robotTimestamp = robotTimestamp;
      this.controlDt = controlDt;
      this.receivedStepSequence = new YoPreallocatedList<>(YoQuadrupedTimedStep.class, "receivedStepSequence", STEP_QUEUE_SIZE, registry);

      initialTransferDurationForShifting.set(1.00);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         upcomingFootTrajectoryCommandList.put(robotQuadrant, new RecyclingArrayDeque<>(SoleTrajectoryCommand.class, SoleTrajectoryCommand::set));
         this.mostRecentCompletedStep.put(robotQuadrant, new YoQuadrupedTimedStep(robotQuadrant.getShortName() + "_LastCompletedStep", registry));
      }

      // the look-ahead step adjustment was doing integer division which was 1.0 for step 0 and 0.0 after, so effectively having a one step recovery
      // TODO tune this value
      numberOfStepsToRecover.set(1);

      parentRegistry.addChild(registry);
   }

   public boolean isStepPlanAvailable()
   {
      return !isPaused.getBooleanValue() && !receivedStepSequence.isEmpty();
   }

   private final TimeInterval tempTimeInterval = new TimeInterval();

   public void handleQuadrupedTimedStepListCommand(QuadrupedTimedStepListCommand command)
   {
      if (!isValidStepPlan(command))
      {
         return;
      }

      // if paused, resume when new steps are received
      if (isPaused.getBooleanValue())
      {
         isPaused.set(false);
         pauseTime.set(Double.NaN);
      }

      double currentTime = robotTimestamp.getDoubleValue();
      boolean isExpressedInAbsoluteTime = command.isExpressedInAbsoluteTime();
      RecyclingArrayList<QuadrupedTimedStepCommand> stepCommands = command.getStepCommands();

      stepPlanIsAdjustable.set(command.isStepPlanAdjustable());
      offsettingHeightPlanWithStepError.set(command.getOffsetStepsHeightWithExecutionError());

      receivedStepSequence.clear();
      for (int i = 0; i < Math.min(stepCommands.size(), STEP_QUEUE_SIZE); i++)
      {
         double timeShift = isExpressedInAbsoluteTime ? 0.0 : currentTime + initialTransferDurationForShifting.getDoubleValue();
         double touchdownTime = stepCommands.get(i).getTimeIntervalCommand().getEndTime();
         if (touchdownTime + timeShift >= currentTime)
         {
            QuadrupedTimedStepCommand stepCommand = stepCommands.get(i);
            QuadrupedTimedStep mostRecentStepOnThatQuadrant = mostRecentCompletedStep.get(stepCommand.getRobotQuadrant());
            stepCommand.getTimeIntervalCommand().getTimeInterval(tempTimeInterval);
            tempTimeInterval.shiftInterval(timeShift);

            if (TimeIntervalTools.doIntervalsOverlap(mostRecentStepOnThatQuadrant.getTimeInterval(), tempTimeInterval))
               continue;

            receivedStepSequence.add();
            YoQuadrupedTimedStep step = receivedStepSequence.get(receivedStepSequence.size() - 1);
            step.set(stepCommand);
            step.getTimeInterval().shiftInterval(timeShift);
         }
      }

      receivedStepSequence.sort(TimeIntervalTools.endTimeComparator);
   }

   private static boolean isValidStepPlan(QuadrupedTimedStepListCommand command)
   {
      double maximumStepTranslation = 1.0;
      RecyclingArrayList<QuadrupedTimedStepCommand> stepCommands = command.getStepCommands();

      for (int i = 0; i < command.getNumberOfSteps(); i++)
      {
         QuadrupedTimedStepCommand stepCommand = stepCommands.get(i);
         for (int j = i + 1; j < command.getNumberOfSteps(); j++)
         {
            if(stepCommands.get(j).getRobotQuadrant() == stepCommands.get(i).getRobotQuadrant())
            {
               QuadrupedTimedStepCommand nextStepCommand = stepCommands.get(j);
               if(nextStepCommand.getGoalPosition().distance(stepCommand.getGoalPosition()) > maximumStepTranslation)
               {
                  return false;
               }
               else
               {
                  break;
               }
            }
         }
      }

      return true;
   }

   public void clearSteps()
   {
      receivedStepSequence.clear();
      activeSteps.clear();
   }

   public void handleSoleTrajectoryCommand(List<SoleTrajectoryCommand> commands)
   {
      for (int i = 0; i < commands.size(); i++)
      {
         SoleTrajectoryCommand command = commands.get(i);
         upcomingFootTrajectoryCommandList.get(command.getRobotQuadrant()).addLast(command);
      }
   }

   public SoleTrajectoryCommand pollFootTrajectoryForSolePositionControl(RobotQuadrant swingQuadrant)
   {
      return upcomingFootTrajectoryCommandList.get(swingQuadrant).poll();
   }

   public boolean hasFootTrajectoryForSolePositionControl(RobotQuadrant swingQuadrant)
   {
      return !upcomingFootTrajectoryCommandList.get(swingQuadrant).isEmpty();
   }

   public boolean hasFootTrajectoryForSolePositionControl()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (hasFootTrajectoryForSolePositionControl(robotQuadrant))
            return true;
      }

      return false;
   }

   public void handlePauseWalkingCommand(PauseWalkingCommand pauseWalkingCommand)
   {
      if(isPaused.getBooleanValue() == pauseWalkingCommand.isPauseRequested())
      {
         return;
      }

      this.isPaused.set(pauseWalkingCommand.isPauseRequested());

      if (pauseWalkingCommand.isPauseRequested())
      {
         pauseTime.set(robotTimestamp.getDoubleValue());
      }
      else
      {
         double earliestStartTime = Double.POSITIVE_INFINITY;
         for (int i = 0; i < receivedStepSequence.size(); i++)
         {
            double startTime = receivedStepSequence.get(i).getTimeInterval().getStartTime();
            if (startTime < earliestStartTime)
               earliestStartTime = startTime;
         }

         double timeShift = robotTimestamp.getDoubleValue() + initialTransferDurationForShifting.getDoubleValue() - earliestStartTime;
         for (int i = 0; i < receivedStepSequence.size(); i++)
         {
            receivedStepSequence.get(i).getTimeInterval().shiftInterval(timeShift);
         }
      }
   }

   public void clearUpcomingSteps()
   {
      TimeIntervalTools.removeStartTimesGreaterThan(robotTimestamp.getDoubleValue(), receivedStepSequence);
   }

   public void clearFootTrajectory(RobotQuadrant robotQuadrant)
   {
      upcomingFootTrajectoryCommandList.get(robotQuadrant).clear();
   }

   public void clearFootTrajectory()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         clearFootTrajectory(robotQuadrant);
   }

   public void initialize()
   {
      isPaused.set(false);
      pauseTime.set(Double.NaN);
   }

   public boolean isDoneWithStepSequence()
   {
      // if paused, wait for current steps to finish
      if (isPaused.getBooleanValue() && activeSteps.isEmpty())
      {
         return true;
      }

      // otherwise only done when step queue is empty
      return receivedStepSequence.isEmpty();
   }

   public boolean isStepPlanAdjustable()
   {
      return stepPlanIsAdjustable.getBooleanValue();
   }

   public void onTouchDown(RobotQuadrant robotQuadrant)
   {
      touchdownTrigger.get(robotQuadrant).setTrue();
   }

   public void shiftPlanTimeBasedOnTouchdown(RobotQuadrant robotQuadrant, double currentTime)
   {
      int index = getIndexOfFirstStep(robotQuadrant, timeEpsilonForStepSelection);
      if (index == -1 || !shiftTimesBasedOnLateTouchdown.getValue())
         return;

      QuadrupedTimedStep completedStep = receivedStepSequence.remove(index);
      double stepDelay = currentTime - completedStep.getTimeInterval().getEndTime();
      if (stepDelay > 0.0)
      {
         for (int i = 0; i < receivedStepSequence.size(); i++)
         {
            receivedStepSequence.get(i).getTimeInterval().shiftInterval(stepDelay);
         }
      }
   }

   private final FramePoint3D tempStep = new FramePoint3D();

   // Fixme this isn't working properly anymore
   public void shiftPlanPositionBasedOnStepAdjustment(FrameVector3DReadOnly stepAdjustment)
   {
      int numberOfStepsToAdjust = Math.min(numberOfStepsToRecover.getIntegerValue(), receivedStepSequence.size());
      for (int i = 0; i < numberOfStepsToAdjust; i++)
      {
         double multiplier = (numberOfStepsToRecover.getIntegerValue() - i) / (double) numberOfStepsToRecover.getIntegerValue();
         tempStep.setIncludingFrame(receivedStepSequence.get(i).getReferenceFrame(), receivedStepSequence.get(i).getGoalPosition());
         tempStep.scaleAdd(multiplier, stepAdjustment, tempStep);
         receivedStepSequence.get(i).setGoalPosition(tempStep);
      }
   }

   private final FrameVector3D stepOffsetVector = new FrameVector3D();

   public void addOffsetVectorOnTouchdown(FrameVector3DReadOnly offset)
   {
      if (!offsettingHeightPlanWithStepError.getValue())
      {
         return;
      }

      stepOffsetVector.setIncludingFrame(offset);
      stepOffsetVector.changeFrame(ReferenceFrame.getWorldFrame());

      stepOffsetVector.setX(0.0);
      stepOffsetVector.setY(0.0);
      stepOffsetVector.scale(offsetHeightCorrectionScale.getValue());

      for (int i = 0; i < receivedStepSequence.size(); i++)
      {
         YoQuadrupedTimedStep step = receivedStepSequence.get(i);
         tempStep.setIncludingFrame(step.getReferenceFrame(), step.getGoalPosition());
         tempStep.add(stepOffsetVector);
         step.setGoalPosition(tempStep);
      }
   }

   public YoPreallocatedList<YoQuadrupedTimedStep> getStepSequence()
   {
      return receivedStepSequence;
   }

   public ArrayList<YoQuadrupedTimedStep> getActiveSteps()
   {
      return activeSteps;
   }

   public void updateActiveSteps()
   {
      // remove steps with a triggered touchdown
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (touchdownTrigger.get(robotQuadrant).isTrue())
         {
            int index = getIndexOfFirstStep(robotQuadrant, timeEpsilonForStepSelection);
            if (index != -1)
            {
               QuadrupedTimedStep completedStep = receivedStepSequence.remove(index);
               mostRecentCompletedStep.get(completedStep.getRobotQuadrant()).set(completedStep);
            }

            touchdownTrigger.get(robotQuadrant).setFalse();
         }
      }

      activeSteps.clear();

      double currentTime = robotTimestamp.getDoubleValue();
      double activeStepTimeThreshold = isPaused.getBooleanValue() ? pauseTime.getDoubleValue() : currentTime;

      for (int i = 0; i < receivedStepSequence.size(); i++)
      {
         double startTime = receivedStepSequence.get(i).getTimeInterval().getStartTime();
         double endTime = receivedStepSequence.get(i).getTimeInterval().getEndTime();

         // add all steps by start time
         if (activeStepTimeThreshold >= startTime)
            activeSteps.add(receivedStepSequence.get(i));

         // extend timing of steps with expired end time
         if (currentTime >= endTime)
            receivedStepSequence.get(i).getTimeInterval().setEndTime(currentTime + controlDt);
      }
   }

   public QuadrupedTimedStep getNextStep()
   {
      for (int i = 0; i < receivedStepSequence.size(); i++)
      {
         double currentTime = robotTimestamp.getDoubleValue();
         double endTime = receivedStepSequence.get(i).getTimeInterval().getEndTime();

         if (currentTime < endTime)
         {
            return receivedStepSequence.get(i);
         }
      }

      return null;
   }

   public void reset()
   {
      receivedStepSequence.clear();
   }

   private int getIndexOfFirstStep(RobotQuadrant robotQuadrant, double timeEpsilon)
   {
      for (int i = 0;i  < receivedStepSequence.size(); i++)
      {
         QuadrupedTimedStep step = receivedStepSequence.get(i);

         if (step.getRobotQuadrant() == robotQuadrant && step.getTimeInterval().epsilonContains(robotTimestamp.getDoubleValue(), timeEpsilon))
            return i;
      }

      return -1;
   }
}
