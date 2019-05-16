package us.ihmc.quadrupedRobotics.messageHandling;

import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayDeque;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SoleTrajectoryCommand;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.YoQuadrupedTimedStep;
import us.ihmc.robotics.lists.YoPreallocatedList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedStepMessageHandler
{
   private static final double timeEpsilonForStepSelection = 0.05;
   private static final int STEP_QUEUE_SIZE = 200;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrantDependentList<RecyclingArrayDeque<SoleTrajectoryCommand>> upcomingFootTrajectoryCommandList = new QuadrantDependentList<>();

   private final YoInteger numberOfStepsToRecover = new YoInteger("numberOfStepsToRecover", registry);
   private final YoDouble initialTransferDurationForShifting = new YoDouble("initialTransferDurationForShifting", registry);

   private final ArrayList<YoQuadrupedTimedStep> activeSteps = new ArrayList<>();
   private final YoDouble robotTimestamp;
   private final DoubleParameter haltTransitionDurationParameter = new DoubleParameter("haltTransitionDuration", registry, 0.0);
   private final QuadrantDependentList<MutableBoolean> touchdownTrigger = new QuadrantDependentList<>(MutableBoolean::new);
   private final YoPreallocatedList<YoQuadrupedTimedStep> receivedStepSequence;

   private final YoDouble haltTime = new YoDouble("haltTime", registry);
   private final YoBoolean haltFlag = new YoBoolean("haltFlag", registry);

   private final YoBoolean stepPlanIsAdjustable = new YoBoolean("stepPlanIsAdjustable", registry);

   private final double controlDt;

   public QuadrupedStepMessageHandler(YoDouble robotTimestamp, double controlDt, YoVariableRegistry parentRegistry)
   {
      this.robotTimestamp = robotTimestamp;
      this.controlDt = controlDt;
      this.receivedStepSequence = new YoPreallocatedList<>(YoQuadrupedTimedStep.class, "receivedStepSequence", STEP_QUEUE_SIZE, registry);

      initialTransferDurationForShifting.set(1.00);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         upcomingFootTrajectoryCommandList.put(robotQuadrant, new RecyclingArrayDeque<>(SoleTrajectoryCommand.class, SoleTrajectoryCommand::set));

      // the look-ahead step adjustment was doing integer division which was 1.0 for step 0 and 0.0 after, so effectively having a one step recovery
      // TODO tune this value
      numberOfStepsToRecover.set(1);

      parentRegistry.addChild(registry);
   }

   public boolean isStepPlanAvailable()
   {
      return receivedStepSequence.size() > 0;
   }

   public void process()
   {
      if (haltFlag.getBooleanValue())
         pruneHaltedSteps();

      updateActiveSteps();
   }

   public void handleQuadrupedTimedStepListCommand(QuadrupedTimedStepListCommand command)
   {
      double currentTime = robotTimestamp.getDoubleValue();
      boolean isExpressedInAbsoluteTime = command.isExpressedInAbsoluteTime();
      RecyclingArrayList<QuadrupedTimedStepCommand> stepCommands = command.getStepCommands();

      stepPlanIsAdjustable.set(command.isStepPlanAdjustable());

      receivedStepSequence.clear();
      for (int i = 0; i < Math.min(stepCommands.size(), STEP_QUEUE_SIZE); i++)
      {
         double timeShift = isExpressedInAbsoluteTime ? 0.0 : currentTime + initialTransferDurationForShifting.getDoubleValue();
         double touchdownTime = stepCommands.get(i).getTimeIntervalCommand().getEndTime();
         if (touchdownTime + timeShift >= currentTime)
         {
            receivedStepSequence.add();
            YoQuadrupedTimedStep step = receivedStepSequence.get(receivedStepSequence.size() - 1);
            step.set(stepCommands.get(i));
            step.getTimeInterval().shiftInterval(timeShift);
         }
      }

      receivedStepSequence.sort(TimeIntervalTools.endTimeComparator);
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

   public void clearFootTrajectory(RobotQuadrant robotQuadrant)
   {
      upcomingFootTrajectoryCommandList.get(robotQuadrant).clear();
   }

   public void clearFootTrajectory()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         clearFootTrajectory(robotQuadrant);
   }

   private void pruneHaltedSteps()
   {
      TimeIntervalTools.removeStartTimesGreaterThan(haltTime.getDoubleValue(), receivedStepSequence);
      for (int i = receivedStepSequence.size() - 1; i >= 0; i--)
      {
         if (receivedStepSequence.get(i).getTimeInterval().getStartTime() > haltTime.getDoubleValue())
            receivedStepSequence.remove(i);
      }
   }

   public void initialize()
   {
      haltFlag.set(false);
   }

   public boolean isDoneWithStepSequence()
   {
      return receivedStepSequence.size() == 0 || receivedStepSequence.get(receivedStepSequence.size() - 1).getTimeInterval().getEndTime() < robotTimestamp
            .getDoubleValue();
   }

   public boolean isStepPlanAdjustable()
   {
      return stepPlanIsAdjustable.getBooleanValue();
   }

   public void onTouchDown(RobotQuadrant robotQuadrant)
   {
      touchdownTrigger.get(robotQuadrant).setTrue();
   }

   private final FramePoint3D tempStep = new FramePoint3D();

   // Fixme this isn't working properly anymore
   public void shiftPlanBasedOnStepAdjustment(FrameVector3DReadOnly stepAdjustment)
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

   public void halt()
   {
      if (!haltFlag.getBooleanValue())
      {
         haltFlag.set(true);
         haltTime.set(robotTimestamp.getDoubleValue() + haltTransitionDurationParameter.getValue());
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

   private void updateActiveSteps()
   {
      // remove steps with a triggered touchdown
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (touchdownTrigger.get(robotQuadrant).isTrue())
         {
            int index = getIndexOfFirstStep(robotQuadrant, timeEpsilonForStepSelection);
            if (index != -1)
               receivedStepSequence.remove(index);

            touchdownTrigger.get(robotQuadrant).setFalse();
         }
      }


      activeSteps.clear();

      for (int i = 0; i < receivedStepSequence.size(); i++)
      {
         double currentTime = robotTimestamp.getDoubleValue();
         double startTime = receivedStepSequence.get(i).getTimeInterval().getStartTime();
         double endTime = receivedStepSequence.get(i).getTimeInterval().getEndTime();

         // add all steps by start time
         if (currentTime >= startTime)
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
      haltFlag.set(false);
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
