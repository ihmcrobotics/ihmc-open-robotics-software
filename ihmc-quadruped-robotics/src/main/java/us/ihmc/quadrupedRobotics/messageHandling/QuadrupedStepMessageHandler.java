package us.ihmc.quadrupedRobotics.messageHandling;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootTrajectoryCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SoleTrajectoryCommand;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.YoQuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.quadrupedRobotics.util.TimeIntervalTools;
import us.ihmc.quadrupedRobotics.util.YoPreallocatedList;
import us.ihmc.robotics.lists.RecyclingArrayDeque;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedStepMessageHandler
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int STEP_QUEUE_SIZE = 40;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrantDependentList<RecyclingArrayDeque<SoleTrajectoryCommand>> upcomingFootTrajectoryCommandList = new QuadrantDependentList<>();

   private final QuadrupedReferenceFrames referenceFrames;
   private final YoFrameOrientation bodyOrientation;
   private final ArrayList<YoQuadrupedTimedStep> activeSteps = new ArrayList<>();
   private final YoDouble robotTimestamp;
   private final DoubleParameter haltTransitionDurationParameter = new DoubleParameter("haltTransitionDuration", registry, 0.0);
   private final YoPreallocatedList<YoQuadrupedTimedStep> receivedStepSequence;
   private final YoPreallocatedList<YoQuadrupedTimedStep> adjustedStepSequence;

   private final YoDouble haltTime = new YoDouble("haltTime", registry);
   private final YoBoolean haltFlag = new YoBoolean("haltFlag", registry);

   private final FramePoint3D tempPoint = new FramePoint3D();

   public QuadrupedStepMessageHandler(YoDouble robotTimestamp, QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      this.robotTimestamp = robotTimestamp;
      this.receivedStepSequence = new YoPreallocatedList<>("receivedStepSequence", registry, STEP_QUEUE_SIZE, YoQuadrupedTimedStep::new);
      this.adjustedStepSequence = new YoPreallocatedList<>("adjustedStepSequence", registry, STEP_QUEUE_SIZE, YoQuadrupedTimedStep::new);
      this.referenceFrames = referenceFrames;
      this.bodyOrientation = new YoFrameOrientation("bodyOrientation", ReferenceFrame.getWorldFrame(), registry);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         upcomingFootTrajectoryCommandList.put(robotQuadrant, new RecyclingArrayDeque<>(SoleTrajectoryCommand.class));

      parentRegistry.addChild(registry);
   }

   public boolean isStepPlanAvailable()
   {
      return receivedStepSequence.size() > 0;
   }

   public void getBodyOrientation(FrameQuaternion bodyOrientation)
   {
      bodyOrientation.setIncludingFrame(this.bodyOrientation.getFrameOrientation());
   }

   public FrameQuaternionReadOnly getBodyOrientation()
   {
      return bodyOrientation.getFrameOrientation();
   }

   /**
    * Consumes incoming footsteps and adjusts their position by the given vector
    */
   public void process(FrameVector3DReadOnly stepAdjustment)
   {
      TimeIntervalTools.removeEndTimesLessThan(robotTimestamp.getDoubleValue(), receivedStepSequence);
      if (haltFlag.getBooleanValue())
         pruneHaltedSteps();

      updateAdjustedStepQueue(stepAdjustment);
      updateActiveSteps();
   }

   public void handleQuadrupedTimedStepListCommand(QuadrupedTimedStepListCommand command)
   {
      double currentTime = robotTimestamp.getDoubleValue();
      boolean isExpressedInAbsoluteTime = command.isExpressedInAbsoluteTime();
      RecyclingArrayList<QuadrupedTimedStepCommand> stepCommands = command.getStepCommands();

      receivedStepSequence.clear();
      for (int i = 0; i < Math.min(stepCommands.size(), STEP_QUEUE_SIZE); i++)
      {
         double timeShift = isExpressedInAbsoluteTime ? 0.0 : currentTime;
         double touchdownTime = stepCommands.get(i).getTimeIntervalCommand().getEndTime();
         if (touchdownTime + timeShift >= currentTime)
         {
            receivedStepSequence.add();
            YoQuadrupedTimedStep step = receivedStepSequence.get(receivedStepSequence.size() - 1);
            step.set(stepCommands.get(i));
            step.getTimeInterval().shiftInterval(timeShift);
         }
      }

      TimeIntervalTools.sortByEndTime(receivedStepSequence);
      bodyOrientation.setFromReferenceFrame(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds());
   }

   public void clearSteps()
   {
      receivedStepSequence.clear();
      adjustedStepSequence.clear();
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

   private void updateAdjustedStepQueue(FrameVector3DReadOnly stepAdjustment)
   {
      adjustedStepSequence.clear();
      for (int i = 0; i < receivedStepSequence.size(); i++)
      {
         adjustedStepSequence.add();
         YoQuadrupedTimedStep receivedStep = receivedStepSequence.get(i);
         YoQuadrupedTimedStep adjustedStep = adjustedStepSequence.get(i);
         adjustedStep.set(receivedStep);

         adjustedStep.getGoalPosition(tempPoint);
         tempPoint.changeFrame(worldFrame);
         tempPoint.add(stepAdjustment);
         adjustedStep.setGoalPosition(tempPoint);
      }
   }

   public boolean isDoneWithStepSequence()
   {
      return receivedStepSequence.size() == 0 || receivedStepSequence.get(receivedStepSequence.size() - 1).getTimeInterval().getEndTime() < robotTimestamp
            .getDoubleValue();
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
      return adjustedStepSequence;
   }

   public ArrayList<YoQuadrupedTimedStep> getActiveSteps()
   {
      return activeSteps;
   }

   private void updateActiveSteps()
   {
      activeSteps.clear();

      for (int i = 0; i < adjustedStepSequence.size(); i++)
      {
         double currentTime = robotTimestamp.getDoubleValue();
         double startTime = adjustedStepSequence.get(i).getTimeInterval().getStartTime();
         double endTime = adjustedStepSequence.get(i).getTimeInterval().getEndTime();

         if (MathTools.intervalContains(currentTime, startTime, endTime))
         {
            activeSteps.add(adjustedStepSequence.get(i));
         }
      }
   }

   public void reset()
   {
      haltFlag.set(false);
      receivedStepSequence.clear();
      adjustedStepSequence.clear();
   }
}
