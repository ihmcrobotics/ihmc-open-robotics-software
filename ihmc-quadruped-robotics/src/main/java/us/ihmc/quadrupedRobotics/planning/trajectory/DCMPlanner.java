package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceController;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedContactSequence;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.robotics.dataStructures.parameter.DoubleParameter;
import us.ihmc.robotics.dataStructures.parameter.ParameterFactory;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class DCMPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final int STEP_SEQUENCE_CAPACITY = 100;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final QuadrupedPiecewiseConstantCopTrajectory piecewiseConstanceCopTrajectory;
   private final PiecewiseReverseDcmTrajectory dcmTrajectory;
   private final FrameTrajectory3D dcmTransitionTrajectory;

   private final ParameterFactory parameterFactory = ParameterFactory.createWithRegistry(getClass(), registry);
   private final DoubleParameter initialTransitionDurationParameter = parameterFactory.createDouble("initialTransitionDuration", 0.5);

   private final QuadrupedTimedContactSequence timedContactSequence = new QuadrupedTimedContactSequence(4, 2 * STEP_SEQUENCE_CAPACITY);
   private final List<QuadrupedTimedStep> stepSequence = new ArrayList<>();

   private final QuadrantDependentList<FramePoint3D> currentSolePositions;

   private final YoDouble robotTimestamp;
   private final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);

   private final ReferenceFrame supportFrame;
   private final FramePoint3D finalDesiredDCM = new FramePoint3D();

   private final FramePoint3D tempPoint = new FramePoint3D();


   public DCMPlanner(double gravity, double nominalHeight, YoDouble robotTimestamp, ReferenceFrame supportFrame,
                     QuadrantDependentList<FramePoint3D> currentSolePositions, YoVariableRegistry parentRegistry)
   {
      this.robotTimestamp = robotTimestamp;
      this.supportFrame = supportFrame;
      this.currentSolePositions = currentSolePositions;
      this.dcmTransitionTrajectory = new FrameTrajectory3D(6, worldFrame);
      dcmTrajectory = new PiecewiseReverseDcmTrajectory(STEP_SEQUENCE_CAPACITY, gravity, nominalHeight);
      piecewiseConstanceCopTrajectory = new QuadrupedPiecewiseConstantCopTrajectory(2 * STEP_SEQUENCE_CAPACITY);

      parentRegistry.addChild(registry);
   }

   public void clearStepSequence()
   {
      stepSequence.clear();
   }

   public void setCoMHeight(double comHeight)
   {
      this.comHeight.set(comHeight);
   }

   public void addStepToSequence(QuadrupedTimedStep step)
   {
      stepSequence.add(step);
   }

   public void addStepsToSequence(List<? extends QuadrupedTimedStep> steps)
   {
      for (int i = 0; i < steps.size(); i++)
         addStepToSequence(steps.get(i));
   }

   public void initializeDcmSetpoints(QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings, FramePoint3DReadOnly dcmPosition)
   {
      timedContactSequence.initialize();

      double currentTime = robotTimestamp.getDoubleValue();
      if (!stepSequence.isEmpty() && stepSequence.get(stepSequence.size() - 1).getTimeInterval().getEndTime() > currentTime)
      {
         // compute dcm trajectory
         computeDcmTrajectory(taskSpaceControllerSettings);
         double transitionEndTime = piecewiseConstanceCopTrajectory.getTimeAtStartOfInterval(1);
         double transitionStartTime = Math.max(currentTime, transitionEndTime - initialTransitionDurationParameter.get());
         dcmTrajectory.computeTrajectory(transitionEndTime);
         dcmTrajectory.getPosition(finalDesiredDCM);
         dcmTransitionTrajectory.setQuinticWithZeroTerminalVelocityAndAcceleration(transitionStartTime, transitionEndTime, dcmPosition, finalDesiredDCM);
      }
   }

   private void computeDcmTrajectory(QuadrupedTaskSpaceController.Settings taskSpaceControllerSettings)
   {
      if (!stepSequence.isEmpty())
      {
         // compute piecewise constant center of pressure plan
         double currentTime = robotTimestamp.getDoubleValue();
         QuadrantDependentList<ContactState> currentContactState = taskSpaceControllerSettings.getContactState();
         timedContactSequence.update(stepSequence, currentSolePositions, currentContactState, currentTime);
         piecewiseConstanceCopTrajectory.initializeTrajectory(timedContactSequence);

         // compute dcm trajectory with final boundary constraint
         int numberOfIntervals = piecewiseConstanceCopTrajectory.getNumberOfIntervals();
         tempPoint.setIncludingFrame(piecewiseConstanceCopTrajectory.getCopPositionAtStartOfInterval(numberOfIntervals - 1));
         tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
         tempPoint.add(0, 0, comHeight.getDoubleValue());

         dcmTrajectory.setComHeight(comHeight.getDoubleValue());
         dcmTrajectory.initializeTrajectory(numberOfIntervals, piecewiseConstanceCopTrajectory.getTimeAtStartOfInterval(), piecewiseConstanceCopTrajectory.getCopPositionAtStartOfInterval(),
                                            piecewiseConstanceCopTrajectory.getTimeAtStartOfInterval(numberOfIntervals - 1), tempPoint);
      }
   }

   public void computeDcmSetpoints(FramePoint3D desiredDCMPositionToPack, FrameVector3D desiredDCMVelocityToPack)
   {
      if (stepSequence.isEmpty())
      {
         // update desired dcm position
         desiredDCMPositionToPack.setToZero(supportFrame);
         desiredDCMVelocityToPack.setToZero(supportFrame);
      }
      else if (robotTimestamp.getDoubleValue() <= dcmTransitionTrajectory.getFinalTime())
      {
         dcmTransitionTrajectory.compute(robotTimestamp.getDoubleValue());
         dcmTransitionTrajectory.getFramePosition(desiredDCMPositionToPack);
         dcmTransitionTrajectory.getFrameVelocity(desiredDCMVelocityToPack);
      }
      else
      {
         dcmTrajectory.computeTrajectory(robotTimestamp.getDoubleValue());
         dcmTrajectory.getPosition(desiredDCMPositionToPack);
         dcmTrajectory.getVelocity(desiredDCMVelocityToPack);
      }
   }

   public void getFinalDesiredDCM(FixedFramePoint3DBasics finalDesiredDCMToPack)
   {
      tempPoint.setIncludingFrame(finalDesiredDCM);
      tempPoint.changeFrame(finalDesiredDCMToPack.getReferenceFrame());
      finalDesiredDCMToPack.set(tempPoint);
   }
}
