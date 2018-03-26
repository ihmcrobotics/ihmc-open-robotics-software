package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedContactSequence;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class DCMPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final boolean VISUALIZE = false;
   private static final double POINT_SIZE = 0.005;

   private static final int STEP_SEQUENCE_CAPACITY = 50;

   private final QuadrupedPiecewiseConstantCopTrajectory piecewiseConstantCopTrajectory;
   private final PiecewiseReverseDcmTrajectory dcmTrajectory;
   private final FrameTrajectory3D dcmTransitionTrajectory;

   private final DoubleParameter initialTransitionDurationParameter = new DoubleParameter("initialTransitionDuration", registry, 0.5);

   private final QuadrupedTimedContactSequence timedContactSequence = new QuadrupedTimedContactSequence(4, 2 * STEP_SEQUENCE_CAPACITY);
   private final List<QuadrupedTimedStep> stepSequence = new ArrayList<>();

   private final QuadrantDependentList<MovingReferenceFrame> soleFrames;

   private final YoDouble robotTimestamp;
   private final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);
   private final YoInteger numberOfStepsInPlanner = new YoInteger("numberOfStepsInPlanner", registry);

   private final YoBoolean isStanding = new YoBoolean("isStanding", registry);

   private final ReferenceFrame supportFrame;
   private final FramePoint3D dcmPositionAtStartOfState = new FramePoint3D();
   private final FramePoint3D finalDesiredDCM = new FramePoint3D();

   private final FramePoint3D tempPoint = new FramePoint3D();

   public DCMPlanner(double gravity, double nominalHeight, YoDouble robotTimestamp, ReferenceFrame supportFrame,
                     QuadrantDependentList<MovingReferenceFrame> soleFrames, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robotTimestamp = robotTimestamp;
      this.supportFrame = supportFrame;
      this.soleFrames = soleFrames;
      this.dcmTransitionTrajectory = new FrameTrajectory3D(6, supportFrame);
      dcmTrajectory = new PiecewiseReverseDcmTrajectory(STEP_SEQUENCE_CAPACITY, gravity, nominalHeight, registry);
      piecewiseConstantCopTrajectory = new QuadrupedPiecewiseConstantCopTrajectory(2 * STEP_SEQUENCE_CAPACITY, registry);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      piecewiseConstantCopTrajectory.setupVisualizers(yoGraphicsList, artifactList, POINT_SIZE);
      dcmTrajectory.setupVisualizers(yoGraphicsList, artifactList, POINT_SIZE);

      artifactList.setVisible(VISUALIZE);
      yoGraphicsList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void clearStepSequence()
   {
      stepSequence.clear();
      numberOfStepsInPlanner.set(0);
   }

   public void setCoMHeight(double comHeight)
   {
      this.comHeight.set(comHeight);
   }

   public void addStepToSequence(QuadrupedTimedStep step)
   {
      stepSequence.add(step);
      numberOfStepsInPlanner.increment();
   }

   public void initializeForStanding()
   {
      isStanding.set(true);
      piecewiseConstantCopTrajectory.resetVariables();
      dcmTrajectory.resetVariables();
   }

   public void initializeForStepping(QuadrantDependentList<ContactState> currentContactStates, FramePoint3DReadOnly dcmPosition)
   {
      isStanding.set(false);

      double currentTime = robotTimestamp.getDoubleValue();
      boolean isCurrentPlanValid = stepSequence.get(numberOfStepsInPlanner.getIntegerValue() - 1).getTimeInterval().getEndTime() > currentTime;

      if (isCurrentPlanValid)
      {
         // compute dcm trajectory
         computeDcmTrajectory(currentContactStates);

         dcmPositionAtStartOfState.setIncludingFrame(dcmPosition);
         computeTransitionTrajectory();
      }
   }

   private void computeDcmTrajectory(QuadrantDependentList<ContactState> currentContactStates)
   {
      // compute piecewise constant center of pressure plan
      double currentTime = robotTimestamp.getDoubleValue();
      timedContactSequence.update(stepSequence, soleFrames, currentContactStates, currentTime);
      piecewiseConstantCopTrajectory.initializeTrajectory(timedContactSequence);

      // compute dcm trajectory with final boundary constraint
      int numberOfIntervals = piecewiseConstantCopTrajectory.getNumberOfIntervals();
      tempPoint.setIncludingFrame(piecewiseConstantCopTrajectory.getCopPositionAtStartOfInterval(numberOfIntervals - 1));
      tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
      tempPoint.add(0, 0, comHeight.getDoubleValue());

      dcmTrajectory.setComHeight(comHeight.getDoubleValue());
      dcmTrajectory.initializeTrajectory(numberOfIntervals, piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(),
                                         piecewiseConstantCopTrajectory.getCopPositionsAtStartOfInterval(),
                                         piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(numberOfIntervals - 1), tempPoint);
   }

   private void computeTransitionTrajectory()
   {
      double transitionEndTime = piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(1);
      double transitionStartTime = Math.max(robotTimestamp.getDoubleValue(), transitionEndTime - initialTransitionDurationParameter.getValue());
      dcmTrajectory.computeTrajectory(transitionEndTime);
      dcmTrajectory.getPosition(finalDesiredDCM);

      dcmPositionAtStartOfState.changeFrame(dcmTransitionTrajectory.getReferenceFrame());
      finalDesiredDCM.changeFrame(dcmTransitionTrajectory.getReferenceFrame());
      dcmTransitionTrajectory
            .setQuinticWithZeroTerminalVelocityAndAcceleration(transitionStartTime, transitionEndTime, dcmPositionAtStartOfState, finalDesiredDCM);
   }

   private final FramePoint3D desiredDCMPosition = new FramePoint3D();
   private final FrameVector3D desiredDCMVelocity = new FrameVector3D();

   public void computeDcmSetpoints(QuadrantDependentList<ContactState> currentContactStates, FixedFramePoint3DBasics desiredDCMPositionToPack,
                                   FixedFrameVector3DBasics desiredDCMVelocityToPack)
   {
      if (isStanding.getBooleanValue())
      {
         // update desired dcm position
         desiredDCMPosition.setToZero(supportFrame);
         desiredDCMVelocity.setToZero(supportFrame);
      }
      else
      {
         computeDcmTrajectory(currentContactStates);

         double currentTime = robotTimestamp.getDoubleValue();
         if (currentTime <= dcmTransitionTrajectory.getFinalTime())
         {
            computeTransitionTrajectory();

            dcmTransitionTrajectory.compute(robotTimestamp.getDoubleValue());
            dcmTransitionTrajectory.getFramePosition(desiredDCMPosition);
            dcmTransitionTrajectory.getFrameVelocity(desiredDCMVelocity);
         }
         else
         {
            dcmTrajectory.computeTrajectory(robotTimestamp.getDoubleValue());
            dcmTrajectory.getPosition(desiredDCMPosition);
            dcmTrajectory.getVelocity(desiredDCMVelocity);
         }
      }

      desiredDCMPosition.changeFrame(desiredDCMPositionToPack.getReferenceFrame());
      desiredDCMVelocity.changeFrame(desiredDCMVelocityToPack.getReferenceFrame());

      desiredDCMPositionToPack.set(desiredDCMPosition);
      desiredDCMVelocityToPack.set(desiredDCMVelocity);
   }

   public void getFinalDesiredDCM(FixedFramePoint3DBasics finalDesiredDCMToPack)
   {
      tempPoint.setIncludingFrame(finalDesiredDCM);
      tempPoint.changeFrame(finalDesiredDCMToPack.getReferenceFrame());
      finalDesiredDCMToPack.set(tempPoint);
   }

   public double getFinalTime()
   {
      return dcmTransitionTrajectory.getFinalTime();
   }
}
