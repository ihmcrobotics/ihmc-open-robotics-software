package us.ihmc.quadrupedRobotics.planning.trajectory;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedContactSequence;
import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

import java.util.ArrayList;
import java.util.List;

public class ContinuousDCMPlanner implements DCMPlannerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private static final boolean VISUALIZE = false;
   private static final double POINT_SIZE = 0.005;

   private static final int STEP_SEQUENCE_CAPACITY = 50;

   private final QuadrupedPiecewiseConstantCopTrajectory piecewiseConstantCopTrajectory;
   private final PiecewiseReverseDcmTrajectory dcmTrajectory;
   private final YoFrameTrajectory3D dcmFirstSpline;
   private final YoFrameTrajectory3D dcmSecondSpline;

   private final DoubleParameter initialTransitionDuration = new DoubleParameter("initialTransitionDuration", registry, 0.25);
   private final DoubleParameter minimumSplineDuration = new DoubleParameter("minimumSplineDuration", registry, 0.05);
   private final DoubleParameter splineSegmentDuration = new DoubleParameter("splineSegmentDuration", registry, 0.1);
   private final DoubleParameter splineSplitFraction = new DoubleParameter("splineSplitFraction", registry, 0.5);

   private final QuadrupedTimedContactSequence timedContactSequence = new QuadrupedTimedContactSequence(2 * STEP_SEQUENCE_CAPACITY);
   private final List<QuadrupedTimedStep> stepSequence = new ArrayList<>();

   private final QuadrantDependentList<MovingReferenceFrame> soleFrames;

   private final YoDouble controllerTime;
   private final YoDouble omega;
   private final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);
   private final YoInteger numberOfStepsInPlanner = new YoInteger("numberOfStepsInPlanner", registry);

   private final YoBoolean isStanding = new YoBoolean("plannerIsStanding", registry);
   private final YoBoolean isInFirstSpline = new YoBoolean("plannerIsInFirstSpline", registry);
   private final YoBoolean isInSecondSpline = new YoBoolean("plannerIsInSecondSpline", registry);
   private final YoBoolean isInitialTransfer = new YoBoolean("plannerIsInitialTransfer", registry);
   private final YoDouble firstSplineStartTime = new YoDouble("firstSplineStartTime", registry);
   private final YoDouble firstSplineEndTime = new YoDouble("firstSplineEndTime", registry);
   private final YoDouble secondSplineStartTime = new YoDouble("secondSplineStartTime", registry);
   private final YoDouble secondSplineEndTime = new YoDouble("secondSplineEndTime", registry);

   private final ReferenceFrame supportFrame;

   private final YoFramePoint3D finalDCMPosition = new YoFramePoint3D("finalDCMPosition", worldFrame, registry);

   private final YoFramePoint3D dcmPositionAtStartOfFirstSpline = new YoFramePoint3D("dcmPositionAtStartOfFirstSpline", worldFrame, registry);
   private final YoFrameVector3D dcmVelocityAtStartOfFirstSpline = new YoFrameVector3D("dcmVelocityAtStartOfFirstSpline", worldFrame, registry);
   private final YoFramePoint3D dcmPositionAtEndOfFirstSpline = new YoFramePoint3D("dcmPositionAtEndOfFirstSpline", worldFrame, registry);
   private final YoFrameVector3D dcmVelocityAtEndOfFirstSpline = new YoFrameVector3D("dcmVelocityAtEndOfFirstSpline", worldFrame, registry);

   private final YoFramePoint3D dcmPositionAtStartOfSecondSpline = new YoFramePoint3D("dcmPositionAtStartOfSecondSpline", worldFrame, registry);
   private final YoFrameVector3D dcmVelocityAtStartOfSecondSpline = new YoFrameVector3D("dcmVelocityAtStartOfSecondSpline", worldFrame, registry);
   private final YoFramePoint3D dcmPositionAtEndOfSecondSpline = new YoFramePoint3D("dcmPositionAtEndOfSecondSpline", worldFrame, registry);
   private final YoFrameVector3D dcmVelocityAtEndOfSecondSpline = new YoFrameVector3D("dcmVelocityAtEndOfSecondSpline", worldFrame, registry);

   private final YoFramePoint3D dcmAtEndOfState = new YoFramePoint3D("dcmPositionAtEndOfState", worldFrame, registry);

   private final FramePoint3D splineInitialPosition;
   private final FrameVector3D splineInitialVelocity;
   private final FramePoint3D splineFinalPosition;
   private final FrameVector3D splineFinalVelocity;

   private final YoDouble timeAtStartOfState = new YoDouble("timeAtStartOfState", registry);
   private final YoDouble timeInState = new YoDouble("timeInState", registry);

   private final YoBoolean holdPosition = new YoBoolean("holdPosition", registry);

   private final YoFramePoint3D dcmPositionAtStartOfState = new YoFramePoint3D("dcmPositionAtStartOfState", worldFrame, registry);
   private final YoFrameVector3D dcmVelocityAtStartOfState = new YoFrameVector3D("dcmVelocityAtStartOfState", worldFrame, registry);

   private final YoFramePoint3D dcmPositionToHold = new YoFramePoint3D("dcmPositionToHold", worldFrame, registry);

   private final YoFramePoint3D desiredDCMPosition = new YoFramePoint3D("plannerDesiredDCMPosition", worldFrame, registry);
   private final YoFrameVector3D desiredDCMVelocity = new YoFrameVector3D("plannerDesiredDCMVelocity", worldFrame, registry);
   private final YoFramePoint3D desiredVRPPosition = new YoFramePoint3D("plannerPerfectVRPPosition", worldFrame, registry);
   private final YoFramePoint3D desiredECMPPosition = new YoFramePoint3D("plannerPerfectECMPPosition", worldFrame, registry);


   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FrameVector3D tempVector = new FrameVector3D();

   public ContinuousDCMPlanner(DCMPlannerParameters dcmPlannerParameters, YoDouble omega, double gravity, YoDouble robotTimestamp, ReferenceFrame supportFrame,
                               QuadrantDependentList<MovingReferenceFrame> soleFrames, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.omega = omega;
      this.controllerTime = robotTimestamp;
      this.supportFrame = supportFrame;
      this.soleFrames = soleFrames;
      this.dcmFirstSpline = new YoFrameTrajectory3D("dcmFirstSpline", 4, supportFrame, registry);
      this.dcmSecondSpline = new YoFrameTrajectory3D("dcmSecondSpline", 4, supportFrame, registry);

      splineInitialPosition = new FramePoint3D(supportFrame);
      splineInitialVelocity = new FrameVector3D(supportFrame);
      splineFinalPosition = new FramePoint3D(supportFrame);
      splineFinalVelocity = new FrameVector3D(supportFrame);

      omega.addVariableChangedListener(v ->
            comHeight.set(gravity / MathTools.square(omega.getDoubleValue())));

      DCMPlannerParameters yoDcmPlannerParameters = new YoDCMPlannerParameters(dcmPlannerParameters, registry);
      dcmTrajectory = new PiecewiseReverseDcmTrajectory(STEP_SEQUENCE_CAPACITY, omega, gravity, registry);
      piecewiseConstantCopTrajectory = new QuadrupedPiecewiseConstantCopTrajectory(2 * STEP_SEQUENCE_CAPACITY, yoDcmPlannerParameters, registry);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      piecewiseConstantCopTrajectory.setupVisualizers(yoGraphicsList, artifactList, POINT_SIZE);
      dcmTrajectory.setupVisualizers(yoGraphicsListRegistry, POINT_SIZE);

      double graphicSize = 0.007;
      YoGraphicPosition perfectCMPPositionViz = new YoGraphicPosition("Perfect CMP Position", desiredECMPPosition, graphicSize, YoAppearance.BlueViolet());
      YoGraphicPosition dcmStartOfFirstSpline = new YoGraphicPosition("Start of First DCM Spline", dcmPositionAtStartOfFirstSpline, graphicSize, YoAppearance.Green(), YoGraphicPosition.GraphicType.SOLID_BALL);
      YoGraphicPosition dcmEndOfFirstSpline = new YoGraphicPosition("End of First DCM Spline", dcmPositionAtEndOfFirstSpline, graphicSize, YoAppearance.Green(), YoGraphicPosition.GraphicType.BALL);
      YoGraphicPosition dcmStartOfSecondSpline = new YoGraphicPosition("Start of Second DCM Spline", dcmPositionAtStartOfSecondSpline, graphicSize, YoAppearance.Red(), YoGraphicPosition.GraphicType.SOLID_BALL);
      YoGraphicPosition dcmEndOfSecondSpline = new YoGraphicPosition("End of Second DCM Spline", dcmPositionAtEndOfSecondSpline, graphicSize, YoAppearance.Red(), YoGraphicPosition.GraphicType.BALL);
      YoGraphicPosition dcmEndOfState = new YoGraphicPosition("End of state DMC", dcmAtEndOfState, graphicSize, YoAppearance.Black(), YoGraphicPosition.GraphicType.SOLID_BALL);

      artifactList.setVisible(VISUALIZE);
      yoGraphicsList.setVisible(VISUALIZE);

      String name = "dcmPlanner";
      yoGraphicsListRegistry.registerYoGraphic(name, perfectCMPPositionViz);
      yoGraphicsListRegistry.registerArtifact(name, perfectCMPPositionViz.createArtifact());
      yoGraphicsListRegistry.registerArtifact(name, dcmStartOfFirstSpline.createArtifact());
      yoGraphicsListRegistry.registerArtifact(name, dcmEndOfFirstSpline.createArtifact());
      yoGraphicsListRegistry.registerArtifact(name, dcmStartOfSecondSpline.createArtifact());
      yoGraphicsListRegistry.registerArtifact(name, dcmEndOfSecondSpline.createArtifact());
      yoGraphicsListRegistry.registerArtifact(name, dcmEndOfState.createArtifact());

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   public void clearStepSequence()
   {
      stepSequence.clear();
      numberOfStepsInPlanner.set(0);
   }

   public void addStepToSequence(QuadrupedTimedStep step)
   {
      stepSequence.add(step);
      numberOfStepsInPlanner.increment();
   }

   public void initializeForStanding()
   {
      isStanding.set(true);
      timedContactSequence.clear();
      piecewiseConstantCopTrajectory.resetVariables();
      dcmTrajectory.resetVariables();
   }

   public void initializeForStepping(QuadrantDependentList<YoEnum<ContactState>> currentContactStates, FramePoint3DReadOnly currentDCMPosition,
                                     FrameVector3DReadOnly currentDCMVelocity)
   {
      isStanding.set(false);
      isInitialTransfer.set(true);

      double currentTime = controllerTime.getDoubleValue();
      boolean isCurrentPlanValid = stepSequence.get(numberOfStepsInPlanner.getIntegerValue() - 1).getTimeInterval().getEndTime() > currentTime;

      if (isCurrentPlanValid)
      {
         // compute dcm trajectory
         computeDcmTrajectory(currentContactStates);

         dcmPositionAtStartOfState.setMatchingFrame(currentDCMPosition);
         dcmVelocityAtStartOfState.setMatchingFrame(currentDCMVelocity);
         timeAtStartOfState.set(controllerTime.getDoubleValue());
         computeInitialTransitionTrajectory();
      }
   }

   public void beganStep()
   {
      onStateChange();
   }

   public void completedStep()
   {
      onStateChange();
   }

   public void setHoldCurrentDesiredPosition(boolean holdPosition)
   {
      this.holdPosition.set(holdPosition);

      if (holdPosition)
      {
         dcmPositionToHold.set(desiredDCMPosition);
      }
   }

   private void onStateChange()
   {
      isInitialTransfer.set(false);

      dcmPositionAtStartOfState.setMatchingFrame(desiredDCMPosition);
      dcmVelocityAtStartOfState.setMatchingFrame(desiredDCMVelocity);
      timeAtStartOfState.set(controllerTime.getDoubleValue());

      setFirstSplineStartFromCurrentState();
   }

   private void computeDcmTrajectory(QuadrantDependentList<YoEnum<ContactState>> currentContactStates)
   {
      // compute piecewise constant center of pressure plan
      timedContactSequence.update(stepSequence, soleFrames, currentContactStates, timeAtStartOfState.getDoubleValue());
      piecewiseConstantCopTrajectory.initializeTrajectory(timeAtStartOfState.getDoubleValue(), timedContactSequence, stepSequence);

      // compute dcm trajectory with final boundary constraint
      int numberOfIntervals = piecewiseConstantCopTrajectory.getNumberOfIntervals();
      finalDCMPosition.setMatchingFrame(piecewiseConstantCopTrajectory.getCopPositionAtStartOfInterval(numberOfIntervals - 1));
      finalDCMPosition.addZ(comHeight.getDoubleValue());

      dcmTrajectory.initializeTrajectory(numberOfIntervals, piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(),
                                         piecewiseConstantCopTrajectory.getCopPositionsAtStartOfInterval(),
                                         piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(numberOfIntervals - 1), finalDCMPosition);
   }

   private void computeInitialTransitionTrajectory()
   {
      double splineSplitFraction = MathTools.clamp(this.splineSplitFraction.getValue(), 0.0, 1.0);

      double firstSequenceDuration = piecewiseConstantCopTrajectory.getTimeAtEndOfInterval(0) - timeAtStartOfState.getValue();
      double secondSequenceDuration = piecewiseConstantCopTrajectory.getIntervalDuration(1);

      double firstSplineDurationOnEntryCMP = initialTransitionDuration.getValue();
      double secondSplineDurationOnEntryCMP = splineSplitFraction * splineSegmentDuration.getValue();
      double secondSplineDurationSpentOnExitCMP = Math.min((1.0 - splineSplitFraction) * splineSegmentDuration.getValue(), secondSequenceDuration);

      if (firstSequenceDuration < firstSplineDurationOnEntryCMP + splineSplitFraction * minimumSplineDuration.getValue())
      {// the first sequence isn't big enough to fit the initial durations on it, so wrap the first spline around it.
         firstSplineEndTime.set(piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(1) + secondSplineDurationSpentOnExitCMP);
         secondSplineStartTime.set(Double.NaN);
         secondSplineEndTime.set(Double.NaN);

         updateSplines();

         dcmFirstSpline.compute(piecewiseConstantCopTrajectory.getTimeAtEndOfInterval(0));
         dcmAtEndOfState.setMatchingFrame(dcmFirstSpline.getFramePosition());
         return;
      }
      else if (firstSequenceDuration < firstSplineDurationOnEntryCMP + secondSplineDurationOnEntryCMP)
      { // we know the first duration can fit both, and we have to maintain the initial transfer duration, so don't modify it any
         secondSplineDurationOnEntryCMP = firstSequenceDuration - firstSplineDurationOnEntryCMP;
      }

      firstSplineEndTime.set(timeAtStartOfState.getDoubleValue() + firstSplineDurationOnEntryCMP);
      secondSplineStartTime.set(piecewiseConstantCopTrajectory.getTimeAtEndOfInterval(0) - secondSplineDurationOnEntryCMP);
      secondSplineEndTime.set(piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(1) + secondSplineDurationSpentOnExitCMP);
      updateSplines();

      dcmSecondSpline.compute(piecewiseConstantCopTrajectory.getTimeAtEndOfInterval(0));
      dcmAtEndOfState.setMatchingFrame(dcmSecondSpline.getFramePosition());
   }

   private void computeTransitionTrajectory()
   {
      double splineSplitFraction = MathTools.clamp(this.splineSplitFraction.getValue(), 0.0, 1.0);

      double firstSequenceDuration = piecewiseConstantCopTrajectory.getTimeAtEndOfInterval(0) - timeAtStartOfState.getValue();
      double secondSequenceDuration = piecewiseConstantCopTrajectory.getIntervalDuration(1);

      double firstSplineDurationOnEntryCMP = (1.0 - splineSplitFraction) * splineSegmentDuration.getValue();
      double secondSplineDurationOnEntryCMP = splineSplitFraction * splineSegmentDuration.getValue();
      double secondSplineDurationSpentOnExitCMP = Math.min((1.0 - splineSplitFraction) * splineSegmentDuration.getValue(), secondSequenceDuration);

      if (firstSequenceDuration < minimumSplineDuration.getValue())
      { // the first duration isn't big enough to have the minimum spline durations on it, so wrap the first spline around it.
         firstSplineEndTime.set(piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(1) + secondSplineDurationSpentOnExitCMP);
         secondSplineStartTime.set(Double.NaN);
         secondSplineEndTime.set(Double.NaN);

         updateSplines();

         dcmFirstSpline.compute(piecewiseConstantCopTrajectory.getTimeAtEndOfInterval(0));
         dcmAtEndOfState.setMatchingFrame(dcmFirstSpline.getFramePosition());
         return;
      }
      else if (firstSequenceDuration < firstSplineDurationOnEntryCMP + secondSplineDurationOnEntryCMP)
      { // we know the first duration is big enough to have the minimum spline durations, but still too short for the desired values
         firstSplineDurationOnEntryCMP = Math.max(firstSequenceDuration - secondSplineDurationOnEntryCMP, minimumSplineDuration.getValue());
         secondSplineDurationOnEntryCMP = Math.max(firstSequenceDuration - firstSplineDurationOnEntryCMP, splineSplitFraction * minimumSplineDuration.getValue());
      }

      firstSplineEndTime.set(timeAtStartOfState.getDoubleValue() + firstSplineDurationOnEntryCMP);
      secondSplineStartTime.set(piecewiseConstantCopTrajectory.getTimeAtEndOfInterval(0) - secondSplineDurationOnEntryCMP);
      secondSplineEndTime.set(piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(1) + secondSplineDurationSpentOnExitCMP);

      updateSplines();

      dcmSecondSpline.compute(piecewiseConstantCopTrajectory.getTimeAtEndOfInterval(0));
      dcmAtEndOfState.setMatchingFrame(dcmSecondSpline.getFramePosition());
   }

   private void setFirstSplineStartFromCurrentState()
   {
      firstSplineStartTime.set(timeAtStartOfState.getDoubleValue());
      dcmPositionAtStartOfFirstSpline.set(dcmPositionAtStartOfState);
      dcmVelocityAtStartOfFirstSpline.set(dcmVelocityAtStartOfState);
   }

   private void updateSplines()
   {
      dcmTrajectory.computeTrajectory(firstSplineEndTime.getDoubleValue());
      dcmTrajectory.getPosition(dcmPositionAtEndOfFirstSpline);
      dcmTrajectory.getVelocity(dcmVelocityAtEndOfFirstSpline);

      dcmTrajectory.computeTrajectory(secondSplineStartTime.getDoubleValue());
      dcmTrajectory.getPosition(dcmPositionAtStartOfSecondSpline);
      dcmTrajectory.getVelocity(dcmVelocityAtStartOfSecondSpline);

      dcmTrajectory.computeTrajectory(secondSplineEndTime.getDoubleValue());
      dcmTrajectory.getPosition(dcmPositionAtEndOfSecondSpline);
      dcmTrajectory.getVelocity(dcmVelocityAtEndOfSecondSpline);

      splineInitialPosition.setMatchingFrame(dcmPositionAtStartOfFirstSpline);
      splineInitialVelocity.setMatchingFrame(dcmVelocityAtStartOfFirstSpline);
      splineFinalPosition.setMatchingFrame(dcmPositionAtEndOfFirstSpline);
      splineFinalVelocity.setMatchingFrame(dcmVelocityAtEndOfFirstSpline);
      dcmFirstSpline.setCubic(firstSplineStartTime.getDoubleValue(), firstSplineEndTime.getDoubleValue(), splineInitialPosition, splineInitialVelocity,
                              splineFinalPosition, splineFinalVelocity);

      splineInitialPosition.setMatchingFrame(dcmPositionAtStartOfSecondSpline);
      splineInitialVelocity.setMatchingFrame(dcmVelocityAtStartOfSecondSpline);
      splineFinalPosition.setMatchingFrame(dcmPositionAtEndOfSecondSpline);
      splineFinalVelocity.setMatchingFrame(dcmVelocityAtEndOfSecondSpline);
      dcmSecondSpline.setCubic(secondSplineStartTime.getDoubleValue(), secondSplineEndTime.getDoubleValue(), splineInitialPosition, splineInitialVelocity,
                               splineFinalPosition, splineFinalVelocity);
   }


   public void computeDcmSetpoints(QuadrantDependentList<YoEnum<ContactState>> currentContactStates, FixedFramePoint3DBasics desiredDCMPositionToPack,
                                   FixedFrameVector3DBasics desiredDCMVelocityToPack)
   {
      timeInState.set(controllerTime.getDoubleValue() - timeAtStartOfState.getValue());

      if (isStanding.getBooleanValue())
      {
         isInFirstSpline.set(false);
         isInSecondSpline.set(false);

         // update desired dcm position
         tempPoint.setToZero(supportFrame);
         tempPoint.addZ(comHeight.getDoubleValue());
         tempVector.setToZero(supportFrame);
         desiredDCMPosition.setMatchingFrame(tempPoint);
         desiredDCMVelocity.setMatchingFrame(tempVector);
      }
      else if (holdPosition.getBooleanValue())
      {
         desiredDCMPosition.setMatchingFrame(dcmPositionToHold);
         desiredDCMVelocity.setToZero();
      }
      else
      {
         computeDcmTrajectory(currentContactStates);

         if (isInitialTransfer.getBooleanValue())
            computeInitialTransitionTrajectory();
         else
            computeTransitionTrajectory();

         double currentTime = controllerTime.getDoubleValue();
         if (dcmFirstSpline.timeIntervalContains(currentTime))
         {
            isInFirstSpline.set(true);
            isInSecondSpline.set(false);
            dcmFirstSpline.compute(currentTime);
            desiredDCMPosition.setMatchingFrame(dcmFirstSpline.getFramePosition());
            desiredDCMVelocity.setMatchingFrame(dcmFirstSpline.getFrameVelocity());
         }
         else if (dcmSecondSpline.timeIntervalContains(currentTime))
         {
            isInFirstSpline.set(false);
            isInSecondSpline.set(true);
            dcmSecondSpline.compute(currentTime);
            desiredDCMPosition.setMatchingFrame(dcmSecondSpline.getFramePosition());
            desiredDCMVelocity.setMatchingFrame(dcmSecondSpline.getFrameVelocity());
         }
         else
         {
            isInFirstSpline.set(false);
            isInSecondSpline.set(false);
            dcmTrajectory.computeTrajectory(currentTime);
            dcmTrajectory.getPosition(desiredDCMPosition);
            dcmTrajectory.getVelocity(desiredDCMVelocity);
         }
      }

      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredDCMPosition, desiredDCMVelocity, omega.getDoubleValue(), desiredVRPPosition);
      desiredECMPPosition.set(desiredVRPPosition);
      desiredECMPPosition.subZ(comHeight.getDoubleValue());

      desiredDCMPositionToPack.setMatchingFrame(desiredDCMPosition);
      desiredDCMVelocityToPack.setMatchingFrame(desiredDCMVelocity);

      if (desiredDCMPositionToPack.containsNaN())
         throw new RuntimeException("Invalid setpoint.");
   }



   public void getFinalDCMPosition(FixedFramePoint3DBasics finalDesiredDCMToPack)
   {
      finalDesiredDCMToPack.setMatchingFrame(dcmAtEndOfState);
   }

   @Override
   public void getDesiredECMPPosition(FramePoint3DBasics desiredECMPPositionToPack)
   {
      desiredECMPPositionToPack.setIncludingFrame(desiredECMPPosition);
   }
}
