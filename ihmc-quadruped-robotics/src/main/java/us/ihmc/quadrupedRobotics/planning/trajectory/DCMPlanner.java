package us.ihmc.quadrupedRobotics.planning.trajectory;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

import java.util.List;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedContactSequence;
import us.ihmc.robotics.math.trajectories.YoFrameTrajectory3D;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class DCMPlanner implements DCMPlannerInterface
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private static final boolean VISUALIZE = true;
   private static final double POINT_SIZE = 0.005;

   private static final int STEP_SEQUENCE_CAPACITY = 50;

   private final QuadrupedPiecewiseConstantCopTrajectory piecewiseConstantCopTrajectory;
   private final PiecewiseReverseDcmTrajectory dcmTrajectory;
   private final YoFrameTrajectory3D dcmTransitionTrajectory;

   private final DoubleParameter initialTransitionDurationParameter = new DoubleParameter("initialTransitionDuration", registry, 0.5);

   private final QuadrupedTimedContactSequence timedContactSequence = new QuadrupedTimedContactSequence(2 * STEP_SEQUENCE_CAPACITY);

   private final QuadrantDependentList<MovingReferenceFrame> soleFrames;

   private final YoDouble omega = new YoDouble("omegaForPlanning", registry);
   private final YoDouble comHeight = new YoDouble("comHeightForPlanning", registry);
   private final YoFramePoint3D perfectVRPPosition = new YoFramePoint3D("perfectVRPPosition", worldFrame, registry);
   private final YoFramePoint3D perfectCMPPosition = new YoFramePoint3D("perfectCMPPosition", worldFrame, registry);

   private final YoBoolean isStanding = new YoBoolean("isStanding", registry);

   private final ReferenceFrame supportFrame;
   private final YoFramePoint3D dcmPositionAtStartOfState = new YoFramePoint3D("dcmPositionAtStartOfState", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D dcmVelocityAtStartOfState = new YoFrameVector3D("dcmVelocityAtStartOfState", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D dcmPositionAtEndOfTransition = new YoFramePoint3D("dcmPositionAtEndOfTransition", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D dcmVelocityAtEndOfTransition = new YoFrameVector3D("dcmVelocityAtEndOfTransition", ReferenceFrame.getWorldFrame(), registry);
   private final YoDouble timeAtStartOfState = new YoDouble("timeAtStartOfState", registry);
   private final FramePoint3D initialTransitionDCMPosition = new FramePoint3D();
   private final FrameVector3D initialTransitionDCMVelocity = new FrameVector3D();
   private final FramePoint3D finalTransitionDCMPosition = new FramePoint3D();
   private final FrameVector3D finalTransitionDCMVelocity = new FrameVector3D();
   private final FramePoint3D finalDCM = new FramePoint3D();

   private final FramePoint3D tempPoint = new FramePoint3D();

   private final boolean debug;

   public DCMPlanner(DCMPlannerParameters dcmPlannerParameters, double nominalHeight, double gravity, ReferenceFrame supportFrame,
                     QuadrantDependentList<MovingReferenceFrame> soleFrames, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(dcmPlannerParameters, nominalHeight, gravity, supportFrame, soleFrames, parentRegistry, yoGraphicsListRegistry, false);
   }

   public DCMPlanner(DCMPlannerParameters dcmPlannerParameters, double nominalHeight, double gravity, ReferenceFrame supportFrame,
                     QuadrantDependentList<MovingReferenceFrame> soleFrames, YoRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry,
                     boolean debug)

   {
      this.supportFrame = supportFrame;
      this.soleFrames = soleFrames;
      this.debug = debug;
      this.dcmTransitionTrajectory = new YoFrameTrajectory3D("dcmTransitionTrajectory", 4, supportFrame, registry);

      comHeight.addListener(v ->
            omega.set(Math.sqrt(gravity / comHeight.getDoubleValue())));
      comHeight.set(nominalHeight);


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

      YoGraphicPosition perfectCMPPositionViz = new YoGraphicPosition("Perfect CMP Position", perfectCMPPosition, 0.002, YoAppearance.BlueViolet());

      artifactList.setVisible(VISUALIZE);
      yoGraphicsList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerYoGraphic("dcmPlanner", perfectCMPPositionViz);
      yoGraphicsListRegistry.registerArtifact("dcmPlanner", perfectCMPPositionViz.createArtifact());

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   @Override
   public void initialize()
   {
   }

   public void setHoldCurrentDesiredPosition(boolean holdPosition)
   {
   }

   public void setNominalCoMHeight(double comHeight)
   {
      this.comHeight.set(comHeight);
   }

   /**
    * The current position {@param currentDCMPosition} must be the DCM position and the current velocity {@param currentDCMVelocity} must be the  DCM Velocity
    */
   @Override
   public void setInitialState(double initialTime, FramePoint3DReadOnly currentDCMPosition, FrameVector3DReadOnly currentDCMVelocity,
                               FramePoint3DReadOnly copPosition)
   {
      timeAtStartOfState.set(initialTime);
      dcmPositionAtStartOfState.set(currentDCMPosition);
      dcmVelocityAtStartOfState.set(currentDCMVelocity);
   }

   private void computeDcmTrajectory(double currentTime, List<RobotQuadrant> currentFeetInContact, List<? extends QuadrupedTimedStep> stepSequence)
   {
      // compute piecewise constant center of pressure plan
      timedContactSequence.update(stepSequence, soleFrames, currentFeetInContact, currentTime);
      piecewiseConstantCopTrajectory.initializeTrajectory(currentTime, timedContactSequence, stepSequence);

      // compute dcm trajectory with final boundary constraint
      int numberOfIntervals = piecewiseConstantCopTrajectory.getNumberOfIntervals();
      tempPoint.setIncludingFrame(piecewiseConstantCopTrajectory.getCopPositionAtStartOfInterval(numberOfIntervals - 1));
      tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
      tempPoint.add(0, 0, comHeight.getDoubleValue());

      dcmTrajectory.initializeTrajectory(numberOfIntervals, piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(),
                                         piecewiseConstantCopTrajectory.getCopPositionsAtStartOfInterval(),
                                         piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(numberOfIntervals - 1), tempPoint);
   }

   private void computeTransitionTrajectory()
   {
      double transitionStartTime = timeAtStartOfState.getDoubleValue();
      double transitionEndTime = Math.min(piecewiseConstantCopTrajectory.getTimeAtStartOfInterval(1), transitionStartTime + initialTransitionDurationParameter.getValue());

      dcmTrajectory.computeTrajectory(transitionEndTime);
      dcmTrajectory.getPosition(finalTransitionDCMPosition);
      dcmTrajectory.getVelocity(finalTransitionDCMVelocity);

      initialTransitionDCMPosition.setIncludingFrame(dcmPositionAtStartOfState);
      initialTransitionDCMVelocity.setIncludingFrame(dcmVelocityAtStartOfState);
      initialTransitionDCMPosition.changeFrame(dcmTransitionTrajectory.getReferenceFrame());
      initialTransitionDCMVelocity.changeFrame(dcmTransitionTrajectory.getReferenceFrame());
      finalTransitionDCMPosition.changeFrame(dcmTransitionTrajectory.getReferenceFrame());
      finalTransitionDCMVelocity.changeFrame(dcmTransitionTrajectory.getReferenceFrame());

      dcmTransitionTrajectory
            .setCubic(transitionStartTime, transitionEndTime, initialTransitionDCMPosition, initialTransitionDCMVelocity, finalTransitionDCMPosition,
                      finalTransitionDCMVelocity);

      dcmPositionAtEndOfTransition.setMatchingFrame(finalTransitionDCMPosition);
      dcmVelocityAtEndOfTransition.setMatchingFrame(finalTransitionDCMVelocity);

      if (debug)
         runTransitionDebugChecks(transitionStartTime, transitionEndTime);
   }

   private void runTransitionDebugChecks(double transitionStartTime, double transitionEndTime)
   {
      if (finalTransitionDCMPosition.containsNaN())
         throw new IllegalArgumentException("Final DCM position at end of transition contains NaN.");
      if (finalTransitionDCMVelocity.containsNaN())
         throw new IllegalArgumentException("Final DCM velocity at end of transition contains NaN.");
      if (dcmPositionAtStartOfState.containsNaN())
         throw new IllegalArgumentException("DCM position at start of state contains NaN.");
      if (dcmVelocityAtStartOfState.containsNaN())
         throw new IllegalArgumentException("DCM velocity at start of state contains NaN.");
      if (!Double.isFinite(transitionStartTime))
         throw new IllegalArgumentException("Transition start time is not valid.");
      if (!Double.isFinite(transitionEndTime))
         throw new IllegalArgumentException("Transition end time is not valid.");
      if (transitionStartTime > transitionEndTime)
         throw new IllegalArgumentException("Transition start time " + transitionStartTime + " is after the transition end time " + transitionEndTime + ".");
      if (!dcmTransitionTrajectory.isValidTrajectory())
         throw new IllegalArgumentException("Transition trajectory is invalid.");
   }

   private final FramePoint3D desiredDCMPosition = new FramePoint3D();
   private final FrameVector3D desiredDCMVelocity = new FrameVector3D();

   @Override
   public void computeSetpoints(double currentTime, List<? extends QuadrupedTimedStep> stepSequence, List<RobotQuadrant> currentFeetInContact)
   {
      isStanding.set(stepSequence.isEmpty());

      if (isStanding.getBooleanValue())
      {
         // update desired dcm position
         desiredDCMPosition.setToZero(supportFrame);
         desiredDCMPosition.setZ(comHeight.getDoubleValue());
         desiredDCMVelocity.setToZero(supportFrame);
      }
      else
      {
         computeDcmTrajectory(currentTime, currentFeetInContact, stepSequence);

         dcmTrajectory.computeTrajectory(currentTime);
         if (currentTime <= dcmTransitionTrajectory.getFinalTime())
         {
            computeTransitionTrajectory();

            dcmTransitionTrajectory.compute(currentTime);
            dcmTransitionTrajectory.getFramePosition(desiredDCMPosition);
            dcmTransitionTrajectory.getFrameVelocity(desiredDCMVelocity);
         }
         else
         {
            dcmTrajectory.getPosition(desiredDCMPosition);
            dcmTrajectory.getVelocity(desiredDCMVelocity);
         }

         dcmTrajectory.getPositionAtEndOfSwing(finalDCM);
      }

      if (debug)
         runOutputDebugChecks();

      CapturePointTools.computeCentroidalMomentumPivot(desiredDCMPosition, desiredDCMVelocity, omega.getDoubleValue(), perfectVRPPosition);
      perfectCMPPosition.set(perfectVRPPosition);
      perfectCMPPosition.subZ(comHeight.getDoubleValue());
   }


   private void runOutputDebugChecks()
   {
      if (desiredDCMPosition.containsNaN())
         throw new IllegalArgumentException("Desired DCM Position contains NaN.");
      if (desiredDCMVelocity.containsNaN())
         throw new IllegalArgumentException("Desired DCM Velocity contains NaN.");
   }

   public void getDCMAtEndOfTransition(FixedFramePoint3DBasics finalDesiredDCMToPack)
   {
      finalDesiredDCMToPack.setMatchingFrame(finalTransitionDCMPosition);
   }

   public FramePoint3DReadOnly getFinalDCMPosition()
   {
      return finalDCM;
   }

   public double getFinalTime()
   {
      return dcmTransitionTrajectory.getFinalTime();
   }

   @Override
   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return desiredDCMPosition;
   }

   @Override
   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return desiredDCMVelocity;
   }

   @Override
   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return null;
   }

   @Override
   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return null;
   }

   @Override
   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return null;
   }

   @Override
   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return perfectVRPPosition;
   }

   @Override
   public FramePoint3DReadOnly getDesiredECMPPosition()
   {
      return perfectCMPPosition;
   }
}
