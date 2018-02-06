package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class AbstractICPPlanner implements ICPPlannerInterface
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   protected final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final String namePrefix = "icpPlanner";

   protected final YoInteger numberFootstepsToConsider = new YoInteger(namePrefix + "NumberFootstepsToConsider", registry);
   protected final YoBoolean isStanding = new YoBoolean(namePrefix + "IsStanding", registry);
   protected final YoBoolean isInitialTransfer = new YoBoolean(namePrefix + "IsInitialTransfer", registry);
   protected final YoBoolean isDoubleSupport = new YoBoolean(namePrefix + "IsDoubleSupport", registry);

   protected final ExecutionTimer timer = new ExecutionTimer(namePrefix + "Timer", registry);

   /////////////////////////////// Start Planner Output ///////////////////////////////

   /** Desired position for the Centroidal Momentum Pivot (CMP) */
   protected final YoFramePoint desiredCMPPosition = new YoFramePoint(namePrefix + "DesiredCMPPosition", worldFrame, registry);
   /** Desired velocity for the Centroidal Momentum Pivot (CMP) */
   protected final YoFrameVector desiredCMPVelocity = new YoFrameVector(namePrefix + "DesiredCMPVelocity", worldFrame, registry);
   /** Desired position for the Instantaneous Capture Point (ICP) */
   protected final YoFramePoint desiredICPPosition = new YoFramePoint(namePrefix + "DesiredICPPosition", worldFrame, registry);
   /** Desired velocity for the Instantaneous Capture Point (ICP) */
   protected final YoFrameVector desiredICPVelocity = new YoFrameVector(namePrefix + "DesiredICPVelocity", worldFrame, registry);
   /** Desired acceleration for the Instantaneous Capture Point (ICP) */
   protected final YoFrameVector desiredICPAcceleration = new YoFrameVector(namePrefix + "DesiredICPAcceleration", worldFrame, registry);
   /** Desired position for the Center of Mass (CoM)*/
   protected final YoFramePoint desiredCoMPosition = new YoFramePoint(namePrefix + "DesiredCoMPosition", worldFrame, registry);

   //////////////////////////////// End Planner Output ////////////////////////////////

   protected final YoDouble omega0 = new YoDouble(namePrefix + "Omega0", registry);
   /**
    * Repartition of the swing duration around the exit corner point:
    * <ul>
    * <li>{@code alpha * swingDuration} is spent with the ICP located before the exit corner point.
    * <li>{@code (1.0 - alpha) * swingDuration} is spent with the ICP located after the exit corner
    * point.
    * </ul>
    * <p>
    * This variable is only used when using two constant CMPs per support:
    * {@code useTwoConstantCMPsPerSupport == true}.
    * </p>
    */
   protected final YoDouble defaultSwingDurationAlpha = new YoDouble(namePrefix + "DefaultSwingDurationAlpha",
                                                                   "Repartition of the swing duration around the exit corner point.", registry);
   protected final ArrayList<YoDouble> swingDurationAlphas = new ArrayList<>();

   /**
    * Repartition of the transfer duration around the entry corner point:
    * <ul>
    * <li>{@code alpha * transferDuration} is spent with the ICP located before the entry corner
    * point.
    * <li>{@code (1.0 - alpha) * transferDuration} is spent with the ICP located after the entry
    * corner point.
    * </ul>
    */
   protected final YoDouble defaultTransferDurationAlpha = new YoDouble(namePrefix + "DefaultTransferDurationAlpha",
                                                                      "Repartition of the transfer duration around the entry corner point.", registry);
   protected final ArrayList<YoDouble> transferDurationAlphas = new ArrayList<>();

   protected final YoDouble finalTransferDurationAlpha = new YoDouble(namePrefix + "FinalTransferDurationAlpha", registry);


   /** Time at which the current state was initialized. */
   protected final YoDouble initialTime = new YoDouble(namePrefix + "CurrentStateInitialTime", registry);
   /** Time spent in the current state. */
   protected final YoDouble timeInCurrentState = new YoDouble(namePrefix + "TimeInCurrentState", registry);
   /** Time remaining before the end of the current state. */
   protected final YoDouble timeInCurrentStateRemaining = new YoDouble(namePrefix + "RemainingTime", registry);

   /**
    * Duration parameter used to linearly decrease the desired ICP velocity once the current state
    * is done.
    * <p>
    * This reduction in desired ICP velocity is particularly useful to reduce the ICP tracking error
    * when the robot is getting stuck at the end of transfer.
    * </p>
    */
   private final YoDouble velocityDecayDurationWhenDone = new YoDouble(namePrefix + "VelocityDecayDurationWhenDone", registry);
   /**
    * Output of the linear reduction being applied on the desired ICP velocity when the current
    * state is done.
    * <p>
    * This reduction in desired ICP velocity is particularly useful to reduce the ICP tracking error
    * when the robot is getting stuck at the end of transfer.
    true* </p>
    */
   private final YoDouble velocityReductionFactor = new YoDouble(namePrefix + "VelocityReductionFactor", registry);

   protected final YoFramePointInMultipleFrames singleSupportInitialICP;
   protected final YoFrameVector singleSupportInitialICPVelocity = new YoFrameVector(namePrefix + "SingleSupportInitialICPVelocity", worldFrame, registry);

   protected final YoFramePointInMultipleFrames singleSupportFinalICP;
   protected final YoFrameVector singleSupportFinalICPVelocity = new YoFrameVector(namePrefix + "SingleSupportFinalICPVelocity", worldFrame, registry);

   protected final YoBoolean requestedHoldPosition = new YoBoolean(namePrefix + "RequestedHoldPosition", registry);
   protected final YoBoolean isHoldingPosition = new YoBoolean(namePrefix + "IsHoldingPosition", registry);
   protected final YoFramePoint icpPositionToHold = new YoFramePoint(namePrefix + "CapturePointPositionToHold", worldFrame, registry);

   protected final YoEnum<RobotSide> transferToSide = new YoEnum<>(namePrefix + "TransferToSide", registry, RobotSide.class, true);
   protected final YoEnum<RobotSide> supportSide = new YoEnum<>(namePrefix + "SupportSide", registry, RobotSide.class, true);

   protected final List<YoDouble> swingDurations = new ArrayList<>();
   protected final List<YoDouble> touchdownDurations = new ArrayList<>();
   protected final List<YoDouble> transferDurations = new ArrayList<>();
   protected final YoDouble defaultFinalTransferDuration = new YoDouble(namePrefix + "DefaultFinalTransferDuration", registry);
   protected final YoDouble finalTransferDuration = new YoDouble(namePrefix + "FinalTransferDuration", registry);

   protected final ReferenceFrame midFeetZUpFrame;
   protected final SideDependentList<ReferenceFrame> soleZUpFrames;

   /**
    * Creates an ICP planner. Refer to the class documentation: {@link ContinuousCMPBasedICPPlanner}.
    * 
    * @param bipedSupportPolygons it is used to get reference frames relevant for walking such as
    *           the sole frames. It is also used in
    *           {@link ReferenceCentroidalMomentumPivotLocationsCalculator} to adapt the ICP plan to
    *           available support polygon. The reference to this parameter is saved internally and
    *           it will be accessed to access up-to-date information.
    */
   public AbstractICPPlanner(BipedSupportPolygons bipedSupportPolygons, int numberOfFootstepsToConsider)
   {
      isStanding.set(true);

      finalTransferDuration.setToNaN();

      icpPositionToHold.setToNaN();
      isHoldingPosition.set(false);

      // Initialize omega0 to NaN to force the user to explicitly set it.
      omega0.set(Double.NaN);

      midFeetZUpFrame = bipedSupportPolygons.getMidFeetZUpFrame();
      soleZUpFrames = bipedSupportPolygons.getSoleZUpFrames();

      ReferenceFrame[] framesToRegister = new ReferenceFrame[] {worldFrame, midFeetZUpFrame, soleZUpFrames.get(RobotSide.LEFT),
            soleZUpFrames.get(RobotSide.RIGHT)};
      singleSupportInitialICP = new YoFramePointInMultipleFrames(namePrefix + "SingleSupportInitialICP", registry, framesToRegister);
      singleSupportFinalICP = new YoFramePointInMultipleFrames(namePrefix + "SingleSupportFinalICP", registry, framesToRegister);


      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         YoDouble swingDuration = new YoDouble(namePrefix + "SwingDuration" + i, registry);
         swingDuration.setToNaN();
         swingDurations.add(swingDuration);
         YoDouble touchdownDuration = new YoDouble(namePrefix + "TouchdownDuration" + i, registry);
         touchdownDuration.setToNaN();
         touchdownDurations.add(touchdownDuration);
         YoDouble transferDuration = new YoDouble(namePrefix + "TransferDuration" + i, registry);
         transferDuration.setToNaN();
         transferDurations.add(transferDuration);

         YoDouble transferDurationAlpha = new YoDouble(namePrefix + "TransferDurationAlpha" + i,
                                                       "Repartition of the transfer duration around the entry corner point.", registry);
         transferDurationAlpha.setToNaN();
         transferDurationAlphas.add(transferDurationAlpha);
         YoDouble swingDurationAlpha = new YoDouble(namePrefix + "SwingDurationAlpha" + i,
                                                    "Repartition of the transfer duration around the entry corner point.", registry);
         swingDurationAlpha.setToNaN();
         swingDurationAlphas.add(swingDurationAlpha);
      }
      YoDouble transferDuration = new YoDouble(namePrefix + "TransferDuration" + numberOfFootstepsToConsider, registry);
      YoDouble transferDurationAlpha = new YoDouble(namePrefix + "TransferDurationAlpha" + numberOfFootstepsToConsider,
                                                    "Repartition of the transfer duration around the entry corner point.", registry);
      transferDuration.setToNaN();
      transferDurationAlpha.setToNaN();
      transferDurations.add(transferDuration);
      transferDurationAlphas.add(transferDurationAlpha);
   }

   public void initializeParameters(ICPPlannerParameters parameters)
   {
      defaultTransferDurationAlpha.set(parameters.getTransferSplitFraction());
      defaultSwingDurationAlpha.set(parameters.getSwingSplitFraction());
      finalTransferDurationAlpha.set(parameters.getTransferSplitFraction());

      velocityDecayDurationWhenDone.set(parameters.getVelocityDecayDurationWhenDone());
      velocityReductionFactor.set(Double.NaN);
   }

   /** {@inheritDoc} */
   @Override
   public void setSupportLeg(RobotSide robotSide)
   {
      supportSide.set(robotSide);
   }

   /** {@inheritDoc} */
   @Override
   public void setTransferToSide(RobotSide robotSide)
   {
      transferToSide.set(robotSide);
   }

   /** {@inheritDoc} */
   @Override
   public void setTransferFromSide(RobotSide robotSide)
   {
      if (robotSide != null)
         transferToSide.set(robotSide.getOppositeSide());
   }

   /** {@inheritDoc} */
   @Override
   public void holdCurrentICP(FramePoint3D icpPositionToHold)
   {
      this.icpPositionToHold.set(icpPositionToHold);
      requestedHoldPosition.set(true);
   }

   /** {@inheritDoc} */
   @Override
   public void updateCurrentPlan()
   {
      if (isDoubleSupport.getBooleanValue())
      {
         if (isHoldingPosition.getBooleanValue())
            requestedHoldPosition.set(true);
         updateTransferPlan();
      }
      else
      {
         updateSingleSupportPlan();
      }
   }

   /** {@inheritDoc} */
   @Override
   public double estimateTimeRemainingForStateUnderDisturbance(FramePoint2D actualCapturePointPosition)
   {
      if (isDone())
         return 0.0;

      double deltaTimeToBeAccounted = estimateDeltaTimeBetweenDesiredICPAndActualICP(actualCapturePointPosition);

      if (Double.isNaN(deltaTimeToBeAccounted))
         return 0.0;

      double estimatedTimeRemaining = getTimeInCurrentStateRemaining() - deltaTimeToBeAccounted;
      estimatedTimeRemaining = MathTools.clamp(estimatedTimeRemaining, 0.0, Double.POSITIVE_INFINITY);

      return estimatedTimeRemaining;
   }

   protected void decayDesiredVelocityIfNeeded()
   {
      if (velocityDecayDurationWhenDone.isNaN() || isStanding.getBooleanValue())
      {
         velocityReductionFactor.set(Double.NaN);
         return;
      }

      double hasBeenDoneForDuration = -timeInCurrentStateRemaining.getDoubleValue();

      if (hasBeenDoneForDuration <= 0.0)
      {
         velocityReductionFactor.set(Double.NaN);
      }
      else
      {
         velocityReductionFactor.set(MathTools.clamp(1.0 - hasBeenDoneForDuration / velocityDecayDurationWhenDone.getDoubleValue(), 0.0, 1.0));
         desiredICPVelocity.scale(velocityReductionFactor.getDoubleValue());
      }
   }

   /** {@inheritDoc} */
   @Override
   public abstract void compute(double time);

   /** {@inheritDoc} */
   @Override
   public abstract void getFinalDesiredCapturePointPosition(FramePoint3D finalDesiredCapturePointPositionToPack);

   /** {@inheritDoc} */
   @Override
   public abstract void getFinalDesiredCapturePointPosition(YoFramePoint2d finalDesiredCapturePointPositionToPack);

   /** {@inheritDoc} */
   @Override
   public abstract void getFinalDesiredCenterOfMassPosition(FramePoint3D finalDesiredCenterOfMassPositionToPack);

   /** {@inheritDoc} */
   @Override
   public abstract void getNextExitCMP(FramePoint3D entryCMPToPack);

   /** {@inheritDoc} */
   @Override
   public abstract boolean isOnExitCMP();

   /** {@inheritDoc} */
   @Override
   public abstract int getNumberOfFootstepsToConsider();

   /** {@inheritDoc} */
   @Override
   public abstract int getNumberOfFootstepsRegistered();

   protected abstract void updateTransferPlan();
   protected abstract void updateSingleSupportPlan();

   private final FramePoint2D desiredICP2d = new FramePoint2D();
   private final FramePoint2D finalICP2d = new FramePoint2D();
   private final FrameLine2D desiredICPToFinalICPLine = new FrameLine2D();
   private final FrameLineSegment2D desiredICPToFinalICPLineSegment = new FrameLineSegment2D();
   private final FramePoint2D actualICP2d = new FramePoint2D();

   private double estimateDeltaTimeBetweenDesiredICPAndActualICP(FramePoint2D actualCapturePointPosition)
   {
      desiredICP2d.setIncludingFrame(desiredICPPosition);
      finalICP2d.setIncludingFrame(singleSupportFinalICP);

      if (desiredICP2d.distance(finalICP2d) < 1.0e-10)
         return Double.NaN;

      desiredICPToFinalICPLineSegment.set(desiredICP2d, finalICP2d);
      actualICP2d.setIncludingFrame(actualCapturePointPosition);
      double percentAlongLineSegmentICP = desiredICPToFinalICPLineSegment.percentageAlongLineSegment(actualICP2d);
      if (percentAlongLineSegmentICP < 0.0)
      {
         desiredICPToFinalICPLine.set(desiredICP2d, finalICP2d);
         desiredICPToFinalICPLine.orthogonalProjection(actualICP2d);
      }
      else
      {
         desiredICPToFinalICPLineSegment.orthogonalProjection(actualICP2d);
      }

      double actualDistanceDueToDisturbance = desiredCMPPosition.distanceXY(actualICP2d);
      double expectedDistanceAccordingToPlan = desiredCMPPosition.distanceXY(desiredICPPosition);

      double distanceRatio = actualDistanceDueToDisturbance / expectedDistanceAccordingToPlan;

      if (distanceRatio < 1.0e-3)
         return 0.0;
      else
         return Math.log(distanceRatio) / omega0.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointPosition(FramePoint3D desiredCapturePointPositionToPack)
   {
      desiredCapturePointPositionToPack.setIncludingFrame(desiredICPPosition);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointPosition(FramePoint2D desiredCapturePointPositionToPack)
   {
      desiredCapturePointPositionToPack.setIncludingFrame(desiredICPPosition);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointPosition(YoFramePoint desiredCapturePointPositionToPack)
   {
      desiredCapturePointPositionToPack.set(desiredICPPosition);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCenterOfMassPosition(FramePoint3D desiredCenterOfMassPositionToPack)
   {
      desiredCenterOfMassPositionToPack.setIncludingFrame(desiredCoMPosition);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCenterOfMassPosition(YoFramePoint desiredCenterOfMassPositionToPack)
   {
      desiredCenterOfMassPositionToPack.set(desiredCoMPosition);
   }


   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(FrameVector3D desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.setIncludingFrame(desiredICPVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(FrameVector2D desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.setIncludingFrame(desiredICPVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointVelocity(YoFrameVector desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.set(desiredICPVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCapturePointAcceleration(FrameVector3D desiredCapturePointAccelerationToPack)
   {
      desiredCapturePointAccelerationToPack.set(desiredICPAcceleration);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint3D desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCentroidalMomentumPivotPositionToPack.setIncludingFrame(desiredCMPPosition);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint2D desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCentroidalMomentumPivotPositionToPack.setIncludingFrame(desiredCMPPosition);
   }
   
   public void getDesiredCentroidalMomentumPivotPosition(YoFramePoint desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCentroidalMomentumPivotPositionToPack.set(desiredCMPPosition);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector3D desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCentroidalMomentumPivotVelocityToPack.setIncludingFrame(desiredCMPVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector2D desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCentroidalMomentumPivotVelocityToPack.setIncludingFrame(desiredCMPVelocity);
   }

   public void getDesiredCentroidalMomentumPivotVelocity(YoFrameVector desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCentroidalMomentumPivotVelocityToPack.set(desiredCMPVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public double getTimeInCurrentState()
   {
      return timeInCurrentState.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getTimeInCurrentStateRemaining()
   {
      return timeInCurrentStateRemaining.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getCurrentStateDuration()
   {
      if (isDoubleSupport.getBooleanValue())
         return getTransferDuration(0);
      else
         return getSwingDuration(0);
   }

   /** {@inheritDoc} */
   @Override
   public void setTransferDuration(int stepNumber, double duration)
   {
      transferDurations.get(stepNumber).set(duration);
   }

   /** {@inheritDoc} */
   @Override
   public void setSwingDuration(int stepNumber, double duration)
   {
      swingDurations.get(stepNumber).set(duration);
   }

   /** {@inheritDoc} */
   @Override
   public void setTouchdownDuration(int stepNumber, double duration)
   {
      touchdownDurations.get(stepNumber).set(duration);
   }

   /** {@inheritDoc} */
    @Override
   public double getTouchdownDuration(int stepNumber)
   {
       return touchdownDurations.get(stepNumber).getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getTransferDuration(int stepNumber)
   {
      return transferDurations.get(stepNumber).getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getSwingDuration(int stepNumber)
   {
      return swingDurations.get(stepNumber).getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public void setFinalTransferDuration(double duration)
   {
      if(duration < Epsilons.ONE_HUNDREDTH)
         return;
      defaultFinalTransferDuration.set(duration);
   }

   /** {@inheritDoc} */
   @Override
   public void setFinalTransferDurationAlpha(double durationAlpha)
   {
      finalTransferDurationAlpha.set(durationAlpha);
   }

   /** {@inheritDoc} */
   @Override
   public void setTransferDurationAlpha(int stepNumber, double transferDurationAlpha)
   {
      transferDurationAlphas.get(stepNumber).set(transferDurationAlpha);
   }

   /** {@inheritDoc} */
   @Override
   public void setSwingDurationAlpha(int stepNumber, double swingDurationAlpha)
   {
      swingDurationAlphas.get(stepNumber).set(swingDurationAlpha);
   }

   /** {@inheritDoc} */
   @Override
   public double getTransferDurationAlpha(int stepNumber)
   {
      return transferDurationAlphas.get(stepNumber).getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getSwingDurationAlpha(int stepNumber)
   {
      return swingDurationAlphas.get(stepNumber).getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public double getInitialTime()
   {
      return initialTime.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public void setOmega0(double omega0)
   {
      this.omega0.set(omega0);
   }

   /** {@inheritDoc} */
   @Override
   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   /** {@inheritDoc} */
   @Override
   public RobotSide getTransferToSide()
   {
      return transferToSide.getEnumValue();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isInDoubleSupport()
   {
      return isDoubleSupport.getBooleanValue();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isInStanding()
   {
      return isStanding.getBooleanValue();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isInInitialTransfer()
   {
      return isInitialTransfer.getBooleanValue();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isDone()
   {
      return timeInCurrentStateRemaining.getDoubleValue() <= 0.0;
   }


}
