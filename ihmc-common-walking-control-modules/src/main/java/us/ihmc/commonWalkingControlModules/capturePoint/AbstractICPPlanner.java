package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commons.Epsilons;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
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

   /** Desired position for the Center of Pressure (CoP) */
   protected final YoFramePoint desiredCoPPosition = new YoFramePoint(namePrefix + "DesiredCoPPosition", worldFrame, registry);
   /** Desired velocity for the Center of Pressure (CoP) */
   protected final YoFrameVector desiredCoPVelocity = new YoFrameVector(namePrefix + "DesiredCoPVelocity", worldFrame, registry);
   /** Desired Centroidal Angular Momentum (CAM) */
   protected final YoFrameVector desiredCentroidalAngularMomentum = new YoFrameVector(namePrefix + "DesiredCentroidalAngularMomentum", worldFrame, registry);
   /** Desired Centroidal Torque (CT) */
   protected final YoFrameVector desiredCentroidalTorque = new YoFrameVector(namePrefix + "DesiredCentroidalTorque", worldFrame, registry);
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
   /** Desired velocity for the Center of Mass (CoM) */
   protected final YoFrameVector desiredCoMVelocity = new YoFrameVector(namePrefix + "DesiredCoMVelocity", worldFrame, registry);
   /** Desired acceleration for the Center of Mass (CoM) */
   protected final YoFrameVector desiredCoMAcceleration = new YoFrameVector(namePrefix + "DesiredCoMAcceleration", worldFrame, registry);

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
    * @param contactableFeet it is used to get the set of default contact points for each foot.
    * @param icpPlannerParameters configuration class used to initialized the constant parameters of
    *           the ICP plan.
    * @param parentRegistry registry to which the ICP planner's registry is attached to.
    * @param yoGraphicsListRegistry registry to which the visualization for the planner should be
    *           added to.
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

   @Override
   /** {@inheritDoc} */
   public void setSupportLeg(RobotSide robotSide)
   {
      supportSide.set(robotSide);
   }

   @Override
   /** {@inheritDoc} */
   public void setTransferToSide(RobotSide robotSide)
   {
      transferToSide.set(robotSide);
   }

   @Override
   /** {@inheritDoc} */
   public void setTransferFromSide(RobotSide robotSide)
   {
      if (robotSide != null)
         transferToSide.set(robotSide.getOppositeSide());
   }

   @Override
   /** {@inheritDoc} */
   public void holdCurrentICP(FramePoint3D icpPositionToHold)
   {
      this.icpPositionToHold.set(icpPositionToHold);
      requestedHoldPosition.set(true);
   }

   @Override
   /** {@inheritDoc} */
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

   @Override
   /** {@inheritDoc} */
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

   @Override
   /** {@inheritDoc} */
   public abstract void compute(double time);

   @Override
   /** {@inheritDoc} */
   public abstract void getFinalDesiredCapturePointPosition(FramePoint3D finalDesiredCapturePointPositionToPack);

   @Override
   /** {@inheritDoc} */
   public abstract void getFinalDesiredCapturePointPosition(YoFramePoint2d finalDesiredCapturePointPositionToPack);

   @Override
   /** {@inheritDoc} */
   public abstract void getFinalDesiredCenterOfMassPosition(FramePoint3D finalDesiredCenterOfMassPositionToPack);

   @Override
   /** {@inheritDoc} */
   public abstract void getNextExitCMP(FramePoint3D entryCMPToPack);

   @Override
   /** {@inheritDoc} */
   public abstract boolean isOnExitCMP();

   @Override
   /** {@inheritDoc} */
   public abstract int getNumberOfFootstepsToConsider();

   @Override
   /** {@inheritDoc} */
   public abstract int getNumberOfFootstepsRegistered();

   protected abstract void updateTransferPlan();
   protected abstract void updateSingleSupportPlan();

   private final FramePoint2D desiredICP2d = new FramePoint2D();
   private final FramePoint2D finalICP2d = new FramePoint2D();
   private final FrameLine2d desiredICPToFinalICPLine = new FrameLine2d();
   private final FrameLineSegment2d desiredICPToFinalICPLineSegment = new FrameLineSegment2d();
   private final FramePoint2D actualICP2d = new FramePoint2D();

   private double estimateDeltaTimeBetweenDesiredICPAndActualICP(FramePoint2D actualCapturePointPosition)
   {
      desiredICPPosition.getFrameTuple2dIncludingFrame(desiredICP2d);
      singleSupportFinalICP.getFrameTuple2dIncludingFrame(finalICP2d);

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

      double actualDistanceDueToDisturbance = desiredCMPPosition.getXYPlaneDistance(actualICP2d);
      double expectedDistanceAccordingToPlan = desiredCMPPosition.getXYPlaneDistance(desiredICPPosition);

      double distanceRatio = actualDistanceDueToDisturbance / expectedDistanceAccordingToPlan;

      if (distanceRatio < 1.0e-3)
         return 0.0;
      else
         return Math.log(distanceRatio) / omega0.getDoubleValue();
   }

   @Override
   /** {@inheritDoc} */
   public void getDesiredCapturePointPosition(FramePoint3D desiredCapturePointPositionToPack)
   {
      desiredICPPosition.getFrameTupleIncludingFrame(desiredCapturePointPositionToPack);
   }

   @Override
   /** {@inheritDoc} */
   public void getDesiredCapturePointPosition(FramePoint2D desiredCapturePointPositionToPack)
   {
      desiredICPPosition.getFrameTuple2dIncludingFrame(desiredCapturePointPositionToPack);
   }

   @Override
   /** {@inheritDoc} */
   public void getDesiredCapturePointPosition(YoFramePoint desiredCapturePointPositionToPack)
   {
      desiredCapturePointPositionToPack.set(desiredICPPosition);
   }

   @Override
   /** {@inheritDoc} */
   public void getDesiredCenterOfMassPosition(FramePoint3D desiredCenterOfMassPositionToPack)
   {
      desiredCoMPosition.getFrameTupleIncludingFrame(desiredCenterOfMassPositionToPack);
   }

   /** {@inheritDoc} */
   public void getDesiredCenterOfMassPosition(YoFramePoint desiredCenterOfMassPositionToPack)
   {
      desiredCenterOfMassPositionToPack.set(desiredCoMPosition);
   }
   
   public void getDesiredCenterOfMassVelocity(YoFrameVector desiredCenterOfMassVelocityToPack)
   {
      desiredCenterOfMassVelocityToPack.set(desiredCoMVelocity);
   }
   
   public void getDesiredCenterOfMassVelocity(FrameVector3D desiredCenterOfMassVelocityToPack)
   {
      desiredCoMVelocity.getFrameTupleIncludingFrame(desiredCenterOfMassVelocityToPack);
   }
   
   public void getDesiredCenterOfMassAcceleration(YoFrameVector desiredCenterOfMassAccelerationToPack)
   {
      desiredCenterOfMassAccelerationToPack.set(desiredCoMAcceleration);
   }

   public void getDesiredCenterOfMassAcceleration(FrameVector3D desiredCenterOfMassAccelerationToPack)
   {
      desiredCoMAcceleration.getFrameTupleIncludingFrame(desiredCenterOfMassAccelerationToPack);;
   }

   @Override
   /** {@inheritDoc} */
   public void getDesiredCapturePointVelocity(FrameVector3D desiredCapturePointVelocityToPack)
   {
      desiredICPVelocity.getFrameTupleIncludingFrame(desiredCapturePointVelocityToPack);
   }

   @Override
   /** {@inheritDoc} */
   public void getDesiredCapturePointVelocity(FrameVector2D desiredCapturePointVelocityToPack)
   {
      desiredICPVelocity.getFrameTuple2dIncludingFrame(desiredCapturePointVelocityToPack);
   }

   @Override
   /** {@inheritDoc} */
   public void getDesiredCapturePointVelocity(YoFrameVector desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.set(desiredICPVelocity);
   }

   @Override
   /** {@inheritDoc} */
   public void getDesiredCapturePointAcceleration(FrameVector3D desiredCapturePointAccelerationToPack)
   {
      desiredICPAcceleration.getFrameTuple(desiredCapturePointAccelerationToPack);
   }

   @Override
   /** {@inheritDoc} */
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint3D desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCMPPosition.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   @Override
   /** {@inheritDoc} */
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint2D desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCMPPosition.getFrameTuple2dIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }
   
   /** {@inheritDoc} */
   public void getDesiredCentroidalMomentumPivotPosition(YoFramePoint desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCentroidalMomentumPivotPositionToPack.set(desiredCMPPosition);
   }

   @Override
   /** {@inheritDoc} */
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector3D desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCMPVelocity.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotVelocityToPack);
   }

   @Override
   /** {@inheritDoc} */
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector2D desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCMPVelocity.getFrameTuple2dIncludingFrame(desiredCentroidalMomentumPivotVelocityToPack);
   }
   
   /** {@inheritDoc} */
   public void getDesiredCentroidalMomentumPivotVelocity(YoFrameVector desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCentroidalMomentumPivotVelocityToPack.set(desiredCMPVelocity);
   }
   
   /** {@inheritDoc} */
   public void getDesiredCenterOfPressurePosition(FramePoint3D desiredCenterOfPressurePositionToPack)
   {
      desiredCoPPosition.getFrameTupleIncludingFrame(desiredCenterOfPressurePositionToPack);
   }

   /** {@inheritDoc} */
   public void getDesiredCenterOfPressurePosition(FramePoint2D desiredCenterOfPressurePositionToPack)
   {
      desiredCoPPosition.getFrameTuple2dIncludingFrame(desiredCenterOfPressurePositionToPack);
   }

   /** {@inheritDoc} */
   public void getDesiredCenterOfPressurePosition(YoFramePoint desiredCenterOfPressurePositionToPack)
   {
      desiredCenterOfPressurePositionToPack.set(desiredCoPPosition);
   }
   
   /** {@inheritDoc} */
   public void getDesiredCenterOfPressureVelocity(FrameVector3D desiredCenterOfPressureVelocityToPack)
   {
      desiredCoPVelocity.getFrameTupleIncludingFrame(desiredCenterOfPressureVelocityToPack);
   }

   /** {@inheritDoc} */
   public void getDesiredCenterOfPressureVelocity(FrameVector2D desiredCenterOfPressureVelocityToPack)
   {
      desiredCoPVelocity.getFrameTuple2dIncludingFrame(desiredCenterOfPressureVelocityToPack);
   }

   /** {@inheritDoc} */
   public void getDesiredCenterOfPressureVelocity(YoFrameVector desiredCenterOfPressureVelocityToPack)
   {
      desiredCenterOfPressureVelocityToPack.set(desiredCoPVelocity);
   }
   
   /** {@inheritDoc} */
   public void getDesiredCentroidalAngularMomentum(FrameVector3D desiredCentroidalAngularMomentumToPack)
   {
      desiredCentroidalAngularMomentum.getFrameTupleIncludingFrame(desiredCentroidalAngularMomentumToPack);
   }
   
   /** {@inheritDoc} */
   public void getDesiredCentroidalTorque(YoFrameVector desiredCentroidalTorqueToPack)
   {
      desiredCentroidalTorqueToPack.set(desiredCentroidalTorque);
   }
   
   public void getDesiredCentroidalTorque(FrameVector3D desiredCentroidalTorqueToPack)
   {
      desiredCentroidalTorque.getFrameTupleIncludingFrame(desiredCentroidalTorqueToPack);
   }

   @Override
   /** {@inheritDoc} */
   public double getTimeInCurrentState()
   {
      return timeInCurrentState.getDoubleValue();
   }

   @Override
   /** {@inheritDoc} */
   public double getTimeInCurrentStateRemaining()
   {
      return timeInCurrentStateRemaining.getDoubleValue();
   }

   @Override
   /** {@inheritDoc} */
   public double getCurrentStateDuration()
   {
      if (isDoubleSupport.getBooleanValue())
         return getTransferDuration(0);
      else
         return getSwingDuration(0);
   }

   @Override
   /** {@inheritDoc} */
   public void setTransferDuration(int stepNumber, double duration)
   {
      transferDurations.get(stepNumber).set(duration);
   }

   @Override
   /** {@inheritDoc} */
   public void setSwingDuration(int stepNumber, double duration)
   {
      swingDurations.get(stepNumber).set(duration);
   }

   @Override
   /** {@inheritDoc} */
   public void setTouchdownDuration(int stepNumber, double duration)
   {
      touchdownDurations.get(stepNumber).set(duration);
   }
   
    @Override
   public double getTouchdownDuration(int stepNumber)
   {
       return touchdownDurations.get(stepNumber).getDoubleValue();
   }

   @Override
   /** {@inheritDoc} */
   public double getTransferDuration(int stepNumber)
   {
      return transferDurations.get(stepNumber).getDoubleValue();
   }

   @Override
   /** {@inheritDoc} */
   public double getSwingDuration(int stepNumber)
   {
      return swingDurations.get(stepNumber).getDoubleValue();
   }

   @Override
   /** {@inheritDoc} */
   public void setFinalTransferDuration(double duration)
   {
      if(duration < Epsilons.ONE_HUNDREDTH)
         return;
      defaultFinalTransferDuration.set(duration);
   }

   @Override
   /** {@inheritDoc} */
   public void setFinalTransferDurationAlpha(double durationAlpha)
   {
      finalTransferDurationAlpha.set(durationAlpha);
   }

   @Override
   /** {@inheritDoc} */
   public void setTransferDurationAlpha(int stepNumber, double transferDurationAlpha)
   {
      transferDurationAlphas.get(stepNumber).set(transferDurationAlpha);
   }

   @Override
   /** {@inheritDoc} */
   public void setSwingDurationAlpha(int stepNumber, double swingDurationAlpha)
   {
      swingDurationAlphas.get(stepNumber).set(swingDurationAlpha);
   }

   @Override
   /** {@inheritDoc} */
   public double getTransferDurationAlpha(int stepNumber)
   {
      return transferDurationAlphas.get(stepNumber).getDoubleValue();
   }

   @Override
   /** {@inheritDoc} */
   public double getSwingDurationAlpha(int stepNumber)
   {
      return swingDurationAlphas.get(stepNumber).getDoubleValue();
   }


   @Override
   /** {@inheritDoc} */
   public double getInitialTime()
   {
      return initialTime.getDoubleValue();
   }

   @Override
   /** {@inheritDoc} */
   public void setOmega0(double omega0)
   {
      this.omega0.set(omega0);
   }

   @Override
   /** {@inheritDoc} */
   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   @Override
   /** {@inheritDoc} */
   public RobotSide getTransferToSide()
   {
      return transferToSide.getEnumValue();
   }


   @Override
   /** {@inheritDoc} */
   public boolean isInDoubleSupport()
   {
      return isDoubleSupport.getBooleanValue();
   }

   @Override
   /** {@inheritDoc} */
   public boolean isInStanding()
   {
      return isStanding.getBooleanValue();
   }

   @Override
   /** {@inheritDoc} */
   public boolean isInInitialTransfer()
   {
      return isInitialTransfer.getBooleanValue();
   }

   // This is a hack. Please remove in case of any issues.
   public void setInInitialTransfer(boolean isInitialTransfer)
   {
      this.isInitialTransfer.set(isInitialTransfer);
   }
   
   @Override
   /** {@inheritDoc} */
   public boolean isDone()
   {
      return timeInCurrentStateRemaining.getDoubleValue() <= 0.0;
   }


}
