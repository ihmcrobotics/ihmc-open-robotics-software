package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPPlannerInterface;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.AngularMomentumTrajectoryMultiplexer;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CMPGeneration.ReferenceCMPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoMGeneration.ReferenceCoMTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.FootstepData;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.ReferenceCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration.ReferenceICPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.messageHandlers.MomentumTrajectoryHandler;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFramePoint3D;

public class SmoothCMPBasedICPPlanner implements ICPPlannerInterface
{
   private static final boolean VISUALIZE = true;
   private static final boolean debug = false;

   private static final RobotSide defaultTransferToSide = RobotSide.LEFT;

   private static final double ZERO_TIME = 0.0;

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   protected final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   protected final String namePrefix = "icpPlanner";

   protected final YoInteger numberFootstepsToConsider = new YoInteger(namePrefix + "NumberFootstepsToConsider", registry);
   protected final YoBoolean isStanding = new YoBoolean(namePrefix + "IsStanding", registry);
   protected final YoBoolean isInitialTransfer = new YoBoolean(namePrefix + "IsInitialTransfer", registry);
   protected final YoBoolean isFinalTransfer = new YoBoolean(namePrefix + "IsFinalTransfer", registry);
   protected final YoBoolean isDoubleSupport = new YoBoolean(namePrefix + "IsDoubleSupport", registry);

   protected final ExecutionTimer timer = new ExecutionTimer(namePrefix + "Timer", registry);

   /////////////////////////////// Start Planner Output ///////////////////////////////

   /** Desired position for the Centroidal Momentum Pivot (CMP) */
   protected final YoFramePoint3D desiredCMPPosition = new YoFramePoint3D(namePrefix + "DesiredCMPPosition", worldFrame, registry);
   /** Desired velocity for the Centroidal Momentum Pivot (CMP) */
   protected final YoFrameVector3D desiredCMPVelocity = new YoFrameVector3D(namePrefix + "DesiredCMPVelocity", worldFrame, registry);
   /** Desired position for the Instantaneous Capture Point (ICP) */
   protected final YoFramePoint3D desiredICPPosition = new YoFramePoint3D(namePrefix + "DesiredICPPosition", worldFrame, registry);
   /** Desired velocity for the Instantaneous Capture Point (ICP) */
   protected final YoFrameVector3D desiredICPVelocity = new YoFrameVector3D(namePrefix + "DesiredICPVelocity", worldFrame, registry);
   /** Desired acceleration for the Instantaneous Capture Point (ICP) */
   protected final YoFrameVector3D desiredICPAcceleration = new YoFrameVector3D(namePrefix + "DesiredICPAcceleration", worldFrame, registry);
   /** Desired position for the Center of Mass (CoM)*/
   protected final YoFramePoint3D desiredCoMPosition = new YoFramePoint3D(namePrefix + "DesiredCoMPosition", worldFrame, registry);

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

   protected final YoMutableFramePoint3D singleSupportInitialICP;
   protected final YoFrameVector3D singleSupportInitialICPVelocity = new YoFrameVector3D(namePrefix + "SingleSupportInitialICPVelocity", worldFrame, registry);

   protected final YoMutableFramePoint3D singleSupportFinalICP;
   protected final YoFrameVector3D singleSupportFinalICPVelocity = new YoFrameVector3D(namePrefix + "SingleSupportFinalICPVelocity", worldFrame, registry);

   protected final YoBoolean requestedHoldPosition = new YoBoolean(namePrefix + "RequestedHoldPosition", registry);
   protected final YoBoolean isHoldingPosition = new YoBoolean(namePrefix + "IsHoldingPosition", registry);
   protected final YoFramePoint3D icpPositionToHold = new YoFramePoint3D(namePrefix + "CapturePointPositionToHold", worldFrame, registry);

   protected final YoEnum<RobotSide> transferToSide = new YoEnum<>(namePrefix + "TransferToSide", registry, RobotSide.class, true);
   protected final YoEnum<RobotSide> previousTransferToSide = new YoEnum<>(namePrefix + "PreviousTransferToSide", registry, RobotSide.class, true);
   protected final YoEnum<RobotSide> supportSide = new YoEnum<>(namePrefix + "SupportSide", registry, RobotSide.class, true);

   protected final List<YoDouble> swingDurations = new ArrayList<>();
   protected final List<YoDouble> transferDurations = new ArrayList<>();
   protected final YoDouble defaultFinalTransferDuration = new YoDouble(namePrefix + "DefaultFinalTransferDuration", registry);
   protected final YoDouble finalTransferDuration = new YoDouble(namePrefix + "FinalTransferDuration", registry);

   /** Desired velocity for the Center of Mass (CoM) */
   final YoFrameVector3D desiredCoMVelocity = new YoFrameVector3D(namePrefix + "DesiredCoMVelocity", worldFrame, registry);
   /** Desired acceleration for the Center of Mass (CoM) */
   final YoFrameVector3D desiredCoMAcceleration = new YoFrameVector3D(namePrefix + "DesiredCoMAcceleration", worldFrame, registry);
   /** Desired position for the Center of Pressure (CoP) */
   private final YoFramePoint3D desiredCoPPosition = new YoFramePoint3D(namePrefix + "DesiredCoPPosition", worldFrame, registry);
   /** Desired velocity for the Center of Pressure (CoP) */
   private final YoFrameVector3D desiredCoPVelocity = new YoFrameVector3D(namePrefix + "DesiredCoPVelocity", worldFrame, registry);

   /** Desired Centroidal Angular Momentum (CAM) */
   final YoFrameVector3D desiredCentroidalAngularMomentum = new YoFrameVector3D(namePrefix + "DesiredCentroidalAngularMomentum", worldFrame, registry);
   /** Desired Centroidal Torque (CT) */
   final YoFrameVector3D desiredCentroidalTorque = new YoFrameVector3D(namePrefix + "DesiredCentroidalTorque", worldFrame, registry);

   private final YoBoolean adjustPlanForSSContinuity = new YoBoolean("adjustCoPPlanForSSContinuity", registry);
   private final YoBoolean adjustPlanForDSContinuity = new YoBoolean("adjustEveryCoPPlanForDSContinuity", registry);
   private final YoBoolean adjustPlanForInitialDSContinuity = new YoBoolean("adjustInitialCoPPlanForDSContinuity", registry);
   private final YoBoolean adjustPlanForStandingContinuity = new YoBoolean("adjustCoPPlanForInitialDSContinuity", registry);
   private final YoBoolean doContinuousReplanningForStanding = new YoBoolean("doContinuousReplanningForStanding", registry);
   private final YoBoolean doContinuousReplanningForTransfer = new YoBoolean("doContinuousReplanningForTransfer", registry);
   private final YoBoolean doContinuousReplanningForSwing = new YoBoolean("doContinuousReplanningForSwing", registry);

   private final ReferenceCoPTrajectoryGenerator referenceCoPGenerator;
   private final ReferenceCMPTrajectoryGenerator referenceCMPGenerator;
   private final ReferenceICPTrajectoryGenerator referenceICPGenerator;
   private final ReferenceCoMTrajectoryGenerator referenceCoMGenerator;
   private final AngularMomentumTrajectoryMultiplexer angularMomentumTrajectoryGenerator;

   private final List<YoDouble> swingDurationShiftFractions = new ArrayList<>();
   private final YoDouble defaultSwingDurationShiftFraction;

   private final YoInteger numberOfUpcomingFootsteps;
   private final RecyclingArrayList<FootstepData> upcomingFootstepsData;

   private static final double ICP_CORNER_POINT_SIZE = 0.002;
   private List<YoMutableFramePoint3D> icpPhaseEntryCornerPoints = new ArrayList<>();
   private List<YoMutableFramePoint3D> icpPhaseExitCornerPoints = new ArrayList<>();

   private final double robotMass;
   private final double gravityZ;

   private final FramePoint3D tempPoint = new FramePoint3D();

   private final YoFramePoint3D yoSingleSupportFinalCoM;
   private final FramePoint3D singleSupportFinalCoM = new FramePoint3D();

   private final int maxNumberOfICPCornerPointsVisualized = 20;

   private final YoBoolean areCoMDynamicsSatisfied;

   private final List<ImmutablePair<FrameTuple3DReadOnly, FixedFrameTuple3DBasics>> visualizationUpdatables = new ArrayList<>();

   public SmoothCMPBasedICPPlanner(FullRobotModel fullRobotModel, BipedSupportPolygons bipedSupportPolygons,
                                   SideDependentList<? extends ReferenceFrame> soleZUpFrames, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                   MomentumTrajectoryHandler momentumTrajectoryHandler, YoDouble yoTime, YoVariableRegistry parentRegistry,
                                   YoGraphicsListRegistry yoGraphicsListRegistry, double gravityZ, ICPPlannerParameters icpPlannerParameters)
   {

      this(fullRobotModel.getTotalMass(), bipedSupportPolygons, soleZUpFrames, contactableFeet, momentumTrajectoryHandler, yoTime, parentRegistry,
           yoGraphicsListRegistry, gravityZ, icpPlannerParameters);
   }

   public SmoothCMPBasedICPPlanner(double robotMass, BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                   SideDependentList<? extends ContactablePlaneBody> contactableFeet, MomentumTrajectoryHandler momentumTrajectoryHandler,
                                   YoDouble yoTime, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, double gravityZ,
                                   ICPPlannerParameters icpPlannerParameters)
   {
      isStanding.set(true);

      finalTransferDuration.setToNaN();

      icpPositionToHold.setToNaN();
      isHoldingPosition.set(false);

      // Initialize omega0 to NaN to force the user to explicitly set it.
      omega0.set(Double.NaN);

      singleSupportInitialICP = new YoMutableFramePoint3D(namePrefix + "SingleSupportInitialICP", "", registry);
      singleSupportFinalICP = new YoMutableFramePoint3D(namePrefix + "SingleSupportFinalICP", "", registry);

      transferToSide.set(null);
      previousTransferToSide.set(null);

      int maxNumberOfFootstepsToConsider = icpPlannerParameters.getNumberOfFootstepsToConsider();
      upcomingFootstepsData = new RecyclingArrayList<>(maxNumberOfFootstepsToConsider, FootstepData.class);

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         YoDouble swingDuration = new YoDouble(namePrefix + "SwingDuration" + i, registry);
         swingDuration.setToNaN();
         swingDurations.add(swingDuration);
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
      YoDouble transferDuration = new YoDouble(namePrefix + "TransferDuration" + maxNumberOfFootstepsToConsider, registry);
      YoDouble transferDurationAlpha = new YoDouble(namePrefix + "TransferDurationAlpha" + maxNumberOfFootstepsToConsider,
                                                    "Repartition of the transfer duration around the entry corner point.", registry);
      transferDuration.setToNaN();
      transferDurationAlpha.setToNaN();
      transferDurations.add(transferDuration);
      transferDurationAlphas.add(transferDurationAlpha);

      yoSingleSupportFinalCoM = new YoFramePoint3D(namePrefix + "SingleSupportFinalCoM", worldFrame, registry);

      this.gravityZ = gravityZ;
      defaultSwingDurationShiftFraction = new YoDouble(namePrefix + "DefaultSwingDurationShiftFraction", registry);
      numberOfUpcomingFootsteps = new YoInteger(namePrefix + "NumberOfUpcomingFootsteps", registry);

      this.robotMass = robotMass;

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         YoDouble swingDurationShiftFraction = new YoDouble(namePrefix + "SwingDurationShiftFraction" + i, registry);
         swingDurationShiftFractions.add(swingDurationShiftFraction);
      }

      for (int i = 0; i < maxNumberOfICPCornerPointsVisualized - 1; i++)
      {
         icpPhaseEntryCornerPoints.add(new YoMutableFramePoint3D(namePrefix + "EntryCornerPoints" + i, "", registry));
         icpPhaseExitCornerPoints.add(new YoMutableFramePoint3D(namePrefix + "ExitCornerPoints" + i, "", registry));
      }

      referenceCoPGenerator = new ReferenceCoPTrajectoryGenerator(namePrefix, maxNumberOfFootstepsToConsider, bipedSupportPolygons, contactableFeet,
                                                                  numberFootstepsToConsider, swingDurations, transferDurations, swingDurationAlphas,
                                                                  swingDurationShiftFractions, transferDurationAlphas, debug, numberOfUpcomingFootsteps,
                                                                  upcomingFootstepsData, soleZUpFrames, registry);
      referenceCMPGenerator = new ReferenceCMPTrajectoryGenerator(namePrefix, maxNumberOfFootstepsToConsider, numberFootstepsToConsider, true, registry,
                                                                  yoGraphicsListRegistry);

      referenceICPGenerator = new ReferenceICPTrajectoryGenerator(namePrefix, omega0, numberFootstepsToConsider, isInitialTransfer, isStanding, true, registry,
                                                                  yoGraphicsListRegistry);

      referenceCoMGenerator = new ReferenceCoMTrajectoryGenerator(namePrefix, omega0, numberFootstepsToConsider, isInitialTransfer, isDoubleSupport, registry);
      angularMomentumTrajectoryGenerator = new AngularMomentumTrajectoryMultiplexer(namePrefix, momentumTrajectoryHandler, yoTime, omega0, debug, registry);

      areCoMDynamicsSatisfied = new YoBoolean("areCoMDynamicsSatisfied", registry);
      areCoMDynamicsSatisfied.set(false);

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry != null)
      {
         setupVisualizers(yoGraphicsListRegistry);
      }

      initializeParameters(icpPlannerParameters);
   }

   private void initializeParameters(ICPPlannerParameters icpPlannerParameters)
   {
      defaultTransferDurationAlpha.set(icpPlannerParameters.getTransferSplitFraction());
      defaultSwingDurationAlpha.set(icpPlannerParameters.getSwingSplitFraction());
      finalTransferDurationAlpha.set(icpPlannerParameters.getTransferSplitFraction());

      velocityDecayDurationWhenDone.set(icpPlannerParameters.getVelocityDecayDurationWhenDone());
      velocityReductionFactor.set(Double.NaN);

      numberFootstepsToConsider.set(icpPlannerParameters.getNumberOfFootstepsToConsider());

      referenceCoPGenerator.initializeParameters(icpPlannerParameters);
      referenceCMPGenerator.setGroundReaction(robotMass * gravityZ);
      angularMomentumTrajectoryGenerator.initializeParameters(icpPlannerParameters, robotMass, gravityZ);
      defaultSwingDurationShiftFraction.set(icpPlannerParameters.getSwingDurationShiftFraction());

      adjustPlanForSSContinuity.set(icpPlannerParameters.adjustCoPPlanForSingleSupportContinuity());
      adjustPlanForDSContinuity.set(icpPlannerParameters.adjustEveryCoPPlanForDoubleSupportContinuity());
      adjustPlanForInitialDSContinuity.set(icpPlannerParameters.adjustInitialCoPPlanForDoubleSupportContinuity());
      adjustPlanForStandingContinuity.set(icpPlannerParameters.adjustCoPPlanForStandingContinuity());

      doContinuousReplanningForStanding.set(icpPlannerParameters.doContinuousReplanningForStanding());
      doContinuousReplanningForTransfer.set(icpPlannerParameters.doContinuousReplanningForTransfer());
      doContinuousReplanningForSwing.set(icpPlannerParameters.doContinuousReplanningForSwing());
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      referenceCoPGenerator.createVisualizerForConstantCoPs(yoGraphicsList, artifactList);

      for (int i = 0; i < maxNumberOfICPCornerPointsVisualized - 1; i++)
      {
         YoFramePoint3D icpEntryCornerPointInWorld = new YoFramePoint3D(namePrefix + "EntryCornerPoints" + i, "WorldViz", worldFrame, registry);
         visualizationUpdatables.add(new ImmutablePair<>(icpPhaseEntryCornerPoints.get(i), icpEntryCornerPointInWorld));
         YoGraphicPosition icpEntryCornerPointsViz = new YoGraphicPosition("ICPEntryCornerPoints" + i, icpEntryCornerPointInWorld, ICP_CORNER_POINT_SIZE,
                                                                           YoAppearance.Blue(), GraphicType.SOLID_BALL);

         yoGraphicsList.add(icpEntryCornerPointsViz);
         artifactList.add(icpEntryCornerPointsViz.createArtifact());

         YoFramePoint3D icpExitCornerPointInWorld = new YoFramePoint3D(namePrefix + "ExitCornerPoints" + i, "WorldViz", worldFrame, registry);
         visualizationUpdatables.add(new ImmutablePair<>(icpPhaseExitCornerPoints.get(i), icpExitCornerPointInWorld));
         YoGraphicPosition icpExitCornerPointsViz = new YoGraphicPosition("ICPExitCornerPoints" + i, icpExitCornerPointInWorld, ICP_CORNER_POINT_SIZE,
                                                                          YoAppearance.Blue(), GraphicType.BALL);

         yoGraphicsList.add(icpExitCornerPointsViz);
         artifactList.add(icpExitCornerPointsViz.createArtifact());
      }

      if (debug)
      {
         YoGraphicPosition referenceCMPPositionViz = new YoGraphicPosition("Reference CMP", desiredCMPPosition, 0.005, YoAppearance.Purple(),
                                                                           GraphicType.SOLID_BALL);
         YoGraphicPosition referenceCoPPositionViz = new YoGraphicPosition("Reference CoP", desiredCoPPosition, 0.005, YoAppearance.Green(),
                                                                           GraphicType.SOLID_BALL);

         yoGraphicsList.add(referenceCMPPositionViz);
         yoGraphicsList.add(referenceCoPPositionViz);
         artifactList.add(referenceCMPPositionViz.createArtifact());
         artifactList.add(referenceCoPPositionViz.createArtifact());
      }

      artifactList.setVisible(VISUALIZE);
      yoGraphicsList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   @Override
   public void setSupportLeg(RobotSide robotSide)
   {
      supportSide.set(robotSide);
   }

   @Override
   public void setTransferToSide(RobotSide robotSide)
   {
      previousTransferToSide.set(transferToSide.getEnumValue());
      transferToSide.set(robotSide);
   }

   @Override
   public void setTransferFromSide(RobotSide robotSide)
   {
      if (robotSide != null)
      {
         previousTransferToSide.set(transferToSide.getEnumValue());
         transferToSide.set(robotSide.getOppositeSide());
      }
      else
      {
         transferToSide.set(null);
      }
   }

   @Override
   public double estimateTimeRemainingForStateUnderDisturbance(FramePoint2DReadOnly actualCapturePointPosition)
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

   private final FramePoint2D desiredICP2d = new FramePoint2D();
   private final FramePoint2D finalICP2d = new FramePoint2D();
   private final FrameLine2D desiredICPToFinalICPLine = new FrameLine2D();
   private final FrameLineSegment2D desiredICPToFinalICPLineSegment = new FrameLineSegment2D();
   private final FramePoint2D actualICP2d = new FramePoint2D();

   private double estimateDeltaTimeBetweenDesiredICPAndActualICP(FramePoint2DReadOnly actualCapturePointPosition)
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

   @Override
   public void getDesiredCapturePointPosition(FramePoint3D desiredCapturePointPositionToPack)
   {
      desiredCapturePointPositionToPack.setIncludingFrame(desiredICPPosition);
   }

   @Override
   public void getDesiredCapturePointPosition(FramePoint2D desiredCapturePointPositionToPack)
   {
      desiredCapturePointPositionToPack.setIncludingFrame(desiredICPPosition);
   }

   @Override
   public void getDesiredCapturePointPosition(YoFramePoint3D desiredCapturePointPositionToPack)
   {
      desiredCapturePointPositionToPack.set(desiredICPPosition);
   }

   @Override
   public void getDesiredCenterOfMassPosition(FramePoint3D desiredCenterOfMassPositionToPack)
   {
      desiredCenterOfMassPositionToPack.setIncludingFrame(desiredCoMPosition);
   }

   @Override
   public void getDesiredCenterOfMassPosition(YoFramePoint3D desiredCenterOfMassPositionToPack)
   {
      desiredCenterOfMassPositionToPack.set(desiredCoMPosition);
   }

   @Override
   public void getDesiredCapturePointVelocity(FrameVector3D desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.setIncludingFrame(desiredICPVelocity);
   }

   @Override
   public void getDesiredCapturePointVelocity(FrameVector2D desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.setIncludingFrame(desiredICPVelocity);
   }

   @Override
   public void getDesiredCapturePointVelocity(YoFrameVector3D desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.set(desiredICPVelocity);
   }

   @Override
   public void getDesiredCapturePointAcceleration(FrameVector3D desiredCapturePointAccelerationToPack)
   {
      desiredCapturePointAccelerationToPack.set(desiredICPAcceleration);
   }

   @Override
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint3D desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCentroidalMomentumPivotPositionToPack.setIncludingFrame(desiredCMPPosition);
   }

   @Override
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint2D desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCentroidalMomentumPivotPositionToPack.setIncludingFrame(desiredCMPPosition);
   }

   public void getDesiredCentroidalMomentumPivotPosition(YoFramePoint3D desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCentroidalMomentumPivotPositionToPack.set(desiredCMPPosition);
   }

   @Override
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector3D desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCentroidalMomentumPivotVelocityToPack.setIncludingFrame(desiredCMPVelocity);
   }

   @Override
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector2D desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCentroidalMomentumPivotVelocityToPack.setIncludingFrame(desiredCMPVelocity);
   }

   public void getDesiredCentroidalMomentumPivotVelocity(YoFrameVector3D desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCentroidalMomentumPivotVelocityToPack.set(desiredCMPVelocity);
   }

   @Override
   public double getTimeInCurrentState()
   {
      return timeInCurrentState.getDoubleValue();
   }

   @Override
   public double getTimeInCurrentStateRemaining()
   {
      return timeInCurrentStateRemaining.getDoubleValue();
   }

   @Override
   public double getCurrentStateDuration()
   {
      if (isDoubleSupport.getBooleanValue())
         return getTransferDuration(0);
      else
         return getSwingDuration(0);
   }

   @Override
   public void setTransferDuration(int stepNumber, double duration)
   {
      transferDurations.get(stepNumber).set(duration);
   }

   @Override
   public void setSwingDuration(int stepNumber, double duration)
   {
      swingDurations.get(stepNumber).set(duration);
   }

   @Override
   public double getTransferDuration(int stepNumber)
   {
      return transferDurations.get(stepNumber).getDoubleValue();
   }

   @Override
   public double getSwingDuration(int stepNumber)
   {
      return swingDurations.get(stepNumber).getDoubleValue();
   }

   @Override
   public void setFinalTransferDuration(double duration)
   {
      if(duration < Epsilons.ONE_HUNDREDTH)
         return;
      defaultFinalTransferDuration.set(duration);
   }

   @Override
   public void setFinalTransferDurationAlpha(double durationAlpha)
   {
      finalTransferDurationAlpha.set(durationAlpha);
   }

   @Override
   public void setTransferDurationAlpha(int stepNumber, double transferDurationAlpha)
   {
      transferDurationAlphas.get(stepNumber).set(transferDurationAlpha);
   }

   @Override
   public void setSwingDurationAlpha(int stepNumber, double swingDurationAlpha)
   {
      swingDurationAlphas.get(stepNumber).set(swingDurationAlpha);
   }

   @Override
   public double getTransferDurationAlpha(int stepNumber)
   {
      return transferDurationAlphas.get(stepNumber).getDoubleValue();
   }

   @Override
   public double getSwingDurationAlpha(int stepNumber)
   {
      return swingDurationAlphas.get(stepNumber).getDoubleValue();
   }

   @Override
   public double getInitialTime()
   {
      return initialTime.getDoubleValue();
   }

   @Override
   public void setOmega0(double omega0)
   {
      this.omega0.set(omega0);
   }

   @Override
   public double getOmega0()
   {
      return omega0.getDoubleValue();
   }

   @Override
   public RobotSide getTransferToSide()
   {
      return transferToSide.getEnumValue();
   }

   @Override
   public boolean isInDoubleSupport()
   {
      return isDoubleSupport.getBooleanValue();
   }

   @Override
   public boolean isInStanding()
   {
      return isStanding.getBooleanValue();
   }

   @Override
   public boolean isInInitialTransfer()
   {
      return isInitialTransfer.getBooleanValue();
   }

   @Override
   public boolean isDone()
   {
      if (timeInCurrentStateRemaining.isNaN())
      {
         return true;
      }
      return timeInCurrentStateRemaining.getDoubleValue() <= 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public void clearPlan()
   {
      referenceCoPGenerator.clear();
      referenceCMPGenerator.reset();
      referenceICPGenerator.reset();
      angularMomentumTrajectoryGenerator.clear();

      for (int i = 0; i < swingDurations.size(); i++)
      {
         swingDurations.get(i).setToNaN();
         transferDurations.get(i).setToNaN();
         swingDurationAlphas.get(i).setToNaN();
         transferDurationAlphas.get(i).setToNaN();
         swingDurationShiftFractions.get(i).setToNaN();
      }

      numberOfUpcomingFootsteps.set(0);
      upcomingFootstepsData.clear();
   }

   public void clearPlanWithoutClearingPlannedFootsteps()
   {
      referenceCoPGenerator.clearPlan();
      referenceCMPGenerator.reset();
      referenceICPGenerator.reset();
      angularMomentumTrajectoryGenerator.clear();
   }

   /** {@inheritDoc} */
   @Override
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      if (footstep == null)
      {
         LogTools.warn("Received null footstep, ignoring");
         return;
      }
      else if (timing == null)
      {
         LogTools.warn("Received null step timing, ignoring");
         return;
      }
      else if (footstep.getSoleReferenceFrame().getTransformToRoot().containsNaN())
      {
         LogTools.warn("Received bad footstep: " + footstep + ", ignoring");
         return;
      }
      else
      {
         upcomingFootstepsData.add().set(footstep, timing);
         numberOfUpcomingFootsteps.increment();
      }

      int footstepIndex = referenceCoPGenerator.getNumberOfFootstepsRegistered() - 1;

      double swingDuration = timing.getSwingTime();
      double transferTime = timing.getTransferTime();

      if (!Double.isFinite(swingDuration) || swingDuration < 0.0)
         swingDuration = 1.0;

      if (!Double.isFinite(transferTime) || transferTime < 0.0)
         transferTime = 1.0;

      swingDurations.get(footstepIndex).set(swingDuration);
      transferDurations.get(footstepIndex).set(transferTime);

      swingDurationAlphas.get(footstepIndex).set(defaultSwingDurationAlpha.getDoubleValue());
      transferDurationAlphas.get(footstepIndex).set(defaultTransferDurationAlpha.getDoubleValue());
      swingDurationShiftFractions.get(footstepIndex).set(defaultSwingDurationShiftFraction.getDoubleValue());

      finalTransferDuration.set(defaultFinalTransferDuration.getDoubleValue());
      finalTransferDurationAlpha.set(defaultTransferDurationAlpha.getDoubleValue());
   }

   /** {@inheritDoc} */
   @Override
   public void initializeForStanding(double initialTime)
   {
      clearPlan();

      previousTransferToSide.set(null);

      this.initialTime.set(initialTime);
      isInitialTransfer.set(true);
      isFinalTransfer.set(false);
      isStanding.set(true);
      isDoubleSupport.set(true);
      transferDurations.get(0).set(finalTransferDuration.getDoubleValue());
      transferDurationAlphas.get(0).set(finalTransferDurationAlpha.getDoubleValue());
      referenceICPGenerator.setInitialConditionsForAdjustment();
      referenceCoMGenerator.initializeForSwingOrTransfer();

      // If continuous update is enabled the plan will be updated in the compute method. If not, we update the plan here.
      if (!doContinuousReplanningForStanding.getValue())
      {
         updateTransferPlan(adjustPlanForStandingContinuity.getBooleanValue());
      }
   }

   /** {@inheritDoc} */
   @Override
   public void initializeForTransfer(double initialTime)
   {
      this.initialTime.set(initialTime);
      isDoubleSupport.set(true);
      isInitialTransfer.set(isStanding.getBooleanValue());

      if (isInitialTransfer.getBooleanValue() || isFinalTransfer.getValue())
         previousTransferToSide.set(null);

      isStanding.set(false);
      int numberOfFootstepRegistered = getNumberOfFootstepsRegistered();
      isFinalTransfer.set(numberOfFootstepRegistered == 0);
      transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());
      transferDurationAlphas.get(numberOfFootstepRegistered).set(finalTransferDurationAlpha.getDoubleValue());
      referenceICPGenerator.setInitialConditionsForAdjustment();
      referenceCoMGenerator.initializeForSwingOrTransfer();

      // If continuous update is enabled the plan will be updated in the compute method. If not, we update the plan here.
      if (!doContinuousReplanningForTransfer.getValue())
      {
         updateTransferPlan(true);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void computeFinalCoMPositionInTransfer()
   {
      referenceCoMGenerator.getFinalCoMPositionInTransfer(singleSupportFinalCoM);
      yoSingleSupportFinalCoM.set(singleSupportFinalCoM);
   }

   /** {@inheritDoc} */
   @Override
   public void initializeForSingleSupport(double initialTime)
   {
      this.initialTime.set(initialTime);
      isStanding.set(false);
      isDoubleSupport.set(false);

      isInitialTransfer.set(false);
      isFinalTransfer.set(false);
      isHoldingPosition.set(false);

      int numberOfFootstepRegistered = getNumberOfFootstepsRegistered();
      transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());
      transferDurationAlphas.get(numberOfFootstepRegistered).set(finalTransferDurationAlpha.getDoubleValue());
      referenceICPGenerator.setInitialConditionsForAdjustment();
      referenceCoPGenerator.initializeForSwing();
      referenceCoMGenerator.initializeForSwingOrTransfer();

      // If continuous update is enabled the plan will be updated in the compute method. If not, we update the plan here.
      if (!doContinuousReplanningForSwing.getValue())
      {
         updateSingleSupportPlan(true);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void computeFinalCoMPositionInSwing()
   {
      referenceCoMGenerator.getFinalCoMPositionInSwing(singleSupportFinalCoM);
      yoSingleSupportFinalCoM.set(singleSupportFinalCoM);
   }

   @Override
   public void updateCurrentPlan()
   {
      if (isInStanding() && !doContinuousReplanningForStanding.getValue())
      {
         updateTransferPlan(true);
      }
      else if (isInDoubleSupport() && !isInStanding() && !doContinuousReplanningForTransfer.getValue())
      {
         updateTransferPlan(true);
      }
      else if (!isInDoubleSupport() && !doContinuousReplanningForSwing.getValue())
      {
         updateSingleSupportPlan(true);
      }
   }

   protected void updateTransferPlan(boolean maintainContinuity)
   {
      updateCount();

      clearPlanWithoutClearingPlannedFootsteps();
      RobotSide transferToSide = this.transferToSide.getEnumValue();
      if (transferToSide == null)
         transferToSide = defaultTransferToSide;

      boolean smoothForInitialContinuity = (adjustPlanForInitialDSContinuity.getBooleanValue() && (isStanding.getBooleanValue() || isInitialTransfer
            .getBooleanValue()));
      boolean smoothForFinalContinuity = numberOfUpcomingFootsteps.getIntegerValue() == 0 && adjustPlanForStandingContinuity.getBooleanValue();
      boolean smoothForContinuity = adjustPlanForDSContinuity.getBooleanValue() || smoothForInitialContinuity || smoothForFinalContinuity;
      boolean performSmoothingAdjustment = maintainContinuity && smoothForContinuity;
      referenceCoPGenerator.setGoingToPerformDSSmoothingAdjustment(performSmoothingAdjustment);

      // TODO set up the CoP Generator to be able to only update the current Support Feet CMPs
      referenceCoPGenerator
            .computeReferenceCoPsStartingFromDoubleSupport(isInitialTransfer.getBooleanValue(), transferToSide, previousTransferToSide.getEnumValue());
      referenceCMPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceICPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());

      referenceICPGenerator.initializeForTransferFromCoPs(referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories());
      if (performSmoothingAdjustment)
         referenceICPGenerator.adjustDesiredTrajectoriesForInitialSmoothing();
      referenceCoMGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());

      if (angularMomentumTrajectoryGenerator.isPredictingAngularMomentum())
      {
         referenceCoMGenerator
               .computeTrajectoryStartingFromTransfer(referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories(),
                                                      referenceICPGenerator.getICPPositonFromCoPDesiredFinalList());
      }

      angularMomentumTrajectoryGenerator
            .addCopAndComSetpointsToPlan(referenceCoPGenerator.getWaypoints(), referenceCoMGenerator.getCoMPositionDesiredInitialList(),
                                         referenceCoMGenerator.getCoMPositionDesiredFinalList(), referenceCoMGenerator.getCoMVelocityDesiredInitialList(),
                                         referenceCoMGenerator.getCoMVelocityDesiredFinalList(), referenceCoMGenerator.getCoMAccelerationDesiredInitialList(),
                                         referenceCoMGenerator.getCoMAccelerationDesiredFinalList(), referenceCoPGenerator.getNumberOfFootstepsRegistered());

      angularMomentumTrajectoryGenerator.computeReferenceAngularMomentumStartingFromDoubleSupport(isInitialTransfer.getValue(), isStanding.getValue());
      angularMomentumTrajectoryGenerator.initializeForDoubleSupport(ZERO_TIME, isStanding.getBooleanValue());

      if (isInitialTransfer.getValue() && isStanding.getValue())
      {
         referenceCMPGenerator
               .initializeForTransfer(ZERO_TIME, referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories(), null,
                                      null);
      }
      else
      {
         referenceCMPGenerator
               .initializeForTransfer(ZERO_TIME, referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories(),
                                      angularMomentumTrajectoryGenerator.getTransferAngularMomentumTrajectories(),
                                      angularMomentumTrajectoryGenerator.getSwingAngularMomentumTrajectories());
      }
      referenceICPGenerator
            .initializeForTransfer(ZERO_TIME, referenceCMPGenerator.getTransferCMPTrajectories(), referenceCMPGenerator.getSwingCMPTrajectories());

      referenceCoMGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceCoMGenerator
            .computeTrajectoryStartingFromTransfer(referenceCMPGenerator.getTransferCMPTrajectories(), referenceCMPGenerator.getSwingCMPTrajectories(),
                                                   referenceICPGenerator.getICPPositionDesiredFinalList());
      referenceICPGenerator.getICPPhaseEntryCornerPoints(icpPhaseEntryCornerPoints);
      referenceICPGenerator.getICPPhaseExitCornerPoints(icpPhaseExitCornerPoints);
      updateListeners();
   }

   protected void updateSingleSupportPlan(boolean maintainContinuity)
   {
      updateCount();

      clearPlanWithoutClearingPlannedFootsteps();
      RobotSide supportSide = this.supportSide.getEnumValue();

      boolean goingToPerformSmoothingAdjustment = maintainContinuity && adjustPlanForSSContinuity.getBooleanValue();
      referenceCoPGenerator.setGoingToPerformSSSmoothingAdjustment(goingToPerformSmoothingAdjustment);

      // TODO set up the CoP Generator to be able to only update the current Support Feet CMPs
      referenceCoPGenerator.computeReferenceCoPsStartingFromSingleSupport(supportSide);
      referenceCMPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceICPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());

      referenceICPGenerator.initializeForSwingFromCoPs(referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories());
      if (goingToPerformSmoothingAdjustment)
         referenceICPGenerator.adjustDesiredTrajectoriesForInitialSmoothing();

      referenceCoMGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());

      if (angularMomentumTrajectoryGenerator.isPredictingAngularMomentum())
      {
         referenceCoMGenerator
               .computeTrajectoryStartingFromSingleSupport(referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories(),
                                                           referenceICPGenerator.getICPPositonFromCoPDesiredFinalList());
      }

      angularMomentumTrajectoryGenerator
            .addCopAndComSetpointsToPlan(referenceCoPGenerator.getWaypoints(), referenceCoMGenerator.getCoMPositionDesiredInitialList(),
                                         referenceCoMGenerator.getCoMPositionDesiredFinalList(), referenceCoMGenerator.getCoMVelocityDesiredInitialList(),
                                         referenceCoMGenerator.getCoMVelocityDesiredFinalList(), referenceCoMGenerator.getCoMAccelerationDesiredInitialList(),
                                         referenceCoMGenerator.getCoMAccelerationDesiredFinalList(), referenceCoPGenerator.getNumberOfFootstepsRegistered());
      angularMomentumTrajectoryGenerator.computeReferenceAngularMomentumStartingFromSingleSupport();
      angularMomentumTrajectoryGenerator.initializeForSingleSupport(ZERO_TIME);

      referenceCMPGenerator.initializeForSwing(ZERO_TIME, referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories(),
                                               angularMomentumTrajectoryGenerator.getTransferAngularMomentumTrajectories(),
                                               angularMomentumTrajectoryGenerator.getSwingAngularMomentumTrajectories());
      referenceICPGenerator.initializeForSwing(ZERO_TIME, referenceCMPGenerator.getTransferCMPTrajectories(), referenceCMPGenerator.getSwingCMPTrajectories());

      referenceCoMGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceCoMGenerator
            .computeTrajectoryStartingFromSingleSupport(referenceCMPGenerator.getTransferCMPTrajectories(), referenceCMPGenerator.getSwingCMPTrajectories(),
                                                        referenceICPGenerator.getICPPositionDesiredFinalList());

      referenceICPGenerator.getICPPhaseEntryCornerPoints(icpPhaseEntryCornerPoints);
      referenceICPGenerator.getICPPhaseExitCornerPoints(icpPhaseExitCornerPoints);

      singleSupportInitialICP.set(referenceICPGenerator.getICPPositionDesiredInitialList().get(0));
      singleSupportFinalICP.set(referenceICPGenerator.getICPPositionDesiredFinalList().get(1));

      referenceCMPGenerator.getSwingCMPTrajectories().get(0).getEntryCMPLocation(tempPoint);
      CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), 0.0, singleSupportInitialICP, tempPoint, singleSupportInitialICPVelocity);
      referenceCMPGenerator.getSwingCMPTrajectories().get(0).getExitCMPLocation(tempPoint);
      CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), 0.0, singleSupportFinalICP, tempPoint, singleSupportFinalICPVelocity);

      updateListeners();
   }

   /** {@inheritDoc} */
   @Override
   public void compute(double time)
   {
      timer.startMeasurement();

      if (isInStanding() && doContinuousReplanningForStanding.getBooleanValue())
         updateTransferPlan(true);
      else if (isInDoubleSupport() && !isInStanding() && doContinuousReplanningForTransfer.getBooleanValue())
         updateTransferPlan(true);
      else if (!isInDoubleSupport() && doContinuousReplanningForSwing.getBooleanValue())
         updateSingleSupportPlan(true);

      if (referenceCoPGenerator.getIsPlanAvailable())
      {
         timeInCurrentState.set(time - initialTime.getDoubleValue());
         timeInCurrentStateRemaining.set(getCurrentStateDuration() - timeInCurrentState.getDoubleValue());

         double timeInCurrentState = MathTools.clamp(this.timeInCurrentState.getDoubleValue(), 0.0, referenceCoPGenerator.getCurrentStateFinalTime());

         referenceICPGenerator.compute(timeInCurrentState);
         referenceCoMGenerator.compute(timeInCurrentState);
         referenceCoPGenerator.update(timeInCurrentState);
         referenceCMPGenerator.update(timeInCurrentState);

         referenceCoPGenerator.getDesiredCenterOfPressure(desiredCoPPosition, desiredCoPVelocity);
         referenceCMPGenerator.getLinearData(desiredCMPPosition, desiredCMPVelocity);
         referenceICPGenerator.getLinearData(desiredICPPosition, desiredICPVelocity, desiredICPAcceleration);
         referenceCoMGenerator.getLinearData(desiredCoMPosition, desiredCoMVelocity, desiredCoMAcceleration);

         if (isInitialTransfer.getValue() && isStanding.getValue())
         {
            desiredCentroidalAngularMomentum.setToZero();
            desiredCentroidalTorque.setToZero();
         }
         else
         {
            angularMomentumTrajectoryGenerator.update(this.timeInCurrentState.getDoubleValue());
            angularMomentumTrajectoryGenerator.getDesiredAngularMomentum(desiredCentroidalAngularMomentum, desiredCentroidalTorque);
         }

         decayDesiredVelocityIfNeeded();

         if (debug)
            checkCoMDynamics(desiredCoMVelocity, desiredICPPosition, desiredCoMPosition);

         // done to account for the delayed velocity
         //computeDesiredCentroidalMomentumPivot(desiredICPPosition, desiredICPVelocity, omega0.getDoubleValue(), desiredCMPPosition);
         //computeDesiredCentroidalMomentumPivotVelocity(desiredICPVelocity, desiredICPAcceleration, omega0.getDoubleValue(), desiredCMPVelocity);

      }
      else
      {
         referenceCoPGenerator.getDoubleSupportPolygonCentroid(desiredCoPPosition);
         desiredCoPVelocity.setToZero();
         desiredCMPPosition.set(desiredCoPPosition);
         desiredCMPVelocity.setToZero();
         desiredICPPosition.set(desiredCoPPosition);
         desiredICPVelocity.setToZero();
         desiredICPAcceleration.setToZero();
         desiredCoMPosition.set(desiredCoPPosition);
         desiredCoMVelocity.setToZero();
         desiredCoMAcceleration.setToZero();
      }

      timer.stopMeasurement();
   }

   public void updateListeners()
   {
      referenceCoPGenerator.updateListeners();

      for (int i = 0; i < visualizationUpdatables.size(); i++)
      {
         visualizationUpdatables.get(i).getRight().setMatchingFrame(visualizationUpdatables.get(i).getLeft());
      }
   }

   private final FramePoint3D tempFinalICP = new FramePoint3D();

   /** {@inheritDoc} */
   @Override
   public void getFinalDesiredCapturePointPosition(FramePoint3D finalDesiredCapturePointPositionToPack)
   {
      if (isStanding.getBooleanValue())
      {
         referenceCoPGenerator.getFinalCoPLocation(finalDesiredCapturePointPositionToPack);
      }
      else if (!isInDoubleSupport())
      {
         List<? extends FramePoint3DReadOnly> finalDesiredICPPositions = referenceICPGenerator.getICPPositionDesiredFinalList();
         tempFinalICP.set(finalDesiredICPPositions.get(referenceCMPGenerator.getSwingCMPTrajectories().get(0).getNumberOfSegments() - 1));
         tempFinalICP.changeFrame(finalDesiredCapturePointPositionToPack.getReferenceFrame());
      }
      else
      {
         List<? extends FramePoint3DReadOnly> finalDesiredICPPositions = referenceICPGenerator.getICPPositionDesiredFinalList();
         tempFinalICP.set(finalDesiredICPPositions.get(referenceCMPGenerator.getTransferCMPTrajectories().get(0).getNumberOfSegments() - 1));
         tempFinalICP.changeFrame(finalDesiredCapturePointPositionToPack.getReferenceFrame());
         finalDesiredCapturePointPositionToPack.set(tempFinalICP);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void getFinalDesiredCapturePointPosition(YoFramePoint2D finalDesiredCapturePointPositionToPack)
   {
      getFinalDesiredCapturePointPosition(tempFinalICP);
      finalDesiredCapturePointPositionToPack.set(tempFinalICP);
   }

   private final FramePoint3D tempFinalCoM = new FramePoint3D();

   /** {@inheritDoc} */
   @Override
   public void getFinalDesiredCenterOfMassPosition(FramePoint3D finalDesiredCenterOfMassPositionToPack)
   {
      if (isStanding.getBooleanValue())
      {
         referenceCoPGenerator.getWaypoints().get(1).get(0).getPosition(tempFinalCoM);
      }
      else
      {
         tempFinalCoM.set(singleSupportFinalCoM);
      }

      tempFinalCoM.changeFrame(worldFrame);
      finalDesiredCenterOfMassPositionToPack.setIncludingFrame(tempFinalCoM);

   }

   /** {@inheritDoc} */
   @Override
   public void getNextExitCMP(FramePoint3D exitCMPToPack)
   {
      List<CoPPointsInFoot> plannedCoPWaypoints = referenceCoPGenerator.getWaypoints();
      CoPPointsInFoot copPointsInFoot = plannedCoPWaypoints.get(1);
      copPointsInFoot.get(copPointsInFoot.getNumberOfCoPPoints() - 1).getPosition(exitCMPToPack);
   }

   /** {@inheritDoc} */
   @Override
   public boolean isOnExitCMP()
   {
      return referenceCoPGenerator.isOnExitCoP();
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfFootstepsToConsider()
   {
      return numberFootstepsToConsider.getIntegerValue();
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfFootstepsRegistered()
   {
      return referenceCoPGenerator.getNumberOfFootstepsRegistered();
   }

   /** {@inheritDoc} */
   @Override
   public void holdCurrentICP(FramePoint3D icpPositionToHold)
   {
      this.icpPositionToHold.set(icpPositionToHold);
      requestedHoldPosition.set(true);
      // Asking the CoP and ICP to be the same since we assume that holds can be requested in static conditions only
      referenceCoPGenerator.holdPosition(icpPositionToHold);
   }

   private final FramePoint3D comVelocityDynamicsCurrent = new FramePoint3D();

   private void checkCoMDynamics(FrameVector3DReadOnly comVelocityDesiredCurrent, FramePoint3DReadOnly icpPositionDesiredCurrent,
                                 FramePoint3DReadOnly comPositionDesiredCurrent)
   {
      comVelocityDynamicsCurrent.setIncludingFrame(icpPositionDesiredCurrent);
      comVelocityDynamicsCurrent.sub(comPositionDesiredCurrent);
      comVelocityDynamicsCurrent.scale(omega0.getDoubleValue());

      areCoMDynamicsSatisfied.set(comVelocityDesiredCurrent.epsilonEquals(comVelocityDynamicsCurrent, 10e-5));
   }

   public void setDefaultPhaseTimes(double defaultSwingTime, double defaultTransferTime)
   {
      referenceCoPGenerator.setDefaultPhaseTimes(defaultSwingTime, defaultTransferTime);
   }

   public void getDesiredCenterOfMassVelocity(YoFrameVector3D desiredCenterOfMassVelocityToPack)
   {
      desiredCenterOfMassVelocityToPack.set(desiredCoMVelocity);
   }

   public void getDesiredCenterOfMassAcceleration(YoFrameVector3D desiredCenterOfMassAccelerationToPack)
   {
      desiredCenterOfMassAccelerationToPack.set(desiredCoMVelocity);
   }

   public void getDesiredCenterOfPressurePosition(YoFramePoint3D desiredCenterOfPressurePositionToPack)
   {
      desiredCenterOfPressurePositionToPack.setMatchingFrame(desiredCoPPosition);
   }


   /** {@inheritDoc} */
   @Override
   public void getDesiredCenterOfPressurePosition(FramePoint3D desiredCenterOfPressurePositionToPack)
   {
      desiredCenterOfPressurePositionToPack.setIncludingFrame(desiredCoPPosition);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCenterOfPressurePosition(FramePoint2D desiredCenterOfPressurePositionToPack)
   {
      desiredCenterOfPressurePositionToPack.setIncludingFrame(desiredCoPPosition);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCenterOfPressureVelocity(FrameVector3D desiredCenterOfPressureVelocityToPack)
   {
      desiredCenterOfPressureVelocityToPack.setIncludingFrame(desiredCoPVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public void getDesiredCenterOfPressureVelocity(FrameVector2D desiredCenterOfPressureVelocityToPack)
   {
      desiredCenterOfPressureVelocityToPack.setIncludingFrame(desiredCoPVelocity);
   }

   void ensureContinuityEnteringEachTransfer(boolean ensureContinuity)
   {
      this.adjustPlanForDSContinuity.set(ensureContinuity);
   }

   // getters for tests and visualizers

   public ReferenceCoPTrajectoryGenerator getReferenceCoPGenerator()
   {
      return referenceCoPGenerator;
   }

   ReferenceCMPTrajectoryGenerator getReferenceCMPGenerator()
   {
      return referenceCMPGenerator;
   }

   public ReferenceICPTrajectoryGenerator getReferenceICPGenerator()
   {
      return referenceICPGenerator;
   }

   public ReferenceCoMTrajectoryGenerator getReferenceCoMGenerator()
   {
      return referenceCoMGenerator;
   }

   AngularMomentumTrajectoryMultiplexer getAngularMomentumTrajectoryGenerator()
   {
      return angularMomentumTrajectoryGenerator;
   }

   private static final boolean printTracesIfComputedSeveralTimes = false;
   private int icpComputeCount = 0;
   private final YoInteger icpPlannerComputeCount = new YoInteger("ICPPlannnerComputeCount", registry);
   private final List<Throwable> traces = printTracesIfComputedSeveralTimes ? new ArrayList<>() : null;

   public void endTick()
   {
      if (icpComputeCount > 1)
      {
         LogTools.error("Computed " + getClass().getSimpleName() + " " + icpComputeCount + " times!");

         if (printTracesIfComputedSeveralTimes)
         {
            traces.forEach(t -> t.printStackTrace());
            System.err.println("");
         }
      }

      icpPlannerComputeCount.set(icpComputeCount);
      icpComputeCount = 0;

      if (printTracesIfComputedSeveralTimes)
      {
         traces.clear();
      }
   }

   private void updateCount()
   {
      icpComputeCount++;

      if (printTracesIfComputedSeveralTimes)
      {
         traces.add(new Throwable());
      }
   }
}
