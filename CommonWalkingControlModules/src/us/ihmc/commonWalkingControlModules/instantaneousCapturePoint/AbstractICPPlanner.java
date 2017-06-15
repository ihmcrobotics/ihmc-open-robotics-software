package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static us.ihmc.commonWalkingControlModules.dynamicReachability.CoMIntegrationTools.integrateCoMPositionUsingConstantCMP;
import static us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools.computeDesiredCapturePointAcceleration;
import static us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools.computeDesiredCapturePointPosition;
import static us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools.computeDesiredCapturePointVelocity;
import static us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools.computeDesiredCentroidalMomentumPivot;
import static us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools.computeDesiredCentroidalMomentumPivotVelocity;
import static us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools.computeDesiredCornerPointsDoubleSupport;
import static us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools.computeDesiredCornerPointsSingleSupport;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public abstract class AbstractICPPlanner
{
   /** Whether to display by default the various artifacts for debug or not. */
   private static final boolean VISUALIZE = false;
   /** Visualization parameter. */
   private static final double ICP_CORNER_POINT_SIZE = 0.008;

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String namePrefix = "icpPlanner";

   private final BooleanYoVariable isStanding = new BooleanYoVariable(namePrefix + "IsStanding", registry);
   private final BooleanYoVariable isInitialTransfer = new BooleanYoVariable(namePrefix + "IsInitialTransfer", registry);
   private final BooleanYoVariable isDoubleSupport = new BooleanYoVariable(namePrefix + "IsDoubleSupport", registry);

   /////////////////////////////// Start Planner Output ///////////////////////////////

   /** Desired position for the Instantaneous Capture Point (ICP) */
   private final YoFramePoint desiredICPPosition = new YoFramePoint(namePrefix + "DesiredCapturePointPosition", worldFrame, registry);
   /** Desired velocity for the Instantaneous Capture Point (ICP) */
   private final YoFrameVector desiredICPVelocity = new YoFrameVector(namePrefix + "DesiredCapturePointVelocity", worldFrame, registry);
   /** Desired acceleration for the Instantaneous Capture Point (ICP) */
   private final YoFrameVector desiredICPAcceleration = new YoFrameVector(namePrefix + "DesiredCapturePointAcceleration", worldFrame, registry);
   /** Desired position for the Centroidal Momentum Pivot (CMP) */
   private final YoFramePoint desiredCMPPosition = new YoFramePoint(namePrefix + "DesiredCentroidalMomentumPosition", worldFrame, registry);
   /** Desired velocity for the Centroidal Momentum Pivot (CMP) */
   private final YoFrameVector desiredCMPVelocity = new YoFrameVector(namePrefix + "DesiredCentroidalMomentumVelocity", worldFrame, registry);
   /** Desired position for the Center of Mass (CoM) */
   private final YoFramePoint2d desiredCoMPosition = new YoFramePoint2d(namePrefix + "DesiredCoMPosition", worldFrame, registry);

   //////////////////////////////// End Planner Output ////////////////////////////////

   private final DoubleYoVariable omega0 = new DoubleYoVariable(namePrefix + "Omega0", registry);

   /** Time at which the current state was initialized. */
   private final DoubleYoVariable initialTime = new DoubleYoVariable(namePrefix + "CurrentStateInitialTime", registry);
   /** Time spent in the current state. */
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable(namePrefix + "TimeInCurrentState", registry);
   /** Time remaining before the end of the current state. */
   private final DoubleYoVariable timeInCurrentStateRemaining = new DoubleYoVariable(namePrefix + "RemainingTime", registry);

   private final BooleanYoVariable useTwoConstantCMPsPerSupport = new BooleanYoVariable(namePrefix + "UseTwoConstantCMPsPerSupport", registry);

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
   private final DoubleYoVariable defaultSwingDurationAlpha = new DoubleYoVariable(namePrefix + "DefaultSwingDurationAlpha",
                                                                            "Repartition of the swing duration around the exit corner point.", registry);
   private final ArrayList<DoubleYoVariable> swingDurationAlphas = new ArrayList<>();

   /**
    * Repartition of the transfer duration around the entry corner point:
    * <ul>
    * <li>{@code alpha * transferDuration} is spent with the ICP located before the entry corner
    * point.
    * <li>{@code (1.0 - alpha) * transferDuration} is spent with the ICP located after the entry
    * corner point.
    * </ul>
    */
   private final DoubleYoVariable defaultTransferDurationAlpha = new DoubleYoVariable(namePrefix + "DefaultTransferDurationAlpha",
                                                                               "Repartition of the transfer duration around the entry corner point.", registry);
   private final ArrayList<DoubleYoVariable> transferDurationAlphas = new ArrayList<>();

   private final IntegerYoVariable numberFootstepsToConsider = new IntegerYoVariable(namePrefix + "NumberFootstepsToConsider", registry);

   /**
    * Duration parameter used to linearly decrease the desired ICP velocity once the current state
    * is done.
    * <p>
    * This reduction in desired ICP velocity is particularly useful to reduce the CIP tracking error
    * when the robot is getting stuck at the end of transfer.
    * </p>
    */
   private final DoubleYoVariable velocityDecayDurationWhenDone = new DoubleYoVariable(namePrefix + "VelocityDecayDurationWhenDone", registry);
   /**
    * Output of the linear reduction being applied on the desired ICP velocity when the current
    * state is done.
    * <p>
    * This reduction in desired ICP velocity is particularly useful to reduce the ICP tracking error
    * when the robot is getting stuck at the end of transfer.
    * </p>
    */
   private final DoubleYoVariable velocityReductionFactor = new DoubleYoVariable(namePrefix + "VelocityReductionFactor", registry);

   private final YoFramePointInMultipleFrames singleSupportInitialICP;
   private final YoFrameVector singleSupportInitialICPVelocity = new YoFrameVector(namePrefix + "SingleSupportInitialICPVelocity", worldFrame, registry);

   private final YoFramePointInMultipleFrames singleSupportFinalICP;
   private final YoFrameVector singleSupportFinalICPVelocity = new YoFrameVector(namePrefix + "SingleSupportFinalICPVelocity", worldFrame, registry);

   private final YoFramePoint2d yoSingleSupportInitialCoM;
   private final YoFramePoint2d yoSingleSupportFinalCoM;
   private final FramePoint2d singleSupportInitialCoM = new FramePoint2d();
   private final FramePoint2d singleSupportFinalCoM = new FramePoint2d();

   private final BooleanYoVariable requestedHoldPosition = new BooleanYoVariable(namePrefix + "RequestedHoldPosition", registry);
   private final BooleanYoVariable isHoldingPosition = new BooleanYoVariable(namePrefix + "IsHoldingPosition", registry);
   private final YoFramePoint icpPositionToHold = new YoFramePoint(namePrefix + "CapturePointPositionToHold", worldFrame, registry);

   private final EnumYoVariable<RobotSide> transferToSide = new EnumYoVariable<>(namePrefix + "TransferToSide", registry, RobotSide.class, true);
   private final EnumYoVariable<RobotSide> supportSide = new EnumYoVariable<>(namePrefix + "SupportSide", registry, RobotSide.class, true);

   private final List<YoFramePointInMultipleFrames> entryCornerPoints = new ArrayList<>();
   private final List<YoFramePointInMultipleFrames> exitCornerPoints = new ArrayList<>();

   private final List<DoubleYoVariable> swingDurations = new ArrayList<>();
   private final List<DoubleYoVariable> transferDurations = new ArrayList<>();
   private final DoubleYoVariable defaultFinalTransferDuration = new DoubleYoVariable(namePrefix + "DefaultFinalTransferDuration", registry);
   private final DoubleYoVariable finalTransferDuration = new DoubleYoVariable(namePrefix + "FinalTransferDuration", registry);
   private final DoubleYoVariable finalTransferDurationAlpha = new DoubleYoVariable(namePrefix + "FinalTransferDurationAlpha", registry);

   private final ICPPlannerTrajectoryGenerator icpDoubleSupportTrajectoryGenerator;
   private final ICPPlannerSegmentedTrajectoryGenerator icpSingleSupportTrajectoryGenerator;
   private final ReferenceCentroidalMomentumPivotLocationsCalculator referenceCMPsCalculator;

   private final ReferenceFrame midFeetZUpFrame;
   private final SideDependentList<ReferenceFrame> soleZUpFrames;

   private final FramePoint tempConstantCMP = new FramePoint();
   private final FramePoint tempICP = new FramePoint();
   private final FramePoint2d tempCoM = new FramePoint2d();

   /**
    * Creates an ICP planner. Refer to the class documentation: {@link ICPPlanner}.
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
   public AbstractICPPlanner(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                     CapturePointPlannerParameters icpPlannerParameters, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      isStanding.set(true);

      finalTransferDuration.setToNaN();

      icpPositionToHold.setToNaN();
      isHoldingPosition.set(false);

      icpDoubleSupportTrajectoryGenerator = new ICPPlannerTrajectoryGenerator(namePrefix + "DoubleSupport", worldFrame, omega0, registry);
      icpSingleSupportTrajectoryGenerator = new ICPPlannerSegmentedTrajectoryGenerator(namePrefix + "SingleSupport", worldFrame, omega0, registry);
      icpSingleSupportTrajectoryGenerator.setMaximumSplineDuration(icpPlannerParameters.getMaxDurationForSmoothingEntryToExitCMPSwitch());
      icpSingleSupportTrajectoryGenerator.setMinimumTimeToSpendOnFinalCMP(icpPlannerParameters.getMinTimeToSpendOnExitCMPInSingleSupport());

      numberFootstepsToConsider.set(icpPlannerParameters.getNumberOfFootstepsToConsider());
      defaultTransferDurationAlpha.set(icpPlannerParameters.getTransferDurationAlpha());
      defaultSwingDurationAlpha.set(icpPlannerParameters.getSwingDurationAlpha());
      finalTransferDurationAlpha.set(icpPlannerParameters.getTransferDurationAlpha());
      useTwoConstantCMPsPerSupport.set(icpPlannerParameters.useTwoCMPsPerSupport());

      velocityDecayDurationWhenDone.set(icpPlannerParameters.getVelocityDecayDurationWhenDone());
      velocityReductionFactor.set(Double.NaN);

      // Initialize omega0 to NaN to force the user to explicitly set it.
      omega0.set(Double.NaN);

      referenceCMPsCalculator = new ReferenceCentroidalMomentumPivotLocationsCalculator(namePrefix, bipedSupportPolygons, contactableFeet,
                                                                                        numberFootstepsToConsider.getIntegerValue(), registry);
      referenceCMPsCalculator.initializeParameters(icpPlannerParameters);

      midFeetZUpFrame = bipedSupportPolygons.getMidFeetZUpFrame();
      soleZUpFrames = bipedSupportPolygons.getSoleZUpFrames();

      ReferenceFrame[] framesToRegister = new ReferenceFrame[] {worldFrame, midFeetZUpFrame, soleZUpFrames.get(RobotSide.LEFT),
            soleZUpFrames.get(RobotSide.RIGHT)};
      singleSupportInitialICP = new YoFramePointInMultipleFrames(namePrefix + "SingleSupportInitialICP", registry, framesToRegister);
      singleSupportFinalICP = new YoFramePointInMultipleFrames(namePrefix + "SingleSupportFinalICP", registry, framesToRegister);

      yoSingleSupportInitialCoM = new YoFramePoint2d(namePrefix + "SingleSupportInitialCoM", worldFrame, registry);
      yoSingleSupportFinalCoM = new YoFramePoint2d(namePrefix + "SingleSupportFinalCoM", worldFrame, registry);

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue() - 1; i++)
      {
         YoFramePointInMultipleFrames entryCornerPoint = new YoFramePointInMultipleFrames(namePrefix + "EntryCornerPoints" + i, registry, framesToRegister);
         entryCornerPoints.add(entryCornerPoint);

         YoFramePointInMultipleFrames exitCornerPoint = new YoFramePointInMultipleFrames(namePrefix + "ExitCornerPoints" + i, registry, framesToRegister);
         exitCornerPoints.add(exitCornerPoint);
      }

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue(); i++)
      {
         DoubleYoVariable swingDuration = new DoubleYoVariable(namePrefix + "SwingDuration" + i, registry);
         swingDuration.setToNaN();
         swingDurations.add(swingDuration);
         DoubleYoVariable transferDuration = new DoubleYoVariable(namePrefix + "TransferDuration" + i, registry);
         transferDuration.setToNaN();
         transferDurations.add(transferDuration);

         DoubleYoVariable transferDurationAlpha = new DoubleYoVariable(namePrefix + "TransferDurationAlpha" + i,
               "Repartition of the transfer duration around the entry corner point.", registry);
         transferDurationAlpha.setToNaN();
         transferDurationAlphas.add(transferDurationAlpha);
         DoubleYoVariable swingDurationAlpha = new DoubleYoVariable(namePrefix + "SwingDurationAlpha" + i,
               "Repartition of the transfer duration around the entry corner point.", registry);
         swingDurationAlpha.setToNaN();
         swingDurationAlphas.add(swingDurationAlpha);
      }

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry != null)
      {
         setupVisualizers(yoGraphicsListRegistry);
      }
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      YoGraphicsList yoGraphicsList = new YoGraphicsList(getClass().getSimpleName());
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      referenceCMPsCalculator.createVisualizerForConstantCMPs(yoGraphicsList, artifactList);

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue() - 1; i++)
      {
         YoFramePoint entryCornerPointInWorld = entryCornerPoints.get(i).buildUpdatedYoFramePointForVisualizationOnly();
         YoGraphicPosition icpEarlyCornerPointsViz = new YoGraphicPosition("EntryCornerPoints" + i, entryCornerPointInWorld, ICP_CORNER_POINT_SIZE,
                                                                           YoAppearance.Blue(), GraphicType.SOLID_BALL);

         yoGraphicsList.add(icpEarlyCornerPointsViz);
         artifactList.add(icpEarlyCornerPointsViz.createArtifact());

         YoFramePoint exitCornerPointInWorld = exitCornerPoints.get(i).buildUpdatedYoFramePointForVisualizationOnly();
         YoGraphicPosition lateCornerPointsViz = new YoGraphicPosition("ExitCornerPoints" + i, exitCornerPointInWorld, ICP_CORNER_POINT_SIZE,
                                                                       YoAppearance.Blue(), GraphicType.BALL);

         yoGraphicsList.add(lateCornerPointsViz);
         artifactList.add(lateCornerPointsViz.createArtifact());
      }

      YoFramePoint initialICPInWorld = singleSupportInitialICP.buildUpdatedYoFramePointForVisualizationOnly();
      YoGraphicPosition singleSupportInitialICPViz = new YoGraphicPosition("singleSupportInitialICP", initialICPInWorld, 0.004, YoAppearance.Chocolate(),
                                                                           GraphicType.SOLID_BALL);
      yoGraphicsList.add(singleSupportInitialICPViz);
      artifactList.add(singleSupportInitialICPViz.createArtifact());

      YoFramePoint finalICPInWorld = singleSupportFinalICP.buildUpdatedYoFramePointForVisualizationOnly();
      YoGraphicPosition singleSupportFinalICPViz = new YoGraphicPosition("singleSupportFinalICP", finalICPInWorld, 0.004, YoAppearance.Chocolate(),
                                                                         GraphicType.BALL);
      yoGraphicsList.add(singleSupportFinalICPViz);
      artifactList.add(singleSupportFinalICPViz.createArtifact());

      YoGraphicPosition desiredCenterOfMassPositionViz = new YoGraphicPosition("desiredCoMLocation", desiredCoMPosition, 0.004, YoAppearance.YellowGreen(),
            GraphicType.BALL_WITH_CROSS);
      yoGraphicsList.add(desiredCenterOfMassPositionViz);
      artifactList.add(desiredCenterOfMassPositionViz.createArtifact());

      YoGraphicPosition singleSupportInitialCoMViz = new YoGraphicPosition("singleSupportInitialCoM", yoSingleSupportInitialCoM, 0.004, YoAppearance.Black(),
            GraphicType.SOLID_BALL);
      yoGraphicsList.add(singleSupportInitialCoMViz);
      artifactList.add(singleSupportInitialCoMViz.createArtifact());

      YoGraphicPosition singleSupportFinalCoMViz = new YoGraphicPosition("singleSupportFinalCoM", yoSingleSupportFinalCoM, 0.004, YoAppearance.Black(),
            GraphicType.BALL);
      yoGraphicsList.add(singleSupportFinalCoMViz);
      artifactList.add(singleSupportFinalCoMViz.createArtifact());


      icpSingleSupportTrajectoryGenerator.createVisualizers(yoGraphicsList, artifactList);

      artifactList.setVisible(VISUALIZE);
      yoGraphicsList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#clearPlan()
    */
   public void clearPlan()
   {
      referenceCMPsCalculator.clear();

      for (int i = 0; i < swingDurations.size(); i++)
      {
         swingDurations.get(i).setToNaN();
         transferDurations.get(i).setToNaN();
         swingDurationAlphas.get(i).setToNaN();
         transferDurationAlphas.get(i).setToNaN();
      }
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#setSupportLeg(us.ihmc.robotics.robotSide.RobotSide)
    */
   public void setSupportLeg(RobotSide robotSide)
   {
      supportSide.set(robotSide);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#setTransferToSide(us.ihmc.robotics.robotSide.RobotSide)
    */
   public void setTransferToSide(RobotSide robotSide)
   {
      transferToSide.set(robotSide);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#setTransferFromSide(us.ihmc.robotics.robotSide.RobotSide)
    */
   public void setTransferFromSide(RobotSide robotSide)
   {
      if (robotSide != null)
         transferToSide.set(robotSide.getOppositeSide());
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#addFootstepToPlan(us.ihmc.humanoidRobotics.footstep.Footstep, us.ihmc.humanoidRobotics.footstep.FootstepTiming)
    */
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      if (footstep == null)
         return;

      referenceCMPsCalculator.addUpcomingFootstep(footstep);
      int footstepIndex = referenceCMPsCalculator.getNumberOfFootstepRegistered() - 1;
      swingDurations.get(footstepIndex).set(timing.getSwingTime());
      transferDurations.get(footstepIndex).set(timing.getTransferTime());

      swingDurationAlphas.get(footstepIndex).set(defaultSwingDurationAlpha.getDoubleValue());
      transferDurationAlphas.get(footstepIndex).set(defaultTransferDurationAlpha.getDoubleValue());

      finalTransferDuration.set(defaultFinalTransferDuration.getDoubleValue());
      finalTransferDurationAlpha.set(defaultTransferDurationAlpha.getDoubleValue());
   }


   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#holdCurrentICP(us.ihmc.robotics.geometry.FramePoint)
    */
   public void holdCurrentICP(FramePoint icpPositionToHold)
   {
      this.icpPositionToHold.set(icpPositionToHold);
      requestedHoldPosition.set(true);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#initializeForStanding(double)
    */
   public void initializeForStanding(double initialTime)
   {
      clearPlan();
      isStanding.set(true);
      isDoubleSupport.set(true);
      this.initialTime.set(initialTime);
      transferDurations.get(0).set(finalTransferDuration.getDoubleValue());
      transferDurationAlphas.get(0).set(finalTransferDurationAlpha.getDoubleValue());
      updateTransferPlan();
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#initializeForTransfer(double)
    */
   public void initializeForTransfer(double initialTime)
   {
      isDoubleSupport.set(true);
      this.initialTime.set(initialTime);

      int numberOfFootstepRegistered = referenceCMPsCalculator.getNumberOfFootstepRegistered();
      if (numberOfFootstepRegistered < numberFootstepsToConsider.getIntegerValue())
      {
         transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());
         transferDurationAlphas.get(numberOfFootstepRegistered).set(finalTransferDurationAlpha.getDoubleValue());
      }

      updateTransferPlan();
   }

   private void updateTransferPlan()
   {
      RobotSide transferToSide = this.transferToSide.getEnumValue();
      if (transferToSide == null)
         transferToSide = RobotSide.LEFT;

      icpSingleSupportTrajectoryGenerator.hideVisualization();

      referenceCMPsCalculator.setUseTwoCMPsPerSupport(useTwoConstantCMPsPerSupport.getBooleanValue());
      referenceCMPsCalculator.computeReferenceCMPsStartingFromDoubleSupport(isStanding.getBooleanValue(), transferToSide);
      referenceCMPsCalculator.update();

      initializeTransferTrajectory(transferToSide);

      if (!isStanding.getBooleanValue())
         computeFinalCoMPositionInTransferInternal();
   }

   private void initializeTransferTrajectory(RobotSide transferToSide)
   {
      RobotSide transferFromSide = transferToSide.getOppositeSide();
      ReferenceFrame transferFromSoleFrame = soleZUpFrames.get(transferFromSide);
      ReferenceFrame transferToSoleFrame = soleZUpFrames.get(transferToSide);

      List<YoFramePoint> entryCMPs = referenceCMPsCalculator.getEntryCMPs();
      List<YoFramePoint> exitCMPs = referenceCMPsCalculator.getExitCMPs();
      switchCornerPointsToWorldFrame();
      singleSupportInitialICP.switchCurrentReferenceFrame(worldFrame);
      singleSupportFinalICP.switchCurrentReferenceFrame(worldFrame);

      boolean isDoneWalking = referenceCMPsCalculator.isDoneWalking();

      ReferenceFrame initialFrame;

      if (isStanding.getBooleanValue())
      {
         initialFrame = midFeetZUpFrame;
      }
      else
      {
         tempICP.setToZero(midFeetZUpFrame);
         tempICP.changeFrame(worldFrame);
         double distanceFromDesiredICPToMidfeetZUpFrame = desiredICPPosition.getXYPlaneDistance(tempICP);
         tempICP.setToZero(transferFromSoleFrame);
         tempICP.changeFrame(worldFrame);
         double distanceFromDesiredICPToTransferFromSoleFrame = desiredICPPosition.getXYPlaneDistance(tempICP);

         if (distanceFromDesiredICPToMidfeetZUpFrame < distanceFromDesiredICPToTransferFromSoleFrame)
            initialFrame = midFeetZUpFrame;
         else
            initialFrame = transferFromSoleFrame;
      }
      ReferenceFrame finalFrame = isDoneWalking ? midFeetZUpFrame : transferToSoleFrame;

      double transferDuration = transferDurations.get(0).getDoubleValue();
      double swingDuration = swingDurations.get(0).getDoubleValue();

      if (requestedHoldPosition.getBooleanValue())
      {
         desiredICPPosition.set(icpPositionToHold);
         desiredICPVelocity.setToZero();
         desiredCoMPosition.set(icpPositionToHold.getX(), icpPositionToHold.getY());
         singleSupportInitialICP.setIncludingFrame(icpPositionToHold);
         singleSupportFinalICP.setIncludingFrame(icpPositionToHold);
         singleSupportInitialICPVelocity.set(0.0, 0.0, 0.0);
         setCornerPointsToNaN();
         icpPositionToHold.setToNaN();
         isDoneWalking = true;
         requestedHoldPosition.set(false);
         isHoldingPosition.set(true);
      }
      else if (isDoneWalking)
      {
         singleSupportInitialICP.setIncludingFrame(entryCMPs.get(0));
         singleSupportFinalICP.setIncludingFrame(singleSupportInitialICP);
         singleSupportInitialICPVelocity.set(0.0, 0.0, 0.0);
         setCornerPointsToNaN();
         isHoldingPosition.set(false);
      }
      else
      {
         double transferAlpha = transferDurationAlphas.get(0).getDoubleValue();
         double swingAlpha = swingDurationAlphas.get(0).getDoubleValue();
         double transferDurationAfterEntryCornerPoint = transferDuration * (1.0 - transferAlpha);

         double omega0 = this.omega0.getDoubleValue();

         if (useTwoConstantCMPsPerSupport.getBooleanValue())
         {
            computeDesiredCornerPointsDoubleSupport(entryCornerPoints, exitCornerPoints, entryCMPs, exitCMPs, swingDurations, transferDurations,
                  swingDurationAlphas, transferDurationAlphas, omega0);

            double swingDurationOnExitCMP = swingDuration * (1.0 - swingAlpha);
            computeDesiredCapturePointPosition(omega0, swingDurationOnExitCMP, exitCornerPoints.get(1), exitCMPs.get(1), singleSupportFinalICP);
            computeDesiredCapturePointVelocity(omega0, swingDurationOnExitCMP, exitCornerPoints.get(1), exitCMPs.get(1), singleSupportFinalICPVelocity);
            exitCornerPoints.get(0).changeFrame(initialFrame);
            exitCornerPoints.get(1).changeFrame(finalFrame);
         }
         else
         {
            computeDesiredCornerPointsDoubleSupport(entryCornerPoints, entryCMPs, swingDurations, transferDurations, transferDurationAlphas, omega0);
            double timeToNextCornerPoint = transferDurationAfterEntryCornerPoint + swingDuration;
            computeDesiredCapturePointPosition(omega0, timeToNextCornerPoint, entryCornerPoints.get(1), entryCMPs.get(1), singleSupportFinalICP);
            computeDesiredCapturePointVelocity(omega0, timeToNextCornerPoint, entryCornerPoints.get(1), entryCMPs.get(1), singleSupportFinalICPVelocity);
         }

         computeDesiredCapturePointPosition(omega0, transferDurationAfterEntryCornerPoint, entryCornerPoints.get(1), entryCMPs.get(1), singleSupportInitialICP);
         computeDesiredCapturePointVelocity(omega0, 0.0, singleSupportInitialICP, entryCMPs.get(1), singleSupportInitialICPVelocity);

         entryCornerPoints.get(0).changeFrame(initialFrame);
         entryCornerPoints.get(1).changeFrame(finalFrame);
         changeFrameOfRemainingCornerPoints(2, worldFrame);
         isHoldingPosition.set(false);
      }

      if (isStanding.getBooleanValue() && !isDoneWalking)
      {
         isInitialTransfer.set(true);
         isStanding.set(false);
      }

      singleSupportInitialICP.changeFrame(finalFrame);
      singleSupportFinalICP.changeFrame(worldFrame);

      icpDoubleSupportTrajectoryGenerator.setTrajectoryTime(transferDuration);
      icpDoubleSupportTrajectoryGenerator.setInitialConditions(desiredICPPosition, desiredICPVelocity, initialFrame);
      icpDoubleSupportTrajectoryGenerator.setFinalConditions(singleSupportInitialICP, singleSupportInitialICPVelocity, finalFrame);
      icpDoubleSupportTrajectoryGenerator.setInitialCoMPosition(desiredCoMPosition, worldFrame);
      icpDoubleSupportTrajectoryGenerator.initialize();
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#computeFinalCoMPositionInTransfer()
    */
   public void computeFinalCoMPositionInTransfer()
   {
      int numberOfFootstepRegistered = referenceCMPsCalculator.getNumberOfFootstepRegistered();
      if (numberOfFootstepRegistered < numberFootstepsToConsider.getIntegerValue())
      {
         transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());
         transferDurationAlphas.get(numberOfFootstepRegistered).set(finalTransferDurationAlpha.getDoubleValue());
      }

      RobotSide transferToSide = this.transferToSide.getEnumValue();
      if (transferToSide == null)
         transferToSide = RobotSide.LEFT;
      initializeTransferTrajectory(transferToSide);

      computeFinalCoMPositionInTransferInternal();
   }


   private void computeFinalCoMPositionInTransferInternal()
   {
      icpDoubleSupportTrajectoryGenerator.computeFinalCoMPosition(singleSupportInitialCoM);
      yoSingleSupportInitialCoM.set(singleSupportInitialCoM);

      double swingDuration = swingDurations.get(0).getDoubleValue();

      if (Double.isFinite(swingDuration))
      {
         if (useTwoConstantCMPsPerSupport.getBooleanValue())
         {
            double swingAlpha = swingDurationAlphas.get(0).getDoubleValue();
            double timeOnEntryDuringSwing = swingDuration * swingAlpha;
            double timeOnExitDuringSwing = swingDuration * (1.0 - swingAlpha);

            ReferenceFrame supportSoleFrame = soleZUpFrames.get(transferToSide.getEnumValue());

            icpSingleSupportTrajectoryGenerator.setBoundaryICP(singleSupportInitialICP, singleSupportFinalICP);
            icpSingleSupportTrajectoryGenerator.setCornerPoints(entryCornerPoints.get(1), exitCornerPoints.get(1));
            icpSingleSupportTrajectoryGenerator.setReferenceCMPs(referenceCMPsCalculator.getEntryCMPs().get(1), referenceCMPsCalculator.getExitCMPs().get(1));
            icpSingleSupportTrajectoryGenerator.setReferenceFrames(supportSoleFrame, worldFrame);
            icpSingleSupportTrajectoryGenerator.setInitialCoMPosition(singleSupportInitialCoM, worldFrame);
            icpSingleSupportTrajectoryGenerator.setTrajectoryTime(timeOnEntryDuringSwing, timeOnExitDuringSwing);
            icpSingleSupportTrajectoryGenerator.initialize();

            icpSingleSupportTrajectoryGenerator.setInitialCoMPosition(singleSupportInitialCoM, worldFrame);
            icpSingleSupportTrajectoryGenerator.computeFinalCoMPosition(singleSupportFinalCoM);
         }
         else
         {
            singleSupportInitialICP.changeFrame(worldFrame);
            integrateCoMPositionUsingConstantCMP(swingDuration, omega0.getDoubleValue(), referenceCMPsCalculator.getEntryCMPs().get(1), singleSupportInitialICP,
                  singleSupportInitialCoM, singleSupportFinalCoM);
         }
      }
      else
      {
         singleSupportFinalCoM.set(singleSupportInitialCoM);
      }
      yoSingleSupportFinalCoM.set(singleSupportFinalCoM);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#initializeForSingleSupport(double)
    */
   public void initializeForSingleSupport(double initialTime)
   {
      isHoldingPosition.set(false);

      isStanding.set(false);
      isInitialTransfer.set(false);
      isDoubleSupport.set(false);
      this.initialTime.set(initialTime);

      int numberOfFootstepRegistered = referenceCMPsCalculator.getNumberOfFootstepRegistered();
      if (numberOfFootstepRegistered < numberFootstepsToConsider.getIntegerValue())
      {
         transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());
         transferDurationAlphas.get(numberOfFootstepRegistered).set(finalTransferDurationAlpha.getDoubleValue());
      }

      yoSingleSupportInitialCoM.set(desiredCoMPosition);
      desiredCoMPosition.getFrameTuple2d(singleSupportInitialCoM);
      updateSingleSupportPlan();
   }

   private void updateSingleSupportPlan()
   {
      RobotSide supportSide = this.supportSide.getEnumValue();

      referenceCMPsCalculator.setUseTwoCMPsPerSupport(useTwoConstantCMPsPerSupport.getBooleanValue());
      referenceCMPsCalculator.computeReferenceCMPsStartingFromSingleSupport(supportSide);
      referenceCMPsCalculator.update();

      ReferenceFrame supportSoleFrame = initializeSwingTrajectory();

      computeFinalCoMPositionInSwingInternal();

      singleSupportInitialICP.changeFrame(supportSoleFrame);
      entryCornerPoints.get(0).changeFrame(supportSoleFrame);
      singleSupportFinalICP.changeFrame(worldFrame);
      changeFrameOfRemainingCornerPoints(1, worldFrame);
   }

   private ReferenceFrame initializeSwingTrajectory()
   {
      List<YoFramePoint> entryCMPs = referenceCMPsCalculator.getEntryCMPs();
      List<YoFramePoint> exitCMPs = referenceCMPsCalculator.getExitCMPs();

      double transferAlpha = transferDurationAlphas.get(0).getDoubleValue();
      double swingAlpha = swingDurationAlphas.get(0).getDoubleValue();
      double swingDuration = swingDurations.get(0).getDoubleValue();
      double transferDurationAfterEntryCornerPoint = transferDurations.get(0).getDoubleValue() * (1.0 - transferAlpha);
      double timeRemainingOnEntryCMP = swingDuration * swingAlpha;
      double timeToSpendOnExitCMPBeforeDoubleSupport = swingDuration * (1.0 - swingAlpha);

      switchCornerPointsToWorldFrame();
      singleSupportInitialICP.switchCurrentReferenceFrame(worldFrame);
      singleSupportFinalICP.switchCurrentReferenceFrame(worldFrame);
      yoSingleSupportInitialCoM.getFrameTuple2d(singleSupportInitialCoM);

      ReferenceFrame supportSoleFrame = soleZUpFrames.get(supportSide.getEnumValue());
      double omega0 = this.omega0.getDoubleValue();
      if (useTwoConstantCMPsPerSupport.getBooleanValue())
      {
         computeDesiredCornerPointsSingleSupport(entryCornerPoints, exitCornerPoints, entryCMPs, exitCMPs, swingDurations, transferDurations,
               swingDurationAlphas, transferDurationAlphas, omega0);
         computeDesiredCapturePointPosition(omega0, transferDurationAfterEntryCornerPoint, entryCornerPoints.get(0), entryCMPs.get(0), singleSupportInitialICP);
         computeDesiredCapturePointVelocity(omega0, transferDurationAfterEntryCornerPoint, entryCornerPoints.get(0), entryCMPs.get(0),
               singleSupportInitialICPVelocity);

         computeDesiredCapturePointPosition(omega0, timeToSpendOnExitCMPBeforeDoubleSupport, exitCornerPoints.get(0), exitCMPs.get(0), singleSupportFinalICP);
         computeDesiredCapturePointVelocity(omega0, timeToSpendOnExitCMPBeforeDoubleSupport, exitCornerPoints.get(0), exitCMPs.get(0),
               singleSupportFinalICPVelocity);

         icpSingleSupportTrajectoryGenerator.setBoundaryICP(singleSupportInitialICP, singleSupportFinalICP);
         icpSingleSupportTrajectoryGenerator.setCornerPoints(entryCornerPoints.get(0), exitCornerPoints.get(0));
         icpSingleSupportTrajectoryGenerator.setReferenceCMPs(entryCMPs.get(0), exitCMPs.get(0));
         icpSingleSupportTrajectoryGenerator.setReferenceFrames(supportSoleFrame, worldFrame);
         icpSingleSupportTrajectoryGenerator.setInitialCoMPosition(singleSupportInitialCoM, worldFrame);
         icpSingleSupportTrajectoryGenerator.setTrajectoryTime(timeRemainingOnEntryCMP, timeToSpendOnExitCMPBeforeDoubleSupport);
         icpSingleSupportTrajectoryGenerator.initialize();

         exitCornerPoints.get(0).changeFrame(supportSoleFrame);
      }
      else
      {
         computeDesiredCornerPointsSingleSupport(entryCornerPoints, entryCMPs, swingDurations, transferDurations, transferDurationAlphas, omega0);
         double tInitial = transferDurationAfterEntryCornerPoint;
         double tFinal = tInitial + swingDuration;
         computeDesiredCapturePointPosition(omega0, tInitial, entryCornerPoints.get(0), entryCMPs.get(0), singleSupportInitialICP);
         computeDesiredCapturePointPosition(omega0, tFinal, entryCornerPoints.get(0), entryCMPs.get(0), singleSupportFinalICP);
      }

      return supportSoleFrame;
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#computeFinalCoMPositionInSwing()
    */
   public void computeFinalCoMPositionInSwing()
   {
      ReferenceFrame supportSoleFrame = initializeSwingTrajectory();

      computeFinalCoMPositionInSwingInternal();

      singleSupportInitialICP.changeFrame(supportSoleFrame);
      entryCornerPoints.get(0).changeFrame(supportSoleFrame);
      singleSupportFinalICP.changeFrame(worldFrame);
      changeFrameOfRemainingCornerPoints(1, worldFrame);
   }

   private void computeFinalCoMPositionInSwingInternal()
   {
      double swingDuration = swingDurations.get(0).getDoubleValue();
      if (Double.isFinite(swingDuration))
      {
         if (useTwoConstantCMPsPerSupport.getBooleanValue())
         {
            icpSingleSupportTrajectoryGenerator.setInitialCoMPosition(singleSupportInitialCoM, worldFrame);
            icpSingleSupportTrajectoryGenerator.computeFinalCoMPosition(singleSupportFinalCoM);
         }
         else
         {
            List<YoFramePoint> entryCMPs = referenceCMPsCalculator.getEntryCMPs();
            singleSupportInitialICP.changeFrame(worldFrame);
            integrateCoMPositionUsingConstantCMP(swingDuration, omega0.getDoubleValue(), entryCMPs.get(0), singleSupportInitialICP, singleSupportInitialCoM,
                  singleSupportFinalCoM);
         }
      }
      else
      {
         singleSupportFinalCoM.set(singleSupportInitialCoM);
      }
      yoSingleSupportFinalCoM.set(singleSupportFinalCoM);
   }



   private void setCornerPointsToNaN()
   {
      for (int i = 0; i < entryCornerPoints.size(); i++)
         entryCornerPoints.get(i).setToNaN();
      for (int i = 0; i < exitCornerPoints.size(); i++)
         exitCornerPoints.get(i).setToNaN();
   }

   private void switchCornerPointsToWorldFrame()
   {
      for (int i = 0; i < entryCornerPoints.size(); i++)
         entryCornerPoints.get(i).switchCurrentReferenceFrame(worldFrame);
      for (int i = 0; i < exitCornerPoints.size(); i++)
         exitCornerPoints.get(i).switchCurrentReferenceFrame(worldFrame);
   }

   private void changeFrameOfRemainingCornerPoints(int fromIndex, ReferenceFrame desiredFrame)
   {
      for (int i = fromIndex; i < entryCornerPoints.size(); i++)
         entryCornerPoints.get(i).changeFrame(desiredFrame);
      for (int i = fromIndex; i < exitCornerPoints.size(); i++)
         exitCornerPoints.get(i).changeFrame(desiredFrame);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#updateCurrentPlan()
    */
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

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#estimateTimeRemainingForStateUnderDisturbance(us.ihmc.robotics.geometry.FramePoint2d)
    */
   public double estimateTimeRemainingForStateUnderDisturbance(FramePoint2d actualCapturePointPosition)
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

   private final FramePoint2d desiredICP2d = new FramePoint2d();
   private final FramePoint2d finalICP2d = new FramePoint2d();
   private final FrameLine2d desiredICPToFinalICPLine = new FrameLine2d();
   private final FrameLineSegment2d desiredICPToFinalICPLineSegment = new FrameLineSegment2d();
   private final FramePoint2d actualICP2d = new FramePoint2d();

   private double estimateDeltaTimeBetweenDesiredICPAndActualICP(FramePoint2d actualCapturePointPosition)
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

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#compute(double)
    */
   public void compute(double time)
   {
      timeInCurrentState.set(time - initialTime.getDoubleValue());
      timeInCurrentStateRemaining.set(getCurrentStateDuration() - timeInCurrentState.getDoubleValue());

      time = timeInCurrentState.getDoubleValue();

      update();
      referenceCMPsCalculator.update();

      double omega0 = this.omega0.getDoubleValue();

      if (isDoubleSupport.getBooleanValue())
      {
         icpDoubleSupportTrajectoryGenerator.compute(time);
         icpDoubleSupportTrajectoryGenerator.getLinearData(desiredICPPosition, desiredICPVelocity, desiredICPAcceleration);
         icpDoubleSupportTrajectoryGenerator.getCoMPosition(desiredCoMPosition);
      }
      else if (useTwoConstantCMPsPerSupport.getBooleanValue())
      {
         icpSingleSupportTrajectoryGenerator.compute(time);
         icpSingleSupportTrajectoryGenerator.getLinearData(desiredICPPosition, desiredICPVelocity, desiredICPAcceleration);
         icpSingleSupportTrajectoryGenerator.getCoMPosition(desiredCoMPosition);
      }
      else
      {
         referenceCMPsCalculator.getNextEntryCMP(tempConstantCMP);
         singleSupportInitialICP.getFrameTupleIncludingFrame(tempICP);
         yoSingleSupportInitialCoM.getFrameTuple2d(singleSupportInitialCoM);
         tempICP.changeFrame(worldFrame);
         double swingDuration = swingDurations.get(0).getDoubleValue();
         time = MathTools.clamp(time, 0.0, swingDuration);
         computeDesiredCapturePointPosition(omega0, time, tempICP, tempConstantCMP, desiredICPPosition);
         computeDesiredCapturePointVelocity(omega0, time, tempICP, tempConstantCMP, desiredICPVelocity);
         computeDesiredCapturePointAcceleration(omega0, time, tempICP, tempConstantCMP, desiredICPAcceleration);

         integrateCoMPositionUsingConstantCMP(0.0, time, omega0, tempConstantCMP, tempICP, singleSupportInitialCoM, tempCoM);
         desiredCoMPosition.set(tempCoM);
      }

      decayDesiredVelocityIfNeeded();

      computeDesiredCentroidalMomentumPivot(desiredICPPosition, desiredICPVelocity, omega0, desiredCMPPosition);
      computeDesiredCentroidalMomentumPivotVelocity(desiredICPVelocity, desiredICPAcceleration, omega0, desiredCMPVelocity);
   }

   private void update()
   {
      singleSupportInitialICP.notifyVariableChangedListeners();
      singleSupportFinalICP.notifyVariableChangedListeners();

      for (int i = 0; i < entryCornerPoints.size(); i++)
         entryCornerPoints.get(i).notifyVariableChangedListeners();
      for (int i = 0; i < exitCornerPoints.size(); i++)
         exitCornerPoints.get(i).notifyVariableChangedListeners();
   }

   private void decayDesiredVelocityIfNeeded()
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

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getDesiredCapturePointPosition(us.ihmc.robotics.geometry.FramePoint)
    */
   public void getDesiredCapturePointPosition(FramePoint desiredCapturePointPositionToPack)
   {
      desiredICPPosition.getFrameTupleIncludingFrame(desiredCapturePointPositionToPack);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getDesiredCapturePointPosition(us.ihmc.robotics.geometry.FramePoint2d)
    */
   public void getDesiredCapturePointPosition(FramePoint2d desiredCapturePointPositionToPack)
   {
      desiredICPPosition.getFrameTuple2dIncludingFrame(desiredCapturePointPositionToPack);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getDesiredCapturePointPosition(us.ihmc.robotics.math.frames.YoFramePoint)
    */
   public void getDesiredCapturePointPosition(YoFramePoint desiredCapturePointPositionToPack)
   {
      desiredCapturePointPositionToPack.set(desiredICPPosition);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getDesiredCenterOfMassPosition(us.ihmc.robotics.geometry.FramePoint)
    */
   public void getDesiredCenterOfMassPosition(FramePoint desiredCenterOfMassPositionToPack)
   {
      desiredCoMPosition.getFrameTupleIncludingFrame(desiredCenterOfMassPositionToPack);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getDesiredCenterOfMassPosition(us.ihmc.robotics.geometry.FramePoint2d)
    */
   public void getDesiredCenterOfMassPosition(FramePoint2d desiredCenterOfMassPositionToPack)
   {
      desiredCoMPosition.getFrameTuple2dIncludingFrame(desiredCenterOfMassPositionToPack);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getDesiredCenterOfMassPosition(us.ihmc.robotics.math.frames.YoFramePoint2d)
    */
   public void getDesiredCenterOfMassPosition(YoFramePoint2d desiredCenterOfMassPositionToPack)
   {
      desiredCenterOfMassPositionToPack.set(desiredCoMPosition);
   }
   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getDesiredCapturePointVelocity(us.ihmc.robotics.geometry.FrameVector)
    */
   public void getDesiredCapturePointVelocity(FrameVector desiredCapturePointVelocityToPack)
   {
      desiredICPVelocity.getFrameTupleIncludingFrame(desiredCapturePointVelocityToPack);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getDesiredCapturePointVelocity(us.ihmc.robotics.geometry.FrameVector2d)
    */
   public void getDesiredCapturePointVelocity(FrameVector2d desiredCapturePointVelocityToPack)
   {
      desiredICPVelocity.getFrameTuple2dIncludingFrame(desiredCapturePointVelocityToPack);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getDesiredCapturePointVelocity(us.ihmc.robotics.math.frames.YoFrameVector)
    */
   public void getDesiredCapturePointVelocity(YoFrameVector desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.set(desiredICPVelocity);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getDesiredCentroidalMomentumPivotPosition(us.ihmc.robotics.geometry.FramePoint)
    */
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCMPPosition.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getDesiredCentroidalMomentumPivotPosition(us.ihmc.robotics.geometry.FramePoint2d)
    */
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint2d desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCMPPosition.getFrameTuple2dIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getDesiredCentroidalMomentumPivotVelocity(us.ihmc.robotics.geometry.FrameVector)
    */
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCMPVelocity.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotVelocityToPack);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getDesiredCentroidalMomentumPivotVelocity(us.ihmc.robotics.geometry.FrameVector2d)
    */
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector2d desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCMPVelocity.getFrameTuple2dIncludingFrame(desiredCentroidalMomentumPivotVelocityToPack);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getTimeInCurrentState()
    */
   public double getTimeInCurrentState()
   {
      return timeInCurrentState.getDoubleValue();
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getTimeInCurrentStateRemaining()
    */
   public double getTimeInCurrentStateRemaining()
   {
      return timeInCurrentStateRemaining.getDoubleValue();
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getCurrentStateDuration()
    */
   public double getCurrentStateDuration()
   {
      if (isDoubleSupport.getBooleanValue())
         return getTransferDuration(0);
      else
         return getSwingDuration(0);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#setTransferDuration(int, double)
    */
   public void setTransferDuration(int stepNumber, double duration)
   {
      int numberOfFootstepRegistered = referenceCMPsCalculator.getNumberOfFootstepRegistered();
      if (stepNumber < numberOfFootstepRegistered + 1)
         transferDurations.get(stepNumber).set(duration);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#setSwingDuration(int, double)
    */
   public void setSwingDuration(int stepNumber, double duration)
   {
      int numberOfFootstepRegistered = referenceCMPsCalculator.getNumberOfFootstepRegistered();
      if (stepNumber < numberOfFootstepRegistered)
         swingDurations.get(stepNumber).set(duration);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getTransferDuration(int)
    */
   public double getTransferDuration(int stepNumber)
   {
      return transferDurations.get(stepNumber).getDoubleValue();
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getSwingDuration(int)
    */
   public double getSwingDuration(int stepNumber)
   {
      return swingDurations.get(stepNumber).getDoubleValue();
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#setFinalTransferDuration(double)
    */
   public void setFinalTransferDuration(double duration)
   {
      defaultFinalTransferDuration.set(duration);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#setFinalTransferDurationAlpha(double)
    */
   public void setFinalTransferDurationAlpha(double durationAlpha)
   {
      finalTransferDurationAlpha.set(durationAlpha);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#setTransferDurationAlpha(int, double)
    */
   public void setTransferDurationAlpha(int stepNumber, double transferDurationAlpha)
   {
      transferDurationAlphas.get(stepNumber).set(transferDurationAlpha);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#setSwingDurationAlpha(int, double)
    */
   public void setSwingDurationAlpha(int stepNumber, double swingDurationAlpha)
   {
      swingDurationAlphas.get(stepNumber).set(swingDurationAlpha);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getTransferDurationAlpha(int)
    */
   public double getTransferDurationAlpha(int stepNumber)
   {
      return transferDurationAlphas.get(stepNumber).getDoubleValue();
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getSwingDurationAlpha(int)
    */
   public double getSwingDurationAlpha(int stepNumber)
   {
      return swingDurationAlphas.get(stepNumber).getDoubleValue();
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getInitialTime()
    */
   public double getInitialTime()
   {
      return initialTime.getDoubleValue();
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#setOmega0(double)
    */
   public void setOmega0(double omega0)
   {
      this.omega0.set(omega0);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#isInDoubleSupport()
    */
   public boolean isInDoubleSupport()
   {
      return isDoubleSupport.getBooleanValue();
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#isInStanding()
    */
   public boolean isInStanding()
   {
      return isStanding.getBooleanValue();
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#isInInitialTranfer()
    */
   public boolean isInInitialTranfer()
   {
      return isInitialTransfer.getBooleanValue();
   }

   private final FramePoint tempFinalICP = new FramePoint();

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getFinalDesiredCapturePointPosition(us.ihmc.robotics.geometry.FramePoint)
    */
   public void getFinalDesiredCapturePointPosition(FramePoint finalDesiredCapturePointPositionToPack)
   {
      if (isStanding.getBooleanValue())
         referenceCMPsCalculator.getNextEntryCMP(tempFinalICP);
      else
         entryCornerPoints.get(1).getFrameTupleIncludingFrame(tempFinalICP);
      tempFinalICP.changeFrame(worldFrame);
      finalDesiredCapturePointPositionToPack.setIncludingFrame(tempFinalICP);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getFinalDesiredCapturePointPosition(us.ihmc.robotics.math.frames.YoFramePoint2d)
    */
   public void getFinalDesiredCapturePointPosition(YoFramePoint2d finalDesiredCapturePointPositionToPack)
   {
      if (isStanding.getBooleanValue())
      {
         referenceCMPsCalculator.getNextEntryCMP(tempFinalICP);
      }
      else if (entryCornerPoints.get(1).containsNaN())
      {
         tempFinalICP.setToZero(midFeetZUpFrame);
         tempFinalICP.changeFrame(finalDesiredCapturePointPositionToPack.getReferenceFrame());
         finalDesiredCapturePointPositionToPack.setByProjectionOntoXYPlane(tempFinalICP);
      }
      else
      {
         entryCornerPoints.get(1).getFrameTupleIncludingFrame(tempFinalICP);
         tempFinalICP.changeFrame(finalDesiredCapturePointPositionToPack.getReferenceFrame());
         finalDesiredCapturePointPositionToPack.setByProjectionOntoXYPlane(tempFinalICP);
      }
   }

   private final FramePoint2d tempFinalCoM = new FramePoint2d();

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getFinalDesiredCenterOfMassPosition(us.ihmc.robotics.geometry.FramePoint2d)
    */
   public void getFinalDesiredCenterOfMassPosition(FramePoint2d finalDesiredCenterOfMassPositionToPack)
   {
      if (isStanding.getBooleanValue())
      {
         referenceCMPsCalculator.getNextEntryCMP(tempFinalICP);
         tempFinalCoM.setByProjectionOntoXYPlane(tempFinalICP);
      }
      else
      {
         tempFinalCoM.set(singleSupportFinalCoM);
      }

      tempFinalCoM.changeFrame(worldFrame);
      finalDesiredCenterOfMassPositionToPack.setIncludingFrame(tempFinalCoM);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getNextExitCMP(us.ihmc.robotics.geometry.FramePoint)
    */
   public void getNextExitCMP(FramePoint entryCMPToPack)
   {
      referenceCMPsCalculator.getNextExitCMP(entryCMPToPack);
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#isDone()
    */
   public boolean isDone()
   {
      return timeInCurrentStateRemaining.getDoubleValue() <= 0.0;
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#isOnExitCMP()
    */
   public boolean isOnExitCMP()
   {
      if (isDoubleSupport.getBooleanValue())
         return false;
      else
         return icpSingleSupportTrajectoryGenerator.isOnExitCMP();
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getNumberOfFootstepsToConsider()
    */
   public int getNumberOfFootstepsToConsider()
   {
      return numberFootstepsToConsider.getIntegerValue();
   }

   /* (non-Javadoc)
    * @see us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPPlannerInterface#getNumberOfFootstepsRegistered()
    */
   public int getNumberOfFootstepsRegistered()
   {
      return referenceCMPsCalculator.getNumberOfFootstepRegistered();
   }
}
