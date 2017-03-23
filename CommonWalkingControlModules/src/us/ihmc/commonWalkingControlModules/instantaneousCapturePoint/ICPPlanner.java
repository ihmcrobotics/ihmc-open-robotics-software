package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools.*;
import static us.ihmc.commonWalkingControlModules.dynamicReachability.CoMIntegrationTools.*;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
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
import us.ihmc.robotics.math.frames.*;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Implementation of the ICP (Instantaneous Capture Point) planners introduced by Johannes
 * Englsberger:
 * <ol>
 * <li>Three-Dimensional Bipedal Walking Control Based on Divergent Component of Motion, IEEE
 * Transactions on Robotics, 2015.
 * <li>Three-Dimensional Bipedal Walking Control Using Divergent Component of Motion, IEEE/RSJ
 * International Conference on Intelligent Robots and Systems, 2013.
 * <li>Trajectory generation for continuous leg forces during double support and heel-to-toe shift
 * based on divergent component of motion, IEEE/RSJ International Conference on Intelligent Robots
 * and Systems, 2014.
 * </ol>
 * 
 * To summarize, the idea is to compute a smooth ICP trajectory for a given set of upcoming
 * footsteps and a set of desired constant CMP (Centroidal Moment Pivot) locations for each support.
 * <p>
 * Note that this ICP planner does not guarantee that the resulting CMP will always be inside the
 * support polygon. Indeed the ICP trajectory is generated using splines without additional
 * constraints on CMP. Usually the CMP will remain inside the support polygon, however, for certain
 * combinations of transfer and swing durations, the generated ICP trajectory might result in CMP
 * overshooting outside the support polygon. This is mostly happens when transfer and swing
 * durations are on very different scales, i.e. one is very small while the other is very large.
 * </p>
 * <p>
 * Note on the reference frames used in the planner. The overall ICP trajectory is a piecewise set
 * of splines representing portions of the trajectory for each phase of the gait cycle (single
 * support, double support). Each of these trajectories starts from initial conditions expressed in
 * a reference frame relevant to the initial support context and ends at final conditions expressed
 * in a reference frame relevant to the final support situation. By doing so, foot slippage is
 * inherently handle by this ICP planner as when a foot is slipping, the active trajectory is
 * immediately stretched such that the initial and final conditions are following the foot they are
 * attached to.
 * </p>
 * <p>
 * How to use the ICP planner.
 * <ul>
 * <li>First clear the current plan with {@link #clearPlan()}.
 * <li>Register the footsteps to be used in the plan with
 * {@link #addFootstepToPlan(Footstep, FootstepTiming)}.
 * <li>Set the duration to be used for the last transfer phase, i.e. once the last footstep has been
 * reached, using {@link #setFinalTransferDuration(double)}.
 * <li>Then several options depending on the current phase of the walking gait:
 * <ul>
 * <li>The robot is simply standing: simply call {@link #initializeForStanding(double)} providing
 * the current controller time.
 * <li>The robot is about to perform a transfer of its weight from one foot to the other. You first
 * need to provide the information about which foot the robot is transferring to via
 * {@link #setTransferToSide(RobotSide)} or transferring from via
 * {@link #setTransferFromSide(RobotSide)}. Then call {@link #initializeForTransfer(double)}
 * providing the current controller time.
 * <li>The robot is about to perform a single support phase. You first need to provide the
 * information about the foot that will be used a the support foot via
 * {@link #setSupportLeg(RobotSide)}. Then call {@link #initializeForSingleSupport(double)}
 * providing the current controller time.
 * </ul>
 * <li>The state of the ICP plan can then computed via {@link #compute(double)} given the current
 * controller time.
 * <li>The output of the planner can be accessed using the various following getters:
 * <ul>
 * <li>To get the ICP position, use either {@link #getDesiredCapturePointPosition(FramePoint)},
 * {@link #getDesiredCapturePointPosition(FramePoint2d)}, or
 * {@link #getDesiredCapturePointPosition(YoFramePoint)}.
 * <li>To get the ICP velocity, use either {@link #getDesiredCapturePointVelocity(FrameVector)},
 * {@link #getDesiredCapturePointVelocity(FrameVector2d)}, or
 * {@link #getDesiredCapturePointVelocity(YoFrameVector)}.
 * <li>To get the CoM position, use either {@link #getDesiredCenterOfMassPosition(FramePoint)},
 * {@link #getDesiredCenterOfMassPosition(FramePoint2d)}, or
 * {@link #getDesiredCenterOfMassPosition(YoFramePoint2d)}.
 * <li>To get the CMP position, use either
 * {@link #getDesiredCentroidalMomentumPivotPosition(FramePoint)}, or
 * {@link #getDesiredCentroidalMomentumPivotPosition(FramePoint2d)}.
 * <li>To get the CMP velocity, use either
 * {@link #getDesiredCentroidalMomentumPivotVelocity(FrameVector)}, or
 * {@link #getDesiredCentroidalMomentumPivotVelocity(FrameVector2d)}.
 * </ul>
 * </ul>
 * </p>
 *
 */
public class ICPPlanner
{
   /** Whether to display by default the various artifacts for debug or not. */
   private static final boolean VISUALIZE = false;
   /** Visualization parameter. */
   private static final double ICP_CORNER_POINT_SIZE = 0.004;

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
   private final DoubleYoVariable swingDurationAlpha = new DoubleYoVariable(namePrefix + "SwingDurationAlpha",
                                                                            "Repartition of the swing duration around the exit corner point.", registry);

   /**
    * Repartition of the transfer duration around the entry corner point:
    * <ul>
    * <li>{@code alpha * transferDuration} is spent with the ICP located before the entry corner
    * point.
    * <li>{@code (1.0 - alpha) * transferDuration} is spent with the ICP located after the entry
    * corner point.
    * </ul>
    */
   private final DoubleYoVariable transferDurationAlpha = new DoubleYoVariable(namePrefix + "TransferDurationAlpha",
                                                                               "Repartition of the transfer duration around the entry corner point.", registry);

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
   private final DoubleYoVariable finalTransferDuration = new DoubleYoVariable(namePrefix + "FinalTransferDuration", registry);

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
   public ICPPlanner(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
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
      transferDurationAlpha.set(icpPlannerParameters.getTransferDurationAlpha());
      swingDurationAlpha.set(icpPlannerParameters.getSwingDurationAlpha());
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

   /**
    * Clear footstep and timing information making the ICP planner ready to be reinitialized with
    * new footsteps.
    * <p>
    * Don't forget to call this method before registering a new set of footsteps.
    * </p>
    */
   public void clearPlan()
   {
      referenceCMPsCalculator.clear();

      for (int i = 0; i < swingDurations.size(); i++)
      {
         swingDurations.get(i).setToNaN();
         transferDurations.get(i).setToNaN();
      }
   }

   /**
    * Registers the side of the support leg.
    * <p>
    * This is required before initializing the planner for a single support phase.
    * </p>
    * 
    * @param robotSide the side of the support leg.
    */
   public void setSupportLeg(RobotSide robotSide)
   {
      supportSide.set(robotSide);
   }

   /**
    * Registers the side to which the robot is about to transfer its weight.
    * <p>
    * This is required before initializing the planner for a transfer phase.
    * </p>
    * 
    * @param robotSide the side towards which the robot is about to transfer.
    */
   public void setTransferToSide(RobotSide robotSide)
   {
      transferToSide.set(robotSide);
   }

   /**
    * Registers the side from which the robot is about to transfer its weight.
    * <p>
    * This is equivalent to: {@code setTransferToSide(robotSide.getOppositeSide())}.
    * </p>
    * <p>
    * This is required before initializing the planner for a transfer phase.
    * </p>
    * 
    * @param robotSide the side from which the robot is about to transfer.
    */
   public void setTransferFromSide(RobotSide robotSide)
   {
      if (robotSide != null)
         transferToSide.set(robotSide.getOppositeSide());
   }

   /**
    * Registers an additional footstep to consider in the next plan.
    * <p>
    * Footsteps have to be registered before initializing the planner.
    * </p>
    * <p>
    * The reference to {@code footstep} is saved internally. The active ICP plan can be modified by
    * updating a footstep and then calling the method {@link #updateCurrentPlan()}.
    * </p>
    * 
    * @param footstep the new footstep to be queued to the current list of footsteps. Not modified.
    * @param timing the timings to use when performing the footstep. Not modified.
    */
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      if (footstep == null)
         return;

      referenceCMPsCalculator.addUpcomingFootstep(footstep);
      int footstepIndex = referenceCMPsCalculator.getNumberOfFootstepRegistered() - 1;
      swingDurations.get(footstepIndex).set(timing.getSwingTime());
      transferDurations.get(footstepIndex).set(timing.getTransferTime());
   }


   /**
    * Prepares the planner to hold the given ICP position {@code icpPositionToHold} during the
    * double support phase.
    * <p>
    * This is usually useful for dealing with unexpected switch to double support where centering
    * the ICP in the support polygon would be undesirable.
    * </p>
    * 
    * @param icpPositionToHold the position at which the ICP will be held during the next double
    *           support phase. Not modified.
    */
   public void holdCurrentICP(FramePoint icpPositionToHold)
   {
      this.icpPositionToHold.set(icpPositionToHold);
      requestedHoldPosition.set(true);
   }

   /**
    * Initializes the planner to smoothly re-center the ICP in the support polygon preparing the
    * robot for standing.
    * <p>
    * This method is typically useful when done with a walking sequence so the robot smoothly
    * terminates its last transfer.
    * </p>
    * <p>
    * Call {@link #setFinalTransferDuration(double)} beforehand to change the time taken to
    * re-center the ICP.
    * </p>
    * 
    * @param initialTime typically refers to the current controller time. Marks the initial phase
    *           time for the planner.
    */
   public void initializeForStanding(double initialTime)
   {
      clearPlan();
      isStanding.set(true);
      isDoubleSupport.set(true);
      this.initialTime.set(initialTime);
      transferDurations.get(0).set(finalTransferDuration.getDoubleValue());
      updateTransferPlan();
   }

   /**
    * Prepares the ICP planner for a transfer phase.
    * <p>
    * Make sure that footsteps have been registered using
    * {@link #addFootstepToPlan(Footstep, FootstepTiming)} and that the transfer side has been
    * registered using {@link #setTransferToSide(RobotSide)} before calling this method.
    * </p>
    * 
    * @param initialTime typically refers to the current controller time. Marks the initial phase
    *           time for the planner.
    */
   public void initializeForTransfer(double initialTime)
   {
      isDoubleSupport.set(true);
      this.initialTime.set(initialTime);

      int numberOfFootstepRegistered = referenceCMPsCalculator.getNumberOfFootstepRegistered();
      if (numberOfFootstepRegistered < numberFootstepsToConsider.getIntegerValue())
         transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());

      updateTransferPlan();
   }

   private void updateTransferPlan()
   {
      RobotSide transferToSide = this.transferToSide.getEnumValue();
      if (transferToSide == null)
         transferToSide = RobotSide.LEFT;
      RobotSide transferFromSide = transferToSide.getOppositeSide();
      ReferenceFrame transferFromSoleFrame = soleZUpFrames.get(transferFromSide);
      ReferenceFrame transferToSoleFrame = soleZUpFrames.get(transferToSide);

      icpSingleSupportTrajectoryGenerator.hideVisualization();

      referenceCMPsCalculator.setUseTwoCMPsPerSupport(useTwoConstantCMPsPerSupport.getBooleanValue());
      referenceCMPsCalculator.computeReferenceCMPsStartingFromDoubleSupport(isStanding.getBooleanValue(), transferToSide);
      referenceCMPsCalculator.update();
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
         double transferAlpha = transferDurationAlpha.getDoubleValue();
         double swingAlpha = swingDurationAlpha.getDoubleValue();
         double transferDurationAfterEntryCornerPoint = transferDuration * (1.0 - transferAlpha);

         double omega0 = this.omega0.getDoubleValue();

         if (useTwoConstantCMPsPerSupport.getBooleanValue())
         {
            computeDesiredCornerPointsDoubleSupport(entryCornerPoints, exitCornerPoints, entryCMPs, exitCMPs, swingDurations, transferDurations, swingAlpha,
                                                    transferAlpha, omega0);

            double swingDurationOnExitCMP = swingDuration * (1.0 - swingAlpha);
            computeDesiredCapturePointPosition(omega0, swingDurationOnExitCMP, exitCornerPoints.get(1), exitCMPs.get(1), singleSupportFinalICP);
            computeDesiredCapturePointVelocity(omega0, swingDurationOnExitCMP, exitCornerPoints.get(1), exitCMPs.get(1), singleSupportFinalICPVelocity);
            exitCornerPoints.get(0).changeFrame(initialFrame);
            exitCornerPoints.get(1).changeFrame(finalFrame);
         }
         else
         {
            computeDesiredCornerPointsDoubleSupport(entryCornerPoints, entryCMPs, swingDurations, transferDurations, transferAlpha, omega0);
            double timeToNextCornerPoint = transferDurationAfterEntryCornerPoint + swingDuration;
            computeDesiredCapturePointPosition(omega0, timeToNextCornerPoint, entryCornerPoints.get(1), entryCMPs.get(1), singleSupportFinalICP);
            computeDesiredCapturePointVelocity(omega0, timeToNextCornerPoint, exitCornerPoints.get(1), exitCMPs.get(1), singleSupportFinalICPVelocity);
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

      if (!isStanding.getBooleanValue())
         computeFinalCoMPositionInTransfer();
   }

   private void computeFinalCoMPositionInTransfer()
   {
      icpDoubleSupportTrajectoryGenerator.computeFinalCoMPosition(singleSupportInitialCoM);
      yoSingleSupportInitialCoM.set(singleSupportInitialCoM);

      double swingDuration = swingDurations.get(0).getDoubleValue();
      if (useTwoConstantCMPsPerSupport.getBooleanValue())
      {
         double swingAlpha = swingDurationAlpha.getDoubleValue();
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
      yoSingleSupportFinalCoM.set(singleSupportFinalCoM);
   }

   /**
    * Prepares the ICP planner for a single support phase.
    * <p>
    * Make sure that footsteps have been registered using
    * {@link #addFootstepToPlan(Footstep, FootstepTiming)} and that the support side has been
    * registered using {@link #setSupportLeg(RobotSide)} before calling this method.
    * </p>
    * 
    * @param initialTime typically refers to the current controller time. Marks the initial phase
    *           time for the planner.
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
         transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());

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
      List<YoFramePoint> entryCMPs = referenceCMPsCalculator.getEntryCMPs();
      List<YoFramePoint> exitCMPs = referenceCMPsCalculator.getExitCMPs();

      double swingDuration = swingDurations.get(0).getDoubleValue();
      double transferAlpha = transferDurationAlpha.getDoubleValue();
      double swingAlpha = swingDurationAlpha.getDoubleValue();
      double transferDurationAfterEntryCornerPoint = transferDurations.get(0).getDoubleValue() * (1.0 - transferAlpha);
      double timeRemainingOnEntryCMP = swingDuration * swingAlpha;
      double timeToSpendOnExitCMPBeforeDoubleSupport = swingDuration * (1.0 - swingAlpha);

      switchCornerPointsToWorldFrame();
      singleSupportInitialICP.switchCurrentReferenceFrame(worldFrame);
      singleSupportFinalICP.switchCurrentReferenceFrame(worldFrame);
      yoSingleSupportInitialCoM.getFrameTuple2d(singleSupportInitialCoM);

      ReferenceFrame supportSoleFrame = soleZUpFrames.get(supportSide);
      double omega0 = this.omega0.getDoubleValue();
      if (useTwoConstantCMPsPerSupport.getBooleanValue())
      {
         computeDesiredCornerPointsSingleSupport(entryCornerPoints, exitCornerPoints, entryCMPs, exitCMPs, swingDurations, transferDurations, swingAlpha,
                                                 transferAlpha, omega0);
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
         computeDesiredCornerPointsSingleSupport(entryCornerPoints, entryCMPs, swingDurations, transferDurations, transferAlpha, omega0);
         double tInitial = transferDurationAfterEntryCornerPoint;
         double tFinal = tInitial + swingDuration;
         computeDesiredCapturePointPosition(omega0, tInitial, entryCornerPoints.get(0), entryCMPs.get(0), singleSupportInitialICP);
         computeDesiredCapturePointPosition(omega0, tFinal, entryCornerPoints.get(0), entryCMPs.get(0), singleSupportFinalICP);
      }

      computeFinalCoMPositionInSwing();

      singleSupportInitialICP.changeFrame(supportSoleFrame);
      entryCornerPoints.get(0).changeFrame(supportSoleFrame);
      singleSupportFinalICP.changeFrame(worldFrame);
      changeFrameOfRemainingCornerPoints(1, worldFrame);
   }

   private void computeFinalCoMPositionInSwing()
   {
      if (useTwoConstantCMPsPerSupport.getBooleanValue())
      {
         icpSingleSupportTrajectoryGenerator.setInitialCoMPosition(singleSupportInitialCoM, worldFrame);
         icpSingleSupportTrajectoryGenerator.computeFinalCoMPosition(singleSupportFinalCoM);
      }
      else
      {
         double swingDuration = swingDurations.get(0).getDoubleValue();
         List<YoFramePoint> entryCMPs = referenceCMPsCalculator.getEntryCMPs();
         singleSupportInitialICP.changeFrame(worldFrame);
         integrateCoMPositionUsingConstantCMP(swingDuration, omega0.getDoubleValue(), entryCMPs.get(0), singleSupportInitialICP, singleSupportInitialCoM,
               singleSupportFinalCoM);
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

   /**
    * Reinitializes the current plan without changing its initial time.
    * <p>
    * This is typically useful for updating the ICP plan with any change in contact state, i.e. on
    * foot switched to toe-off or the support polygon has been resized.
    * </p>
    * <p>
    * It can also be used to update the ICP plan when one of the registered footstep has been
    * modified from the outside. i.e. when dealing with push recovery via step adjustment.
    * </p>
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

   /**
    * Given the location of the actual ICP {@code actualCapturePointPosition}, this method estimates
    * the duration before the capture point reaches its desired location at foot touchdown.
    * <p>
    * Note this method is to be used when in single support and assumes that the internal state of
    * the planner is up-to-date, i.e. {@link #compute(double)} has been called in the current
    * control tick.
    * 
    * @param actualCapturePointPosition the current position of the measured ICP. Not modified.
    * @return the estimated time remaining before the capture point reaches its desired position at
    *         the end of this state.
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

   /**
    * Updates the current state of the ICP plan.
    * <p>
    * The ICP planner has to be initialized before calling this method.
    * </p>
    * <p>
    * The ICP planner has to be updated before accessing its outputs.
    * </p>
    * 
    * @param time the current controller time.
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

   /**
    * Gets the current ICP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    * 
    * @param desiredCapturePointPositionToPack the current ICP position. Modified.
    */
   public void getDesiredCapturePointPosition(FramePoint desiredCapturePointPositionToPack)
   {
      desiredICPPosition.getFrameTupleIncludingFrame(desiredCapturePointPositionToPack);
   }

   /**
    * Gets the current ICP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    * 
    * @param desiredCapturePointPositionToPack the current ICP position. Modified.
    */
   public void getDesiredCapturePointPosition(FramePoint2d desiredCapturePointPositionToPack)
   {
      desiredICPPosition.getFrameTuple2dIncludingFrame(desiredCapturePointPositionToPack);
   }

   /**
    * Gets the current ICP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    * 
    * @param desiredCapturePointPositionToPack the current ICP position. Modified.
    */
   public void getDesiredCapturePointPosition(YoFramePoint desiredCapturePointPositionToPack)
   {
      desiredCapturePointPositionToPack.set(desiredICPPosition);
   }

   /**
    * Gets the current CoM position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCenterOfMassPositionToPack the current CoM position. Modified.
    */
   public void getDesiredCenterOfMassPosition(FramePoint desiredCenterOfMassPositionToPack)
   {
      desiredCoMPosition.getFrameTupleIncludingFrame(desiredCenterOfMassPositionToPack);
   }

   /**
    * Gets the current CoM position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCenterOfMassPositionToPack the current CoM position. Modified.
    */
   public void getDesiredCenterOfMassPosition(FramePoint2d desiredCenterOfMassPositionToPack)
   {
      desiredCoMPosition.getFrameTuple2dIncludingFrame(desiredCenterOfMassPositionToPack);
   }

   /**
    * Gets the current CoM position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    *
    * @param desiredCenterOfMassPositionToPack the current CoM position. Modified.
    */
   public void getDesiredCenterOfMassPosition(YoFramePoint2d desiredCenterOfMassPositionToPack)
   {
      desiredCenterOfMassPositionToPack.set(desiredCoMPosition);
   }
   /**
    * Gets the current ICP velocity.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    * 
    * @param desiredCapturePointVelocityToPack the current ICP velocity. Modified.
    */
   public void getDesiredCapturePointVelocity(FrameVector desiredCapturePointVelocityToPack)
   {
      desiredICPVelocity.getFrameTupleIncludingFrame(desiredCapturePointVelocityToPack);
   }

   /**
    * Gets the current ICP velocity.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    * 
    * @param desiredCapturePointVelocityToPack the current ICP velocity. Modified.
    */
   public void getDesiredCapturePointVelocity(FrameVector2d desiredCapturePointVelocityToPack)
   {
      desiredICPVelocity.getFrameTuple2dIncludingFrame(desiredCapturePointVelocityToPack);
   }

   /**
    * Gets the current ICP velocity.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    * 
    * @param desiredCapturePointVelocityToPack the current ICP velocity. Modified.
    */
   public void getDesiredCapturePointVelocity(YoFrameVector desiredCapturePointVelocityToPack)
   {
      desiredCapturePointVelocityToPack.set(desiredICPVelocity);
   }

   /**
    * Gets the current CMP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    * 
    * @param desiredCentroidalMomentumPivotPositionToPack the current CMP position. Modified.
    */
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCMPPosition.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   /**
    * Gets the current CMP position.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    * 
    * @param desiredCentroidalMomentumPivotPositionToPack the current CMP position. Modified.
    */
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint2d desiredCentroidalMomentumPivotPositionToPack)
   {
      desiredCMPPosition.getFrameTuple2dIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   /**
    * Gets the current CMP velocity.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    * 
    * @param desiredCentroidalMomentumPivotVelocityToPack the current CMP velocity. Modified.
    */
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCMPVelocity.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotVelocityToPack);
   }

   /**
    * Gets the current CMP velocity.
    * <p>
    * The ICP planner has to be updated every control tick using the method
    * {@link #compute(double)}.
    * </p>
    * 
    * @param desiredCentroidalMomentumPivotVelocityToPack the current CMP velocity. Modified.
    */
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector2d desiredCentroidalMomentumPivotVelocityToPack)
   {
      desiredCMPVelocity.getFrameTuple2dIncludingFrame(desiredCentroidalMomentumPivotVelocityToPack);
   }

   /**
    * Gets the time relative to the beginning of the current state.
    * 
    * @return the time spent in the current state.
    */
   public double getTimeInCurrentState()
   {
      return timeInCurrentState.getDoubleValue();
   }

   /**
    * Gets the time remaining before the end of the current state.
    * 
    * @return the time remaining.
    */
   public double getTimeInCurrentStateRemaining()
   {
      return timeInCurrentStateRemaining.getDoubleValue();
   }

   /**
    * Gets the current state overall duration.
    * 
    * @return the current state duration.
    */
   public double getCurrentStateDuration()
   {
      if (isDoubleSupport.getBooleanValue())
         return getCurrentTransferDuration();
      else
         return getCurrentSwingDuration();
   }

   /**
    * Gets the duration of the current transfer state.
    *
    * @return the current transfer duration
    */
   public double getCurrentTransferDuration()
   {
      return transferDurations.get(0).getDoubleValue();
   }

   /**
    * Gets the duration of the current swing state.
    *
    * @return the current swing duration
    */
   public double getCurrentSwingDuration()
   {
      return swingDurations.get(0).getDoubleValue();
   }

   /**
    * Changes the duration for the current transfer duration.
    *
    * @param duration
    */
   public void setCurrentTransferDuration(double duration)
   {
      transferDurations.get(0).set(duration);
   }

   /**
    * Changes the duration for the current swing duration.
    *
    * @param duration
    */
   public void setCurrentSwingDuration(double duration)
   {
      swingDurations.get(0).set(duration);
   }

   /**
    * Changes the duration for the last transfer when going to standing state.
    * <p>
    * This method mostly affects {@link #initializeForStanding(double)}.
    * </p>
    * 
    * @param duration
    */
   public void setFinalTransferDuration(double duration)
   {
      finalTransferDuration.set(duration);
   }

   /**
    * Gets the time at which the swing phase is initalized
    *
    * @return initialTime
    */
   public double getInitialTime()
   {
      return initialTime.getDoubleValue();
   }

   /**
    * Intrinsic robot parameter.
    * <p>
    * Correspond the natural frequency response of the robot when modeled as an inverted pendulum:
    * {@code omega0 = Math.sqrt(g / z0)}, where {@code g} is equal to the magnitude of the gravity,
    * and {@code z0} is the constant center of mass height of the robot with respect to is feet.
    * </p>
    * 
    * @param omega0 the robot's natural frequency.
    */
   public void setOmega0(double omega0)
   {
      this.omega0.set(omega0);
   }

   /**
    * Returns whether the ICP planner currently assumes to that the robot is in double support.
    * 
    * @return whether the ICP plan is in double support state or not.
    */
   public boolean isInDoubleSupport()
   {
      return isDoubleSupport.getBooleanValue();
   }

   /**
    * Returns whether the ICP planner currently assumes to that the robot is standing.
    * 
    * @return whether the ICP plan is in standing state or not.
    */
   public boolean isInStanding()
   {
      return isStanding.getBooleanValue();
   }

   /**
    * Returns whether the ICP planner currently assumes to that the robot is performing the first
    * transfer of a walking sequence, i.e. just left standing state.
    * 
    * @return whether the ICP plan is in initial transfer state or not.
    */
   public boolean isInInitialTranfer()
   {
      return isInitialTransfer.getBooleanValue();
   }

   private final FramePoint tempFinalICP = new FramePoint();

   /**
    * Retrieves the desired ICP position at the end of the current state.
    * 
    * @param finalDesiredCapturePointPositionToPack the final desired ICP position. Modified.
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

   /**
    * Retrieves the desired ICP position at the end of the current state.
    * 
    * @param finalDesiredCapturePointPositionToPack the final desired ICP position. Modified.
    */
   public void getFinalDesiredCapturePointPosition(YoFramePoint2d finalDesiredCapturePointPositionToPack)
   {
      if (isStanding.getBooleanValue())
         referenceCMPsCalculator.getNextEntryCMP(tempFinalICP);
      else
         entryCornerPoints.get(1).getFrameTupleIncludingFrame(tempFinalICP);
      tempFinalICP.changeFrame(finalDesiredCapturePointPositionToPack.getReferenceFrame());
      finalDesiredCapturePointPositionToPack.setByProjectionOntoXYPlane(tempFinalICP);
   }

   private final FramePoint2d tempFinalCoM = new FramePoint2d();

   /**
    * Retrieves the desired CoM position at the end of the current step.
    *
    * @param finalDesiredCenterOfMassPositionToPack the final desired ICP position. Modified.
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

   /**
    * Retrieves the position of the next exit CMP.
    * <p>
    * This is typically useful to estimate where the robot will put its center of pressure at the
    * end of single support.
    * </p>
    * 
    * @param entryCMPToPack the next exit CMP position. Modified.
    */
   public void getNextExitCMP(FramePoint entryCMPToPack)
   {
      referenceCMPsCalculator.getNextExitCMP(entryCMPToPack);
   }

   /**
    * Tests if the ICP planner is done with the current state.
    * 
    * @return {@code true} if the plan for the current state is done, returns {@code false}
    *         otherwise.
    */
   public boolean isDone()
   {
      return timeInCurrentStateRemaining.getDoubleValue() <= 0.0;
   }

   /**
    * Tests the current state in the ICP plan results in having the desired CMP located at the exit
    * CMP.
    * 
    * @return {@code true} if the current CMP is located on the exit CMP, returns {@code false}
    *         otherwise.
    */
   public boolean isOnExitCMP()
   {
      if (isDoubleSupport.getBooleanValue())
         return false;
      else
         return icpSingleSupportTrajectoryGenerator.isOnExitCMP();
   }
}
