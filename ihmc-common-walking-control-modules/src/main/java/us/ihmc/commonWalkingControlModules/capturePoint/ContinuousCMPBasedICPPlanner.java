package us.ihmc.commonWalkingControlModules.capturePoint;

import static us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools.computeDesiredCapturePointAcceleration;
import static us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools.computeDesiredCapturePointPosition;
import static us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools.computeDesiredCapturePointVelocity;
import static us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools.computeDesiredCentroidalMomentumPivot;
import static us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools.computeDesiredCentroidalMomentumPivotVelocity;
import static us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools.computeDesiredCornerPointsDoubleSupport;
import static us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools.computeDesiredCornerPointsSingleSupport;
import static us.ihmc.commonWalkingControlModules.dynamicReachability.CoMIntegrationTools.integrateCoMPositionUsingConstantCMP;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.ContinuousCMPICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

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
 * <li>To get the ICP position, use either {@link #getDesiredCapturePointPosition(FramePoint3D)},
 * {@link #getDesiredCapturePointPosition(FramePoint2D)}, or
 * {@link #getDesiredCapturePointPosition(YoFramePoint)}.
 * <li>To get the ICP velocity, use either {@link #getDesiredCapturePointVelocity(FrameVector)},
 * {@link #getDesiredCapturePointVelocity(FrameVector2d)}, or
 * {@link #getDesiredCapturePointVelocity(YoFrameVector)}.
 * <li>To get the CoM position, use either {@link #getDesiredCenterOfMassPosition(FramePoint3D)},
 * {@link #getDesiredCenterOfMassPosition(FramePoint2D)}, or
 * {@link #getDesiredCenterOfMassPosition(YoFramePoint2d)}.
 * <li>To get the CMP position, use either
 * {@link #getDesiredCentroidalMomentumPivotPosition(FramePoint3D)}, or
 * {@link #getDesiredCentroidalMomentumPivotPosition(FramePoint2D)}.
 * <li>To get the CMP velocity, use either
 * {@link #getDesiredCentroidalMomentumPivotVelocity(FrameVector)}, or
 * {@link #getDesiredCentroidalMomentumPivotVelocity(FrameVector2d)}.
 * </ul>
 * </ul>
 * </p>
 *
 */
public class ContinuousCMPBasedICPPlanner extends AbstractICPPlanner
{
   /** Whether to display by default the various artifacts for debug or not. */
   private static final boolean VISUALIZE = false;
   /** Visualization parameter. */
   private static final double ICP_CORNER_POINT_SIZE = 0.008;

   private final YoBoolean useTwoConstantCMPsPerSupport = new YoBoolean(namePrefix + "UseTwoConstantCMPsPerSupport", registry);

   private final YoFramePoint yoSingleSupportInitialCoM;
   private final YoFramePoint yoSingleSupportFinalCoM;
   private final FramePoint3D singleSupportInitialCoM = new FramePoint3D();
   private final FramePoint3D singleSupportFinalCoM = new FramePoint3D();

   private final List<YoFramePointInMultipleFrames> entryCornerPoints = new ArrayList<>();
   private final List<YoFramePointInMultipleFrames> exitCornerPoints = new ArrayList<>();

   private final ICPPlannerTrajectoryGenerator icpDoubleSupportTrajectoryGenerator;
   private final ICPPlannerSegmentedTrajectoryGenerator icpSingleSupportTrajectoryGenerator;
   private final ReferenceCentroidalMomentumPivotLocationsCalculator referenceCMPsCalculator;

   private final FramePoint3D tempConstantCMP = new FramePoint3D();
   private final FramePoint3D tempICP = new FramePoint3D();
   private final FramePoint3D tempCoM = new FramePoint3D();

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
   public ContinuousCMPBasedICPPlanner(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                       int numberOfFootstepsToConsider, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(bipedSupportPolygons, numberOfFootstepsToConsider);

      icpDoubleSupportTrajectoryGenerator = new ICPPlannerTrajectoryGenerator(namePrefix + "DoubleSupport", worldFrame, omega0, registry);
      icpSingleSupportTrajectoryGenerator = new ICPPlannerSegmentedTrajectoryGenerator(namePrefix + "SingleSupport", worldFrame, omega0, registry);


      referenceCMPsCalculator = new ReferenceCentroidalMomentumPivotLocationsCalculator(namePrefix, bipedSupportPolygons, contactableFeet,
                                                                                        numberOfFootstepsToConsider, registry);

      yoSingleSupportInitialCoM = new YoFramePoint(namePrefix + "SingleSupportInitialCoM", worldFrame, registry);
      yoSingleSupportFinalCoM = new YoFramePoint(namePrefix + "SingleSupportFinalCoM", worldFrame, registry);

      ReferenceFrame[] framesToRegister = new ReferenceFrame[] {worldFrame, midFeetZUpFrame, soleZUpFrames.get(RobotSide.LEFT),
            soleZUpFrames.get(RobotSide.RIGHT)};
      for (int i = 0; i < numberOfFootstepsToConsider - 1; i++)
      {
         YoFramePointInMultipleFrames entryCornerPoint = new YoFramePointInMultipleFrames(namePrefix + "EntryCornerPoints" + i, registry, framesToRegister);
         entryCornerPoints.add(entryCornerPoint);

         YoFramePointInMultipleFrames exitCornerPoint = new YoFramePointInMultipleFrames(namePrefix + "ExitCornerPoints" + i, registry, framesToRegister);
         exitCornerPoints.add(exitCornerPoint);
      }

      parentRegistry.addChild(registry);

      if (yoGraphicsListRegistry != null)
      {
         setupVisualizers(yoGraphicsListRegistry);
      }
   }

   @Override
   public void initializeParameters(ICPPlannerParameters icpPlannerParameters)
   {
      super.initializeParameters(icpPlannerParameters);

      if (icpPlannerParameters instanceof ContinuousCMPICPPlannerParameters)
      {
         ContinuousCMPICPPlannerParameters continuousCMPICPPlannerParameters = (ContinuousCMPICPPlannerParameters) icpPlannerParameters;
         icpSingleSupportTrajectoryGenerator.setMaximumSplineDuration(continuousCMPICPPlannerParameters.getMaxDurationForSmoothingEntryToExitCoPSwitch());
         icpSingleSupportTrajectoryGenerator.setMinimumTimeToSpendOnFinalCMP(continuousCMPICPPlannerParameters.getMinTimeToSpendOnExitCoPInSingleSupport());

         numberFootstepsToConsider.set(continuousCMPICPPlannerParameters.getNumberOfFootstepsToConsider());
         useTwoConstantCMPsPerSupport.set(continuousCMPICPPlannerParameters.getNumberOfCoPWayPointsPerFoot() > 1);

         referenceCMPsCalculator.initializeParameters(continuousCMPICPPlannerParameters);
      }
      else
      {
         throw new RuntimeException("Tried to submit the wrong type of parameters.");
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

   @Override
   /** {@inheritDoc} */
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

   @Override
   /** {@inheritDoc} */
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


   @Override
   /** {@inheritDoc} */
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

   @Override
   /** {@inheritDoc} */
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


   @Override
   /** {@inheritDoc} */
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


   @Override
   /** {@inheritDoc} */
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
      singleSupportInitialCoM.set(desiredCoMPosition);
      updateSingleSupportPlan();
   }


   @Override
   /** {@inheritDoc} */
   protected void updateTransferPlan()
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

   @Override
   /** {@inheritDoc} */
   protected void updateSingleSupportPlan()
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

   @Override
   /** {@inheritDoc} */
   public void computeFinalCoMPositionInSwing()
   {
      ReferenceFrame supportSoleFrame = initializeSwingTrajectory();

      computeFinalCoMPositionInSwingInternal();

      singleSupportInitialICP.changeFrame(supportSoleFrame);
      entryCornerPoints.get(0).changeFrame(supportSoleFrame);
      singleSupportFinalICP.changeFrame(worldFrame);
      changeFrameOfRemainingCornerPoints(1, worldFrame);
   }

   @Override
   /** {@inheritDoc} */
   public void compute(double time)
   {
      timer.startMeasurement();

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
         tempICP.setIncludingFrame(singleSupportInitialICP);
         singleSupportInitialCoM.set(yoSingleSupportInitialCoM);
         tempICP.changeFrame(worldFrame);
         double swingDuration = swingDurations.get(0).getDoubleValue();
         time = MathTools.clamp(time, 0.0, swingDuration);
         computeDesiredCapturePointPosition(omega0, time, tempICP, tempConstantCMP, desiredICPPosition);
         computeDesiredCapturePointVelocity(omega0, time, tempICP, tempConstantCMP, desiredICPVelocity);
         computeDesiredCapturePointAcceleration(omega0, time, tempICP, tempConstantCMP, desiredICPAcceleration);

         integrateCoMPositionUsingConstantCMP(0.0, time, omega0, tempConstantCMP, tempICP, singleSupportInitialCoM, tempCoM);
         desiredCoMPosition.set(tempCoM);
      }
      // TODO: possible set this
//      desiredCoMPosition.setZ(0.0);

      decayDesiredVelocityIfNeeded();

      computeDesiredCentroidalMomentumPivot(desiredICPPosition, desiredICPVelocity, omega0, desiredCMPPosition);
      computeDesiredCentroidalMomentumPivotVelocity(desiredICPVelocity, desiredICPAcceleration, omega0, desiredCMPVelocity);

      timer.stopMeasurement();
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
         double distanceFromDesiredICPToMidfeetZUpFrame = desiredICPPosition.distanceXY(tempICP);
         tempICP.setToZero(transferFromSoleFrame);
         tempICP.changeFrame(worldFrame);
         double distanceFromDesiredICPToTransferFromSoleFrame = desiredICPPosition.distanceXY(tempICP);

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
         desiredCoMPosition.set(icpPositionToHold.getX(), icpPositionToHold.getY(), 0.0);
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

      if (Double.isNaN(transferDuration))
      {
         transferDuration = 0.0;
      }

      icpDoubleSupportTrajectoryGenerator.setTrajectoryTime(transferDuration);
      icpDoubleSupportTrajectoryGenerator.setInitialConditions(desiredICPPosition, desiredICPVelocity, initialFrame);
      icpDoubleSupportTrajectoryGenerator.setFinalConditions(singleSupportInitialICP, singleSupportInitialICPVelocity, finalFrame);
      icpDoubleSupportTrajectoryGenerator.setInitialCoMPosition(desiredCoMPosition, worldFrame);
      icpDoubleSupportTrajectoryGenerator.initialize();
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
      singleSupportInitialCoM.set(yoSingleSupportInitialCoM);

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

   private void update()
   {
      singleSupportInitialICP.notifyVariableChangedListeners();
      singleSupportFinalICP.notifyVariableChangedListeners();

      for (int i = 0; i < entryCornerPoints.size(); i++)
         entryCornerPoints.get(i).notifyVariableChangedListeners();
      for (int i = 0; i < exitCornerPoints.size(); i++)
         exitCornerPoints.get(i).notifyVariableChangedListeners();
   }


   @Override
   /** {@inheritDoc} */
   public void setTransferDuration(int stepNumber, double duration)
   {
      int numberOfFootstepRegistered = referenceCMPsCalculator.getNumberOfFootstepRegistered();
      if (stepNumber < numberOfFootstepRegistered + 1)
         transferDurations.get(stepNumber).set(duration);
   }

   @Override
   /** {@inheritDoc} */
   public void setSwingDuration(int stepNumber, double duration)
   {
      int numberOfFootstepRegistered = referenceCMPsCalculator.getNumberOfFootstepRegistered();
      if (stepNumber < numberOfFootstepRegistered)
         swingDurations.get(stepNumber).set(duration);
   }

   private final FramePoint3D tempFinalICP = new FramePoint3D();

   @Override
   /** {@inheritDoc} */
   public void getFinalDesiredCapturePointPosition(FramePoint3D finalDesiredCapturePointPositionToPack)
   {
      if (isStanding.getBooleanValue())
         referenceCMPsCalculator.getNextEntryCMP(tempFinalICP);
      else
         tempFinalICP.setIncludingFrame(entryCornerPoints.get(1));
      tempFinalICP.changeFrame(worldFrame);
      finalDesiredCapturePointPositionToPack.setIncludingFrame(tempFinalICP);
   }

   @Override
   /** {@inheritDoc} */
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
         finalDesiredCapturePointPositionToPack.set(tempFinalICP);
      }
      else
      {
         tempFinalICP.setIncludingFrame(entryCornerPoints.get(1));
         tempFinalICP.changeFrame(finalDesiredCapturePointPositionToPack.getReferenceFrame());
         finalDesiredCapturePointPositionToPack.set(tempFinalICP);
      }
   }

   private final FramePoint3D tempFinalCoM = new FramePoint3D();

   @Override
   /** {@inheritDoc} */
   public void getFinalDesiredCenterOfMassPosition(FramePoint3D finalDesiredCenterOfMassPositionToPack)
   {
      if (isStanding.getBooleanValue())
      {
         referenceCMPsCalculator.getNextEntryCMP(tempFinalICP);
         tempFinalCoM.set(tempFinalICP);
      }
      else
      {
         tempFinalCoM.set(singleSupportFinalCoM);
      }

      tempFinalCoM.changeFrame(worldFrame);
      finalDesiredCenterOfMassPositionToPack.setIncludingFrame(tempFinalCoM);
   }

   @Override
   /** {@inheritDoc} */
   public void getNextExitCMP(FramePoint3D entryCMPToPack)
   {
      referenceCMPsCalculator.getNextExitCMP(entryCMPToPack);
   }

   @Override
   /** {@inheritDoc} */
   public boolean isOnExitCMP()
   {
      if (isDoubleSupport.getBooleanValue())
         return false;
      else
         return icpSingleSupportTrajectoryGenerator.isOnExitCMP();
   }

   @Override
   /** {@inheritDoc} */
   public int getNumberOfFootstepsToConsider()
   {
      return numberFootstepsToConsider.getIntegerValue();
   }

   @Override
   /** {@inheritDoc} */
   public int getNumberOfFootstepsRegistered()
   {
      return referenceCMPsCalculator.getNumberOfFootstepRegistered();
   }
}
