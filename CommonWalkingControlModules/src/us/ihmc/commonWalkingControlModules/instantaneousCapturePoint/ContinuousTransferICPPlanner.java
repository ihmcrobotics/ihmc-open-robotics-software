package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint;

import static us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools.computeDesiredCapturePointPosition;
import static us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools.computeDesiredCapturePointVelocity;
import static us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools.computeDesiredCornerPointsDoubleSupport;
import static us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools.computeDesiredCornerPointsSingleSupport;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
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
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Implementation of the ICP (Instantaneous Capture Point) planners introduced by Johannes Englsberger:
 * <ol>
 * <li> Three-Dimensional Bipedal Walking Control Based on Divergent Component of Motion, IEEE Transactions on Robotics, 2015.
 * <li> Three-Dimensional Bipedal Walking Control Using Divergent Component of Motion, IEEE/RSJ International Conference on Intelligent Robots and Systems, 2013. 
 * <li> Trajectory generation for continuous leg forces during double support and heel-to-toe shift based on divergent component of motion, IEEE/RSJ International Conference on Intelligent Robots and Systems, 2014.
 * </ol>
 * 
 * To summarize, the idea is to compute a smooth ICP trajectory for a given set of upcoming footsteps and a set of desired constant CMP (Centroidal Moment Pivot) locations for each support.
 * <p>
 * Note that this ICP planner does not guarantee that the resulting CMP will always be inside the support polygon. Indeed the ICP trajectory is generated using splines without additional constraints on CMP.
 * Usually the CMP will remain inside the support polygon, however, for certain combinations of transfer and swing durations, the generated ICP trajectory might result in CMP overshooting outside the support polygon.
 * This is mostly happens when transfer and swing durations are on very different scales, i.e. one is very small while the other is very large.
 * </p>
 * <p>
 * Note on the reference frames used in the planner.
 * The overall ICP trajectory is a piecewise set of splines representing portions of the trajectory for each phase of the gait cycle (single support, double support).
 * Each of these trajectories starts from initial conditions expressed in a reference frame relevant to the initial support context and ends at final conditions expressed in a reference frame relevant to the final support situation.
 * By doing so, foot slippage is inherently handle by this ICP planner as when a foot is slipping, the active trajectory is immediately stretched such that the initial and final conditions are following the foot they are attached to.
 * </p>
 * <p>
 * How to use the ICP planner.
 * <ul>
 * <li> First clear the current plan with {@link #clearPlan()}.
 * <li> Register the footsteps to be used in the plan with {@link #addFootstepToPlan(Footstep, FootstepTiming)}.
 * <li> Set the duration to be used for the last transfer phase, i.e. once the last footstep has been reached, using {@link #setFinalTransferTime(double)}.
 * <li> Then several options depending on the current phase of the walking gait:
 * <ul>
 * <li> The robot is simply standing: simply call {@link #initializeForStanding(double)} providing the current controller time.
 * <li> The robot is about to perform a transfer of its weight from one foot to the other. You first need to provide the information about which foot the robot is transferring to via {@link #setTransferToSide(RobotSide)}
 *     or transferring from via {@link #setTransferFromSide(RobotSide)}. Then call {@link #initializeForTransfer(double)} providing the current controller time.
 * <li> The robot is about to perform a single support phase. You first need to provide the information about the foot that will be used a the support foot via {@link #setSupportLeg(RobotSide)}.
 *     Then call {@link #initializeForSingleSupport(double)} providing the current controller time.
 * </ul>
 * <li> TODO finish up documentation.
 * </ul>
 * </p>
 *
 */
public class ContinuousTransferICPPlanner implements ICPPlanner
{
   private static final boolean VISUALIZE = false;
   private static final double ICP_CORNER_POINT_SIZE = 0.004;

   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String namePrefix = "icpPlanner";

   private final BooleanYoVariable isStanding = new BooleanYoVariable(namePrefix + "IsStanding", registry);
   private final BooleanYoVariable isInitialTransfer = new BooleanYoVariable(namePrefix + "IsInitialTransfer", registry);
   private final BooleanYoVariable isDoubleSupport = new BooleanYoVariable(namePrefix + "IsDoubleSupport", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable(namePrefix + "TimeInCurrentState", registry);
   private final DoubleYoVariable minSwingTime = new DoubleYoVariable(namePrefix + "MinSwingTime", registry);
   private final DoubleYoVariable finalTransferTime = new DoubleYoVariable(namePrefix + "FinalTransferTime", registry);
   private final DoubleYoVariable transferTimeSplitFraction = new DoubleYoVariable(namePrefix + "TransferTimeSplitFraction", registry);
   private final DoubleYoVariable initialTime = new DoubleYoVariable(namePrefix + "InitialTime", registry);
   private final DoubleYoVariable remainingTime = new DoubleYoVariable(namePrefix + "RemainingTime", registry);
   private final IntegerYoVariable numberFootstepsToConsider = new IntegerYoVariable(namePrefix + "NumberFootstepsToConsider", registry);

   private final DoubleYoVariable velocityDecayDurationWhenDone = new DoubleYoVariable(namePrefix + "VelocityDecayDurationWhenDone", registry);
   private final DoubleYoVariable velocityReductionFactor = new DoubleYoVariable(namePrefix + "VelocityReductionFactor", registry);

   private final YoFramePointInMultipleFrames singleSupportInitialICP;
   private final YoFrameVector singleSupportInitialICPVelocity = new YoFrameVector(namePrefix + "SingleSupportInitialICPVelocity", worldFrame, registry);

   private final YoFramePointInMultipleFrames singleSupportFinalICP;
   private final YoFrameVector singleSupportFinalICPVelocity = new YoFrameVector(namePrefix + "SingleSupportFinalICPVelocity", worldFrame, registry);

   private final YoFramePointInMultipleFrames singleSupportInitialCoM;
   private final YoFramePointInMultipleFrames singleSupportFinalCoM;

   private final YoFramePoint desiredCentroidalMomentumPivotPosition = new YoFramePoint(namePrefix + "DesiredCentroidalMomentumPosition", worldFrame, registry);
   private final YoFrameVector desiredCentroidalMomentumPivotVelocity = new YoFrameVector(namePrefix + "DesiredCentroidalMomentumVelocity", worldFrame,
                                                                                          registry);
   private final YoFramePoint desiredCapturePointPosition = new YoFramePoint(namePrefix + "DesiredCapturePointPosition", worldFrame, registry);
   private final YoFrameVector desiredCapturePointVelocity = new YoFrameVector(namePrefix + "DesiredCapturePointVelocity", worldFrame, registry);
   private final YoFrameVector desiredCapturePointAcceleration = new YoFrameVector(namePrefix + "DesiredCapturePointAcceleration", worldFrame, registry);
   private final DoubleYoVariable omega0 = new DoubleYoVariable(namePrefix + "Omega0", registry);

   private final YoFramePoint desiredCenterOfMassPosition = new YoFramePoint(namePrefix + "DesiredCenterOfMassPosition", worldFrame, registry);

   private final BooleanYoVariable requestedHoldPosition = new BooleanYoVariable(namePrefix + "RequestedHoldPosition", registry);
   private final BooleanYoVariable isHoldingPosition = new BooleanYoVariable(namePrefix + "IsHoldingPosition", registry);
   private final YoFramePoint actualICPToHold = new YoFramePoint(namePrefix + "ActualCapturePointToHold", worldFrame, registry);

   private final BooleanYoVariable useTwoConstantCMPsPerSupport = new BooleanYoVariable(namePrefix + "UseTwoConstantCMPsPerSupport", registry);
   private final DoubleYoVariable swingTimeSplitFraction = new DoubleYoVariable(namePrefix + "SwingTimeSplitFraction", registry);

   private final EnumYoVariable<RobotSide> transferToSide = new EnumYoVariable<>(namePrefix + "TransferToSide", registry, RobotSide.class, true);
   private final EnumYoVariable<RobotSide> supportSide = new EnumYoVariable<>(namePrefix + "SupportSide", registry, RobotSide.class, true);

   private final List<YoFramePointInMultipleFrames> entryCornerPoints = new ArrayList<>();
   private final List<YoFramePointInMultipleFrames> exitCornerPoints = new ArrayList<>();

   private final List<DoubleYoVariable> swingTimes = new ArrayList<>();
   private final List<DoubleYoVariable> transferTimes = new ArrayList<>();

   private final ICPPlannerTrajectoryGenerator icpDoubleSupportTrajectoryGenerator;
   private final ICPPlannerSegmentedTrajectoryGenerator icpSingleSupportTrajectoryGenerator;
   private final ReferenceCentroidalMomentumPivotLocationsCalculator referenceCMPsCalculator;

   private final ReferenceFrame midFeetZUpFrame;
   private final SideDependentList<ReferenceFrame> soleZUpFrames;

   private final FramePoint tempConstantCMP = new FramePoint();
   private final FramePoint tempICP = new FramePoint();

   public ContinuousTransferICPPlanner(BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                     CapturePointPlannerParameters icpPlannerParameters, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      isStanding.set(true);

      finalTransferTime.setToNaN();

      actualICPToHold.setToNaN();
      isHoldingPosition.set(false);

      icpDoubleSupportTrajectoryGenerator = new ICPPlannerTrajectoryGenerator(namePrefix + "DoubleSupport", worldFrame, omega0, registry);
      icpSingleSupportTrajectoryGenerator = new ICPPlannerSegmentedTrajectoryGenerator(namePrefix + "SingleSupport", worldFrame, omega0, yoGraphicsListRegistry,
                                                                                       registry);
      icpSingleSupportTrajectoryGenerator.setMaximumSplineDuration(icpPlannerParameters.getMaxDurationForSmoothingEntryToExitCMPSwitch());
      icpSingleSupportTrajectoryGenerator.setMinimumTimeToSpendOnExitCMP(icpPlannerParameters.getMinTimeToSpendOnExitCMPInSingleSupport());

      numberFootstepsToConsider.set(icpPlannerParameters.getNumberOfFootstepsToConsider());
      transferTimeSplitFraction.set(icpPlannerParameters.getDoubleSupportSplitFraction());
      swingTimeSplitFraction.set(icpPlannerParameters.getTimeSpentOnExitCMPInPercentOfStepTime());
      useTwoConstantCMPsPerSupport.set(icpPlannerParameters.useTwoCMPsPerSupport());

      velocityDecayDurationWhenDone.set(icpPlannerParameters.getVelocityDecayDurationWhenDone());
      velocityReductionFactor.set(Double.NaN);

      // Initialize omega0 to NaN to force the user to explicitly set it.
      omega0.set(Double.NaN);

      minSwingTime.set(0.3); // TODO Need to be extracted

      referenceCMPsCalculator = new ReferenceCentroidalMomentumPivotLocationsCalculator(namePrefix, bipedSupportPolygons, contactableFeet,
                                                                                        numberFootstepsToConsider.getIntegerValue(), registry);
      referenceCMPsCalculator.initializeParameters(icpPlannerParameters);

      midFeetZUpFrame = bipedSupportPolygons.getMidFeetZUpFrame();
      soleZUpFrames = bipedSupportPolygons.getSoleZUpFrames();

      ReferenceFrame[] framesToRegister = new ReferenceFrame[] {worldFrame, midFeetZUpFrame, soleZUpFrames.get(RobotSide.LEFT),
            soleZUpFrames.get(RobotSide.RIGHT)};
      singleSupportInitialICP = new YoFramePointInMultipleFrames(namePrefix + "SingleSupportInitialICP", registry, framesToRegister);
      singleSupportFinalICP = new YoFramePointInMultipleFrames(namePrefix + "SingleSupportFinalICP", registry, framesToRegister);

      singleSupportInitialCoM = new YoFramePointInMultipleFrames(namePrefix + "SingleSupportInitialCoM", registry, framesToRegister);
      singleSupportFinalCoM = new YoFramePointInMultipleFrames(namePrefix + "SingleSupportFinalCoM", registry, framesToRegister);

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue() - 1; i++)
      {
         YoFramePointInMultipleFrames earlyCornerPoint = new YoFramePointInMultipleFrames(namePrefix + "EntryCornerPoints" + i, registry, framesToRegister);
         entryCornerPoints.add(earlyCornerPoint);

         YoFramePointInMultipleFrames lateCornerPoint = new YoFramePointInMultipleFrames(namePrefix + "ExitCornerPoints" + i, registry, framesToRegister);
         exitCornerPoints.add(lateCornerPoint);
      }

      for (int i = 0; i < numberFootstepsToConsider.getIntegerValue(); i++)
      {
         DoubleYoVariable swingTime = new DoubleYoVariable(namePrefix + "SwingTime" + i, registry);
         swingTime.setToNaN();
         swingTimes.add(swingTime);
         DoubleYoVariable transferTime = new DoubleYoVariable(namePrefix + "TransferTime" + i, registry);
         transferTime.setToNaN();
         transferTimes.add(transferTime);
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
         YoGraphicPosition icpEarlyCornerPointsViz = new YoGraphicPosition("EntryCornerPoints" + i,
                                                                           entryCornerPoints.get(i).buildUpdatedYoFramePointForVisualizationOnly(),
                                                                           ICP_CORNER_POINT_SIZE, YoAppearance.Blue(), GraphicType.SOLID_BALL);

         yoGraphicsList.add(icpEarlyCornerPointsViz);
         artifactList.add(icpEarlyCornerPointsViz.createArtifact());

         YoGraphicPosition lateCornerPointsViz = new YoGraphicPosition("ExitCornerPoints" + i,
                                                                       exitCornerPoints.get(i).buildUpdatedYoFramePointForVisualizationOnly(),
                                                                       ICP_CORNER_POINT_SIZE, YoAppearance.Blue(), GraphicType.BALL);

         yoGraphicsList.add(lateCornerPointsViz);
         artifactList.add(lateCornerPointsViz.createArtifact());
      }

      YoGraphicPosition singleSupportInitialICPViz = new YoGraphicPosition("singleSupportInitialICP",
                                                                           singleSupportInitialICP.buildUpdatedYoFramePointForVisualizationOnly(), 0.004,
                                                                           YoAppearance.Chocolate(), GraphicType.SOLID_BALL);
      yoGraphicsList.add(singleSupportInitialICPViz);
      artifactList.add(singleSupportInitialICPViz.createArtifact());

      YoGraphicPosition singleSupportFinalICPViz = new YoGraphicPosition("singleSupportFinalICP",
                                                                         singleSupportFinalICP.buildUpdatedYoFramePointForVisualizationOnly(), 0.004,
                                                                         YoAppearance.Chocolate(), GraphicType.BALL);
      yoGraphicsList.add(singleSupportFinalICPViz);
      artifactList.add(singleSupportFinalICPViz.createArtifact());

      YoGraphicPosition desiredCenterOfMassPositionViz = new YoGraphicPosition("desiredCenterOfMassPositionViz",
            desiredCenterOfMassPosition, 0.004, YoAppearance.Black(), GraphicType.BALL_WITH_CROSS);
      yoGraphicsList.add(desiredCenterOfMassPositionViz);
      artifactList.add(desiredCenterOfMassPositionViz.createArtifact());

      YoGraphicPosition singleSupportFinalCoMViz = new YoGraphicPosition("singleSupportFinalCoM",
            singleSupportFinalICP.buildUpdatedYoFramePointForVisualizationOnly(), 0.004, YoAppearance.Black(), GraphicType.BALL_WITH_CROSS);
      yoGraphicsList.add(singleSupportFinalCoMViz);
      artifactList.add(singleSupportFinalCoMViz.createArtifact());

      icpSingleSupportTrajectoryGenerator.createVisualizers(yoGraphicsList, artifactList);

      artifactList.setVisible(VISUALIZE);
      yoGraphicsList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   @Override
   public void clearPlan()
   {
      referenceCMPsCalculator.clear();

      for (int i = 0; i < swingTimes.size(); i++)
      {
         swingTimes.get(i).setToNaN();
         transferTimes.get(i).setToNaN();
      }
   }

   @Override
   public void setSupportLeg(RobotSide robotSide)
   {
      supportSide.set(robotSide);
   }

   @Override
   public void setTransferToSide(RobotSide robotSide)
   {
      transferToSide.set(robotSide);
   }

   @Override
   public void setTransferFromSide(RobotSide robotSide)
   {
      if (robotSide != null)
         transferToSide.set(robotSide.getOppositeSide());
   }

   @Override
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      if (footstep == null)
         return;

      referenceCMPsCalculator.addUpcomingFootstep(footstep);
      int footstepIndex = referenceCMPsCalculator.getNumberOfFootstepRegistered() - 1;
      swingTimes.get(footstepIndex).set(timing.getSwingTime());
      transferTimes.get(footstepIndex).set(timing.getTransferTime());
   }

   @Override
   public void setDesiredCapturePointState(FramePoint currentDesiredCapturePointPosition, FrameVector currentDesiredCapturePointVelocity)
   {
      desiredCapturePointPosition.set(currentDesiredCapturePointPosition);
      desiredCapturePointVelocity.set(currentDesiredCapturePointVelocity);
   }

   @Override
   public void setDesiredCapturePointState(YoFramePoint currentDesiredCapturePointPosition, YoFrameVector currentDesiredCapturePointVelocity)
   {
      desiredCapturePointPosition.set(currentDesiredCapturePointPosition);
      desiredCapturePointVelocity.set(currentDesiredCapturePointVelocity);
   }

   @Override
   public void setDesiredCapturePointState(FramePoint2d currentDesiredCapturePointPosition, FrameVector2d currentDesiredCapturePointVelocity)
   {
      // Do not set the Z to zero!
      desiredCapturePointPosition.checkReferenceFrameMatch(currentDesiredCapturePointPosition);
      desiredCapturePointPosition.setX(currentDesiredCapturePointPosition.getX());
      desiredCapturePointPosition.setY(currentDesiredCapturePointPosition.getY());

      desiredCapturePointVelocity.checkReferenceFrameMatch(currentDesiredCapturePointVelocity);
      desiredCapturePointVelocity.setX(currentDesiredCapturePointVelocity.getX());
      desiredCapturePointVelocity.setY(currentDesiredCapturePointVelocity.getY());
   }

   @Override
   public void setDesiredCapturePointState(YoFramePoint2d currentDesiredCapturePointPosition, YoFrameVector2d currentDesiredCapturePointVelocity)
   {
      // Do not set the Z to zero!
      desiredCapturePointPosition.checkReferenceFrameMatch(currentDesiredCapturePointPosition);
      desiredCapturePointPosition.setX(currentDesiredCapturePointPosition.getX());
      desiredCapturePointPosition.setY(currentDesiredCapturePointPosition.getY());

      desiredCapturePointVelocity.checkReferenceFrameMatch(currentDesiredCapturePointVelocity);
      desiredCapturePointVelocity.setX(currentDesiredCapturePointVelocity.getX());
      desiredCapturePointVelocity.setY(currentDesiredCapturePointVelocity.getY());
   }

   public void setDesiredCenterOfMassPosition(YoFramePoint currentDesiredCenterOfMassPosition)
   {
      desiredCenterOfMassPosition.set(currentDesiredCenterOfMassPosition);
   }


   @Override
   public void holdCurrentICP(double initialTime, FramePoint actualICPToHold)
   {
      this.actualICPToHold.set(actualICPToHold);
      requestedHoldPosition.set(true);
   }

   @Override
   public void initializeForStanding(double initialTime)
   {
      clearPlan();
      isStanding.set(true);
      isDoubleSupport.set(true);
      this.initialTime.set(initialTime);
      transferTimes.get(0).set(finalTransferTime.getDoubleValue());
      updateTransferPlan();
   }

   @Override
   public void initializeForTransfer(double initialTime)
   {
      isDoubleSupport.set(true);
      this.initialTime.set(initialTime);

      int numberOfFootstepRegistered = referenceCMPsCalculator.getNumberOfFootstepRegistered();
      if (numberOfFootstepRegistered < numberFootstepsToConsider.getIntegerValue())
         transferTimes.get(numberOfFootstepRegistered).set(finalTransferTime.getDoubleValue());

      updateTransferPlan();
      computeCoMFinalCoMPositionInTransfer();
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
         double distanceFromDesiredICPToMidfeetZUpFrame = desiredCapturePointPosition.getXYPlaneDistance(tempICP);
         tempICP.setToZero(transferFromSoleFrame);
         tempICP.changeFrame(worldFrame);
         double distanceFromDesiredICPToTransferFromSoleFrame = desiredCapturePointPosition.getXYPlaneDistance(tempICP);

         if (distanceFromDesiredICPToMidfeetZUpFrame < distanceFromDesiredICPToTransferFromSoleFrame)
            initialFrame = midFeetZUpFrame;
         else
            initialFrame = transferFromSoleFrame;
      }
      ReferenceFrame finalFrame = isDoneWalking ? midFeetZUpFrame : transferToSoleFrame;

      if (requestedHoldPosition.getBooleanValue())
      {
         desiredCapturePointPosition.set(actualICPToHold);
         desiredCapturePointVelocity.setToZero();
         singleSupportInitialICP.setIncludingFrame(actualICPToHold);
         singleSupportFinalICP.setIncludingFrame(actualICPToHold);
         singleSupportInitialICPVelocity.set(0.0, 0.0, 0.0);
         setCornerPointsToNaN();
         actualICPToHold.setToNaN();
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
         double doubleSupportTimeSpentAfterEntryCornerPoint = transferTimes.get(0).getDoubleValue() * (1.0 - transferTimeSplitFraction.getDoubleValue());
         double timeToSpendOnExitCMPBeforeNextDoubleSupport = swingTimes.get(0).getDoubleValue() * (1.0 - swingTimeSplitFraction.getDoubleValue());

         double omega0 = this.omega0.getDoubleValue();

         if (useTwoConstantCMPsPerSupport.getBooleanValue())
         {
            computeDesiredCornerPointsDoubleSupport(entryCornerPoints, exitCornerPoints, entryCMPs, exitCMPs, swingTimes, transferTimes,
                                                    swingTimeSplitFraction.getDoubleValue(), transferTimeSplitFraction.getDoubleValue(), omega0);
            computeDesiredCapturePointPosition(omega0, timeToSpendOnExitCMPBeforeNextDoubleSupport, exitCornerPoints.get(1), exitCMPs.get(1),
                                               singleSupportFinalICP);
            exitCornerPoints.get(0).changeFrame(initialFrame);
            exitCornerPoints.get(1).changeFrame(finalFrame);
         }
         else
         {
            computeDesiredCornerPointsDoubleSupport(entryCornerPoints, entryCMPs, swingTimes, transferTimes, transferTimeSplitFraction.getDoubleValue(),
                                                    omega0);
            double timeToNextCornerPoint = doubleSupportTimeSpentAfterEntryCornerPoint + swingTimes.get(0).getDoubleValue();
            computeDesiredCapturePointPosition(omega0, timeToNextCornerPoint, entryCornerPoints.get(1), entryCMPs.get(1), singleSupportFinalICP);
         }

         computeDesiredCapturePointPosition(omega0, doubleSupportTimeSpentAfterEntryCornerPoint, entryCornerPoints.get(1), entryCMPs.get(1),
                                            singleSupportInitialICP);
         computeDesiredCapturePointVelocity(omega0, 0.0, singleSupportInitialICP, entryCMPs.get(1), singleSupportInitialICPVelocity);

         entryCornerPoints.get(0).changeFrame(initialFrame);
         entryCornerPoints.get(1).changeFrame(finalFrame);
         changeFrameOfRemainingCornerPoints(2, worldFrame);
         isHoldingPosition.set(false);
      }

      singleSupportInitialICP.changeFrame(finalFrame);
      singleSupportFinalICP.changeFrame(worldFrame);

      if (isStanding.getBooleanValue() && !isDoneWalking)
      {
         isInitialTransfer.set(true);
         isStanding.set(false);
      }

      icpDoubleSupportTrajectoryGenerator.setTrajectoryTime(transferTimes.get(0).getDoubleValue());
      icpDoubleSupportTrajectoryGenerator.setInitialConditions(desiredCapturePointPosition, desiredCapturePointVelocity, initialFrame);
      icpDoubleSupportTrajectoryGenerator.setFinalConditions(singleSupportInitialICP, singleSupportInitialICPVelocity, finalFrame);
      icpDoubleSupportTrajectoryGenerator.setInitialCoMPosition(desiredCenterOfMassPosition, worldFrame);
      icpDoubleSupportTrajectoryGenerator.initialize();
   }

   private void computeCoMFinalCoMPositionInTransfer()
   {
      icpDoubleSupportTrajectoryGenerator.computeFinalCoMPosition(desiredCenterOfMassPosition, singleSupportInitialCoM);

      updateSingleSupportPlan(); // // TODO: 3/13/17 this may not work
      icpSingleSupportTrajectoryGenerator.computeFinalCoMPosition(singleSupportInitialCoM, singleSupportFinalCoM);
   }


   @Override
   public void initializeForSingleSupport(double initialTime)
   {
      isHoldingPosition.set(false);

      isStanding.set(false);
      isInitialTransfer.set(false);
      isDoubleSupport.set(false);
      this.initialTime.set(initialTime);

      int numberOfFootstepRegistered = referenceCMPsCalculator.getNumberOfFootstepRegistered();
      if (numberOfFootstepRegistered < numberFootstepsToConsider.getIntegerValue())
         transferTimes.get(numberOfFootstepRegistered).set(finalTransferTime.getDoubleValue());

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
      double doubleSupportTimeSpentAfterEntryCornerPoint = transferTimes.get(0).getDoubleValue() * (1.0 - transferTimeSplitFraction.getDoubleValue());
      double timeRemainingOnEntryCMP = swingTimes.get(0).getDoubleValue() * swingTimeSplitFraction.getDoubleValue();
      double timeToSpendOnExitCMPBeforeDoubleSupport = swingTimes.get(0).getDoubleValue() * (1.0 - swingTimeSplitFraction.getDoubleValue());

      switchCornerPointsToWorldFrame();
      singleSupportInitialICP.switchCurrentReferenceFrame(worldFrame);
      singleSupportFinalICP.switchCurrentReferenceFrame(worldFrame);

      ReferenceFrame supportSoleFrame = soleZUpFrames.get(supportSide);
      double omega0 = this.omega0.getDoubleValue();
      if (useTwoConstantCMPsPerSupport.getBooleanValue())
      {
         computeDesiredCornerPointsSingleSupport(entryCornerPoints, exitCornerPoints, entryCMPs, exitCMPs, swingTimes, transferTimes,
                                                 swingTimeSplitFraction.getDoubleValue(), transferTimeSplitFraction.getDoubleValue(), omega0);
         computeDesiredCapturePointPosition(omega0, doubleSupportTimeSpentAfterEntryCornerPoint, entryCornerPoints.get(0), entryCMPs.get(0),
                                            singleSupportInitialICP);
         computeDesiredCapturePointVelocity(omega0, doubleSupportTimeSpentAfterEntryCornerPoint, entryCornerPoints.get(0), entryCMPs.get(0),
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
         computeDesiredCornerPointsSingleSupport(entryCornerPoints, entryCMPs, swingTimes, transferTimes, transferTimeSplitFraction.getDoubleValue(), omega0);
         computeDesiredCapturePointPosition(omega0, doubleSupportTimeSpentAfterEntryCornerPoint, entryCornerPoints.get(0), entryCMPs.get(0),
                                            singleSupportInitialICP);
         computeDesiredCapturePointPosition(omega0, doubleSupportTimeSpentAfterEntryCornerPoint + swingTimes.get(0).getDoubleValue(), entryCornerPoints.get(0),
                                            entryCMPs.get(0), singleSupportFinalICP);
         
         
         // // TODO: 3/13/17 CoM position computation
      }

      singleSupportInitialICP.changeFrame(supportSoleFrame);
      entryCornerPoints.get(0).changeFrame(supportSoleFrame);
      singleSupportFinalICP.changeFrame(worldFrame);
      changeFrameOfRemainingCornerPoints(1, worldFrame);
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

   @Override
   public void updatePlanForSingleSupportDisturbances(double time, FramePoint2d actualCapturePointPosition)
   {
      initializeForSingleSupport(initialTime.getDoubleValue());

      if (isDone(time))
         return;

      double deltaTimeToBeAccounted = estimateDeltaTimeBetweenDesiredICPAndActualICP(time, actualCapturePointPosition);

      if (Double.isNaN(deltaTimeToBeAccounted))
         return;

      // Ensure that we don't shift the time by more than what's remaining
      deltaTimeToBeAccounted = Math.min(deltaTimeToBeAccounted, computeAndReturnTimeRemaining(time));
      // Ensure the time shift won't imply a single support that's crazy short
      double currentSwingTime = swingTimes.get(0).getDoubleValue();
      deltaTimeToBeAccounted = Math.min(deltaTimeToBeAccounted, currentSwingTime - minSwingTime.getDoubleValue());

      initialTime.sub(deltaTimeToBeAccounted);
   }

   @Override
   public double estimateTimeRemainingForStateUnderDisturbance(double time, FramePoint2d actualCapturePointPosition)
   {
      if (isDone(time))
         return 0.0;

      double deltaTimeToBeAccounted = estimateDeltaTimeBetweenDesiredICPAndActualICP(time, actualCapturePointPosition);

      if (Double.isNaN(deltaTimeToBeAccounted))
         return 0.0;

      double estimatedTimeRemaining = computeAndReturnTimeRemaining(time) - deltaTimeToBeAccounted;
      estimatedTimeRemaining = MathTools.clamp(estimatedTimeRemaining, 0.0, Double.POSITIVE_INFINITY);

      return estimatedTimeRemaining;
   }

   private final FramePoint2d desiredICP2d = new FramePoint2d();
   private final FramePoint2d finalICP2d = new FramePoint2d();
   private final FrameLine2d desiredICPToFinalICPLine = new FrameLine2d();
   private final FrameLineSegment2d desiredICPToFinalICPLineSegment = new FrameLineSegment2d();
   private final FramePoint2d actualICP2d = new FramePoint2d();

   private double estimateDeltaTimeBetweenDesiredICPAndActualICP(double time, FramePoint2d actualCapturePointPosition)
   {
      computeDesiredCapturePoint(computeAndReturnTimeInCurrentState(time));
      computeDesiredCentroidalMomentumPivot();
      desiredCapturePointPosition.getFrameTuple2dIncludingFrame(desiredICP2d);
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

      double actualDistanceDueToDisturbance = desiredCentroidalMomentumPivotPosition.getXYPlaneDistance(actualICP2d);
      double expectedDistanceAccordingToPlan = desiredCentroidalMomentumPivotPosition.getXYPlaneDistance(desiredCapturePointPosition);

      computeTimeInCurrentState(time);
      double distanceRatio = actualDistanceDueToDisturbance / expectedDistanceAccordingToPlan;

      if (distanceRatio < 1.0e-3)
         return 0.0;
      else
         return Math.log(distanceRatio) / omega0.getDoubleValue();
   }

   private void computeDesiredCapturePoint(double timeInCurrentState)
   {
      update();
      referenceCMPsCalculator.update();

      if (isDoubleSupport.getBooleanValue())
      {
         icpDoubleSupportTrajectoryGenerator.compute(timeInCurrentState);
         icpDoubleSupportTrajectoryGenerator.get(desiredCapturePointPosition);
         icpDoubleSupportTrajectoryGenerator.getVelocity(desiredCapturePointVelocity);
         icpDoubleSupportTrajectoryGenerator.getAcceleration(desiredCapturePointAcceleration);

         icpDoubleSupportTrajectoryGenerator.getCoMPosition(desiredCenterOfMassPosition);
      }
      else if (useTwoConstantCMPsPerSupport.getBooleanValue())
      {
         icpSingleSupportTrajectoryGenerator.compute(timeInCurrentState);
         icpSingleSupportTrajectoryGenerator.get(desiredCapturePointPosition);
         icpSingleSupportTrajectoryGenerator.getVelocity(desiredCapturePointVelocity);
         icpSingleSupportTrajectoryGenerator.getAcceleration(desiredCapturePointAcceleration);

         icpSingleSupportTrajectoryGenerator.getCoMPosition(desiredCenterOfMassPosition);
      }
      else
      {
         referenceCMPsCalculator.getNextEntryCMP(tempConstantCMP);
         singleSupportInitialICP.getFrameTupleIncludingFrame(tempICP);
         tempICP.changeFrame(worldFrame);
         double currentSwingTime = swingTimes.get(0).getDoubleValue();
         timeInCurrentState = MathTools.clamp(timeInCurrentState, 0.0, currentSwingTime);
         CapturePointTools.computeDesiredCapturePointPosition(omega0.getDoubleValue(), timeInCurrentState, tempICP, tempConstantCMP,
                                                              desiredCapturePointPosition);
         CapturePointTools.computeDesiredCapturePointVelocity(omega0.getDoubleValue(), timeInCurrentState, tempICP, tempConstantCMP,
                                                              desiredCapturePointVelocity);
         CapturePointTools.computeDesiredCapturePointAcceleration(omega0.getDoubleValue(), timeInCurrentState, tempICP, tempConstantCMP,
                                                                  desiredCapturePointAcceleration);
      }

      decayDesiredVelocityIfNeeded(timeInCurrentState);
   }

   private void decayDesiredVelocityIfNeeded(double timeInCurrentState)
   {
      if (velocityDecayDurationWhenDone.isNaN() || isStanding.getBooleanValue())
      {
         velocityReductionFactor.set(Double.NaN);
         return;
      }

      double hasBeenDoneForDuration = timeInCurrentState;
      if (isDoubleSupport.getBooleanValue())
         hasBeenDoneForDuration -= transferTimes.get(0).getDoubleValue();
      else
         hasBeenDoneForDuration -= swingTimes.get(0).getDoubleValue();

      if (hasBeenDoneForDuration <= 0.0)
      {
         velocityReductionFactor.set(Double.NaN);
      }
      else
      {
         velocityReductionFactor.set(MathTools.clamp(1.0 - hasBeenDoneForDuration / velocityDecayDurationWhenDone.getDoubleValue(), 0.0, 1.0));
         desiredCapturePointVelocity.scale(velocityReductionFactor.getDoubleValue());
      }
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

   private void computeDesiredCentroidalMomentumPivot()
   {
      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredCapturePointPosition, desiredCapturePointVelocity, omega0.getDoubleValue(),
                                                              desiredCentroidalMomentumPivotPosition);
   }

   @Override
   public void getDesiredCapturePointPositionAndVelocity(FramePoint2d desiredCapturePointPositionToPack, FrameVector2d desiredCapturePointVelocityToPack,
                                                         double time)
   {
      computeTimeInCurrentState(time);
      computeDesiredCapturePoint(timeInCurrentState.getDoubleValue());

      desiredCapturePointPosition.getFrameTuple2dIncludingFrame(desiredCapturePointPositionToPack);
      desiredCapturePointVelocity.getFrameTuple2dIncludingFrame(desiredCapturePointVelocityToPack);
   }

   @Override
   public void getDesiredCapturePointPositionAndVelocity(FramePoint desiredCapturePointPositionToPack, FrameVector desiredCapturePointVelocityToPack,
                                                         double time)
   {
      computeTimeInCurrentState(time);
      computeDesiredCapturePoint(timeInCurrentState.getDoubleValue());

      desiredCapturePointPosition.getFrameTupleIncludingFrame(desiredCapturePointPositionToPack);
      desiredCapturePointVelocity.getFrameTupleIncludingFrame(desiredCapturePointVelocityToPack);
   }

   @Override
   public void getDesiredCapturePointPositionAndVelocity(YoFramePoint desiredCapturePointPositionToPack, YoFrameVector desiredCapturePointVelocityToPack,
                                                         double time)
   {
      computeTimeInCurrentState(time);
      computeDesiredCapturePoint(timeInCurrentState.getDoubleValue());

      desiredCapturePointPositionToPack.set(desiredCapturePointPosition);
      desiredCapturePointVelocityToPack.set(desiredCapturePointVelocity);
   }

   @Override
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint desiredCentroidalMomentumPivotPositionToPack)
   {
      computeDesiredCentroidalMomentumPivot();
      desiredCentroidalMomentumPivotPosition.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   @Override
   public void getDesiredCentroidalMomentumPivotPosition(FramePoint2d desiredCentroidalMomentumPivotPositionToPack)
   {
      computeDesiredCentroidalMomentumPivot();
      desiredCentroidalMomentumPivotPosition.getFrameTuple2dIncludingFrame(desiredCentroidalMomentumPivotPositionToPack);
   }

   @Override
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector desiredCentroidalMomentumPivotVelocityToPack)
   {
      CapturePointTools.computeDesiredCentroidalMomentumPivotVelocity(desiredCapturePointVelocity, desiredCapturePointAcceleration, omega0.getDoubleValue(),
                                                                      desiredCentroidalMomentumPivotVelocity);
      desiredCentroidalMomentumPivotVelocity.getFrameTupleIncludingFrame(desiredCentroidalMomentumPivotVelocityToPack);
   }

   @Override
   public void getDesiredCentroidalMomentumPivotVelocity(FrameVector2d desiredCentroidalMomentumPivotVelocityToPack)
   {
      CapturePointTools.computeDesiredCentroidalMomentumPivotVelocity(desiredCapturePointVelocity, desiredCapturePointAcceleration, omega0.getDoubleValue(),
                                                                      desiredCentroidalMomentumPivotVelocity);
      desiredCentroidalMomentumPivotVelocity.getFrameTuple2dIncludingFrame(desiredCentroidalMomentumPivotVelocityToPack);
   }

   protected void computeTimeInCurrentState(double time)
   {
      timeInCurrentState.set(time - initialTime.getDoubleValue());
   }

   @Override
   public double computeAndReturnTimeRemaining(double time)
   {
      computeTimeInCurrentState(time);

      DoubleYoVariable stateDuration;

      if (isDoubleSupport.getBooleanValue())
         stateDuration = transferTimes.get(0);
      else
         stateDuration = swingTimes.get(0);

      remainingTime.set(stateDuration.getDoubleValue() - timeInCurrentState.getDoubleValue());
      return remainingTime.getDoubleValue();
   }

   @Override
   public void setFinalTransferTime(double time)
   {
      finalTransferTime.set(time);
   }

   @Override
   public void setMinimumSingleSupportTimeForDisturbanceRecovery(double minTime)
   {
      minSwingTime.set(minTime);
   }

   public void setTransferTimeSplitFraction(double transferTimeSplitFraction)
   {
      this.transferTimeSplitFraction.set(transferTimeSplitFraction);
   }

   @Override
   public void setOmega0(double omega0)
   {
      this.omega0.set(omega0);
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
   public boolean isInInitialTranfer()
   {
      return isInitialTransfer.getBooleanValue();
   }

   @Override
   public void getSingleSupportInitialCapturePointPosition(FramePoint capturePointPositionToPack)
   {
      singleSupportInitialICP.getFrameTupleIncludingFrame(capturePointPositionToPack);
   }

   @Override
   public double computeAndReturnTimeInCurrentState(double time)
   {
      computeTimeInCurrentState(time);
      return timeInCurrentState.getDoubleValue();
   }

   @Override
   public void getFinalDesiredCapturePointPosition(FramePoint finalDesiredCapturePointPositionToPack)
   {
      if (isStanding.getBooleanValue())
         referenceCMPsCalculator.getNextEntryCMP(tempFinalICP);
      else
         entryCornerPoints.get(1).getFrameTupleIncludingFrame(tempFinalICP);
      tempFinalICP.changeFrame(worldFrame);
      finalDesiredCapturePointPositionToPack.setIncludingFrame(tempFinalICP);
   }

   private final FramePoint tempFinalICP = new FramePoint();

   @Override
   public void getFinalDesiredCapturePointPosition(YoFramePoint2d finalDesiredCapturePointPositionToPack)
   {
      if (isStanding.getBooleanValue())
         referenceCMPsCalculator.getNextEntryCMP(tempFinalICP);
      else
         entryCornerPoints.get(1).getFrameTupleIncludingFrame(tempFinalICP);
      tempFinalICP.changeFrame(finalDesiredCapturePointPositionToPack.getReferenceFrame());
      finalDesiredCapturePointPositionToPack.setByProjectionOntoXYPlane(tempFinalICP);
   }

   @Override
   public void getNextExitCMP(FramePoint entryCMPToPack)
   {
      referenceCMPsCalculator.getNextExitCMP(entryCMPToPack);
   }

   @Override
   public boolean isDone(double time)
   {
      return computeAndReturnTimeRemaining(time) <= 0.0;
   }

   @Override
   public boolean isOnExitCMP()
   {
      if (isDoubleSupport.getBooleanValue())
         return false;
      else
         return icpSingleSupportTrajectoryGenerator.isOnExitCMP();
   }
}
