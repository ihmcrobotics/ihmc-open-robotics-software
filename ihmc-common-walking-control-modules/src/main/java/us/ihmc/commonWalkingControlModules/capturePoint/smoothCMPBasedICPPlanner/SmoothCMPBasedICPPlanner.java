package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.AbstractICPPlanner;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.AngularMomentumTrajectoryMultiplexer;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CMPGeneration.ReferenceCMPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoMGeneration.ReferenceCoMTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.FootstepData;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.ReferenceCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration.ReferenceICPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commonWalkingControlModules.messageHandlers.MomentumTrajectoryHandler;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameTuple3DBasics;
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
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;
import us.ihmc.yoVariables.variable.frameObjects.YoMutableFramePoint3D;

public class SmoothCMPBasedICPPlanner extends AbstractICPPlanner
{
   private static final boolean VISUALIZE = true;
   private static final boolean debug = false;
   private static final int maxNumberOfFootstepsToConsider = 4;

   private static final RobotSide defaultTransferToSide = RobotSide.LEFT;

   private static final double ZERO_TIME = 0.0;

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
   private final RecyclingArrayList<FootstepData> upcomingFootstepsData = new RecyclingArrayList<>(maxNumberOfFootstepsToConsider, FootstepData.class);

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
                                   SideDependentList<? extends ContactablePlaneBody> contactableFeet, int maxNumberOfFootstepsToConsider,
                                   MomentumTrajectoryHandler momentumTrajectoryHandler, YoDouble yoTime, YoVariableRegistry parentRegistry,
                                   YoGraphicsListRegistry yoGraphicsListRegistry, double gravityZ)
   {

      this(fullRobotModel.getTotalMass(), bipedSupportPolygons, contactableFeet, maxNumberOfFootstepsToConsider, momentumTrajectoryHandler, yoTime,
           parentRegistry, yoGraphicsListRegistry, gravityZ);
   }

   public SmoothCMPBasedICPPlanner(double robotMass, BipedSupportPolygons bipedSupportPolygons,
                                   SideDependentList<? extends ContactablePlaneBody> contactableFeet, int maxNumberOfFootstepsToConsider,
                                   MomentumTrajectoryHandler momentumTrajectoryHandler, YoDouble yoTime, YoVariableRegistry parentRegistry,
                                   YoGraphicsListRegistry yoGraphicsListRegistry, double gravityZ)
   {
      super(bipedSupportPolygons, maxNumberOfFootstepsToConsider);

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
                                                                  numberFootstepsToConsider, swingDurations, transferDurations, touchdownDurations,
                                                                  swingDurationAlphas, swingDurationShiftFractions, transferDurationAlphas, debug,
                                                                  numberOfUpcomingFootsteps, upcomingFootstepsData, registry);
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
   }

   @Override
   public void initializeParameters(ICPPlannerParameters icpPlannerParameters)
   {
      super.initializeParameters(icpPlannerParameters);

      if (icpPlannerParameters instanceof SmoothCMPPlannerParameters)
      {
         SmoothCMPPlannerParameters smoothCMPPlannerParameters = (SmoothCMPPlannerParameters) icpPlannerParameters;
         numberFootstepsToConsider.set(smoothCMPPlannerParameters.getNumberOfFootstepsToConsider());

         referenceCoPGenerator.initializeParameters(smoothCMPPlannerParameters);
         referenceCMPGenerator.setGroundReaction(robotMass * gravityZ);
         angularMomentumTrajectoryGenerator.initializeParameters(smoothCMPPlannerParameters, robotMass, gravityZ);
         defaultSwingDurationShiftFraction.set(smoothCMPPlannerParameters.getSwingDurationShiftFraction());

         adjustPlanForSSContinuity.set(smoothCMPPlannerParameters.adjustCoPPlanForSingleSupportContinuity());
         adjustPlanForDSContinuity.set(smoothCMPPlannerParameters.adjustEveryCoPPlanForDoubleSupportContinuity());
         adjustPlanForInitialDSContinuity.set(smoothCMPPlannerParameters.adjustInitialCoPPlanForDoubleSupportContinuity());
         adjustPlanForStandingContinuity.set(smoothCMPPlannerParameters.adjustCoPPlanForStandingContinuity());

         doContinuousReplanningForStanding.set(smoothCMPPlannerParameters.doContinuousReplanningForStanding());
         doContinuousReplanningForTransfer.set(smoothCMPPlannerParameters.doContinuousReplanningForTransfer());
         doContinuousReplanningForSwing.set(smoothCMPPlannerParameters.doContinuousReplanningForSwing());
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
         touchdownDurations.get(i).setToNaN();
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
         PrintTools.warn("Received null footstep, ignoring");
         return;
      }
      else if (timing == null)
      {
         PrintTools.warn("Received null step timing, ignoring");
         return;
      }
      else if (footstep.getSoleReferenceFrame().getTransformToRoot().containsNaN())
      {
         PrintTools.warn("Received bad footstep: " + footstep + ", ignoring");
         return;
      }
      else
      {
         upcomingFootstepsData.add().set(footstep, timing);
         numberOfUpcomingFootsteps.increment();
      }

      int footstepIndex = referenceCoPGenerator.getNumberOfFootstepsRegistered() - 1;

      double swingDuration = timing.getSwingTime();
      double touchdownDuration = timing.getTouchdownDuration();
      double transferTime = timing.getTransferTime();

      if (!Double.isFinite(swingDuration) || swingDuration < 0.0)
         swingDuration = 1.0;

      if (!Double.isFinite(touchdownDuration) || touchdownDuration < 0.0)
         touchdownDuration = 0.0;

      if (!Double.isFinite(transferTime) || transferTime < 0.0)
         transferTime = 1.0;

      swingDurations.get(footstepIndex).set(swingDuration);
      touchdownDurations.get(footstepIndex).set(touchdownDuration);
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

   /** {@inheritDoc} */
   @Override
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

   /** {@inheritDoc} */
   @Override
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
      super.holdCurrentICP(icpPositionToHold);
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
