package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.AbstractICPPlanner;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.FootstepAngularMomentumPredictor;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CMPGeneration.ReferenceCMPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoMGeneration.ReferenceCoMTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.ReferenceCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration.ReferenceICPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class SmoothCMPBasedICPPlanner extends AbstractICPPlanner
{
   private static final boolean VISUALIZE = false;
   private static final boolean debug = false;

   private static final double ZERO_TIME = 0.0;

   private boolean adjustICPForSingleSupport = false;
   private boolean adjustICPForInitialDoubleSupport = true;
   private boolean adjustICPForEachDoubleSupport = true;

   /** Desired velocity for the Center of Mass (CoM) */
   private final YoFrameVector3D desiredCoMVelocity = new YoFrameVector3D(namePrefix + "DesiredCoMVelocity", worldFrame, registry);
   /** Desired acceleration for the Center of Mass (CoM) */
   private final YoFrameVector3D desiredCoMAcceleration = new YoFrameVector3D(namePrefix + "DesiredCoMAcceleration", worldFrame, registry);
   /** Desired position for the Center of Pressure (CoP) */
   private final YoFramePoint3D desiredCoPPosition = new YoFramePoint3D(namePrefix + "DesiredCoPPosition", worldFrame, registry);
   /** Desired velocity for the Center of Pressure (CoP) */
   private final YoFrameVector3D desiredCoPVelocity = new YoFrameVector3D(namePrefix + "DesiredCoPVelocity", worldFrame, registry);

   /** Desired Centroidal Angular Momentum (CAM) */
   private final YoFrameVector3D desiredCentroidalAngularMomentum = new YoFrameVector3D(namePrefix + "DesiredCentroidalAngularMomentum", worldFrame, registry);
   /** Desired Centroidal Torque (CT) */
   private final YoFrameVector3D desiredCentroidalTorque = new YoFrameVector3D(namePrefix + "DesiredCentroidalTorque", worldFrame, registry);

   private final ReferenceCoPTrajectoryGenerator referenceCoPGenerator;
   private final ReferenceCMPTrajectoryGenerator referenceCMPGenerator;
   private final ReferenceICPTrajectoryGenerator referenceICPGenerator;
   private final ReferenceCoMTrajectoryGenerator referenceCoMGenerator;
   private final FootstepAngularMomentumPredictor angularMomentumGenerator;

   private final List<YoDouble> swingDurationShiftFractions = new ArrayList<>();
   private final YoDouble defaultSwingDurationShiftFraction;

   private static final double ICP_CORNER_POINT_SIZE = 0.002;
   private List<YoFramePointInMultipleFrames> icpPhaseEntryCornerPoints = new ArrayList<>();
   private List<YoFramePointInMultipleFrames> icpPhaseExitCornerPoints = new ArrayList<>();

   private final double robotMass;
   private final double gravityZ;

   private final FramePoint3D tempPoint = new FramePoint3D();

   private final YoFramePoint3D yoSingleSupportFinalCoM;
   private final FramePoint3D singleSupportFinalCoM = new FramePoint3D();

   private final int maxNumberOfICPCornerPointsVisualized = 20;

   private final YoBoolean areCoMDynamicsSatisfied;

   public SmoothCMPBasedICPPlanner(FullRobotModel fullRobotModel, BipedSupportPolygons bipedSupportPolygons,
                                   SideDependentList<? extends ContactablePlaneBody> contactableFeet, int maxNumberOfFootstepsToConsider,
                                   YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, double gravityZ)
   {
      this(fullRobotModel.getTotalMass(), bipedSupportPolygons, contactableFeet, maxNumberOfFootstepsToConsider, parentRegistry,
           yoGraphicsListRegistry, gravityZ);
   }

   public SmoothCMPBasedICPPlanner(double robotMass, BipedSupportPolygons bipedSupportPolygons,
                                   SideDependentList<? extends ContactablePlaneBody> contactableFeet, int maxNumberOfFootstepsToConsider,
                                   YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry, double gravityZ)
   {
      super(bipedSupportPolygons, maxNumberOfFootstepsToConsider);

      yoSingleSupportFinalCoM = new YoFramePoint3D(namePrefix + "SingleSupportFinalCoM", worldFrame, registry);

      this.gravityZ = gravityZ;
      defaultSwingDurationShiftFraction = new YoDouble(namePrefix + "DefaultSwingDurationShiftFraction", registry);

      this.robotMass = robotMass;

      ReferenceFrame[] framesToRegister = new ReferenceFrame[] {worldFrame, midFeetZUpFrame, soleZUpFrames.get(RobotSide.LEFT),
            soleZUpFrames.get(RobotSide.RIGHT)};

      for (int i = 0; i < maxNumberOfFootstepsToConsider; i++)
      {
         YoDouble swingDurationShiftFraction = new YoDouble(namePrefix + "SwingDurationShiftFraction" + i, registry);
         swingDurationShiftFractions.add(swingDurationShiftFraction);
      }

      for (int i = 0; i < maxNumberOfICPCornerPointsVisualized - 1; i++)
      {
         YoFramePointInMultipleFrames entryCornerPoint = new YoFramePointInMultipleFrames(namePrefix + "EntryCornerPoints" + i, registry, framesToRegister);
         icpPhaseEntryCornerPoints.add(entryCornerPoint);

         YoFramePointInMultipleFrames exitCornerPoint = new YoFramePointInMultipleFrames(namePrefix + "ExitCornerPoints" + i, registry, framesToRegister);
         icpPhaseExitCornerPoints.add(exitCornerPoint);
      }

      referenceCoPGenerator = new ReferenceCoPTrajectoryGenerator(namePrefix, maxNumberOfFootstepsToConsider, bipedSupportPolygons,
                                                                  contactableFeet, numberFootstepsToConsider, swingDurations, transferDurations, touchdownDurations,
                                                                  swingDurationAlphas, swingDurationShiftFractions, transferDurationAlphas, debug, registry);
      referenceCMPGenerator = new ReferenceCMPTrajectoryGenerator(namePrefix, maxNumberOfFootstepsToConsider, numberFootstepsToConsider, registry);

      referenceICPGenerator = new ReferenceICPTrajectoryGenerator(namePrefix, omega0, numberFootstepsToConsider, isInitialTransfer, debug, registry);

      referenceCoMGenerator = new ReferenceCoMTrajectoryGenerator(namePrefix, omega0, numberFootstepsToConsider, isInitialTransfer, isDoubleSupport, registry);

      angularMomentumGenerator = new FootstepAngularMomentumPredictor(namePrefix, omega0, debug, registry);

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
         //FIXME have the angular momentum parameters be passed into or as part of the ICP Planner parameters to the trajectory generator
         angularMomentumGenerator.initializeParameters(smoothCMPPlannerParameters, robotMass, gravityZ);
         defaultSwingDurationShiftFraction.set(smoothCMPPlannerParameters.getSwingDurationShiftFraction());
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
         YoFramePoint3D icpEntryCornerPointInWorld = icpPhaseEntryCornerPoints.get(i).buildUpdatedYoFramePointForVisualizationOnly();
         YoGraphicPosition icpEntryCornerPointsViz = new YoGraphicPosition("ICPEntryCornerPoints" + i, icpEntryCornerPointInWorld, ICP_CORNER_POINT_SIZE,
                                                                           YoAppearance.Blue(), GraphicType.SOLID_BALL);

         yoGraphicsList.add(icpEntryCornerPointsViz);
         artifactList.add(icpEntryCornerPointsViz.createArtifact());

         YoFramePoint3D icpExitCornerPointInWorld = icpPhaseExitCornerPoints.get(i).buildUpdatedYoFramePointForVisualizationOnly();
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
      angularMomentumGenerator.clear();

      for (int i = 0; i < swingDurations.size(); i++)
      {
         swingDurations.get(i).setToNaN();
         transferDurations.get(i).setToNaN();
         touchdownDurations.get(i).setToNaN();
         swingDurationAlphas.get(i).setToNaN();
         transferDurationAlphas.get(i).setToNaN();
         swingDurationShiftFractions.get(i).setToNaN();
      }
   }

   public void clearPlanWithoutClearingPlannedFootsteps()
   {
      referenceCoPGenerator.clearPlan();
      referenceCMPGenerator.reset();
      referenceICPGenerator.reset();
      angularMomentumGenerator.clear();
   }

   /** {@inheritDoc} */
   @Override
   public void addFootstepToPlan(Footstep footstep, FootstepTiming timing)
   {
      if (footstep == null)
         return;
      referenceCoPGenerator.addFootstepToPlan(footstep, timing);

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

      this.initialTime.set(initialTime);
      isInitialTransfer.set(isStanding.getBooleanValue());
      isStanding.set(true);
      isDoubleSupport.set(true);
      transferDurations.get(0).set(finalTransferDuration.getDoubleValue());
      transferDurationAlphas.get(0).set(finalTransferDurationAlpha.getDoubleValue());
      referenceICPGenerator.setInitialConditionsForAdjustment();
      updateTransferPlan(false);
   }

   /** {@inheritDoc} */
   @Override
   public void initializeForTransfer(double initialTime)
   {
      this.initialTime.set(initialTime);
      isDoubleSupport.set(true);
      isInitialTransfer.set(isStanding.getBooleanValue());
      isStanding.set(false);
      int numberOfFootstepRegistered = getNumberOfFootstepsRegistered();
      transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());
      transferDurationAlphas.get(numberOfFootstepRegistered).set(finalTransferDurationAlpha.getDoubleValue());
      referenceICPGenerator.setInitialConditionsForAdjustment();
      updateTransferPlan(true);
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
      isHoldingPosition.set(false);

      int numberOfFootstepRegistered = getNumberOfFootstepsRegistered();
      transferDurations.get(numberOfFootstepRegistered).set(finalTransferDuration.getDoubleValue());
      transferDurationAlphas.get(numberOfFootstepRegistered).set(finalTransferDurationAlpha.getDoubleValue());
      referenceICPGenerator.setInitialConditionsForAdjustment();
      updateSingleSupportPlan(true);
   }

   /** {@inheritDoc} */
   @Override
   public void computeFinalCoMPositionInSwing()
   {
      referenceCoMGenerator.getFinalCoMPositionInSwing(singleSupportFinalCoM);
      yoSingleSupportFinalCoM.set(singleSupportFinalCoM);
   }

   /** {@inheritDoc} */
   @Override
   protected void updateTransferPlan(boolean maintainContinuity)
   {
      clearPlanWithoutClearingPlannedFootsteps();
      RobotSide transferToSide = this.transferToSide.getEnumValue();
      if (transferToSide == null)
         transferToSide = RobotSide.LEFT;

      // TODO set up the CoP Generator to be able to only update the current Support Feet CMPs      
      referenceCoPGenerator.computeReferenceCoPsStartingFromDoubleSupport(isInitialTransfer.getBooleanValue(), transferToSide);
      referenceCMPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceICPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());

      referenceCoPGenerator.initializeForTransfer(ZERO_TIME);
      referenceICPGenerator.initializeForTransferFromCoPs(referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories());
      if(maintainContinuity && ((adjustICPForInitialDoubleSupport && isStanding.getBooleanValue()) || adjustICPForEachDoubleSupport))
         referenceICPGenerator.adjustDesiredTrajectoriesForInitialSmoothing();

      referenceCoMGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceCoMGenerator.initializeForTransfer(ZERO_TIME, referenceCoPGenerator.getTransferCoPTrajectories(),
                                                  referenceCoPGenerator.getSwingCoPTrajectories(),
                                                  referenceICPGenerator.getICPPositonFromCoPDesiredFinalList());

      angularMomentumGenerator.addFootstepCoPsToPlan(referenceCoPGenerator.getWaypoints(), referenceCoMGenerator.getCoMPositionDesiredInitialList(),
                                                     referenceCoMGenerator.getCoMPositionDesiredFinalList(),
                                                     referenceCoMGenerator.getCoMVelocityDesiredInitialList(),
                                                     referenceCoMGenerator.getCoMVelocityDesiredFinalList(),
                                                     referenceCoMGenerator.getCoMAccelerationDesiredInitialList(),
                                                     referenceCoMGenerator.getCoMAccelerationDesiredFinalList(),
                                                     referenceCoPGenerator.getNumberOfFootstepsRegistered());
      angularMomentumGenerator.computeReferenceAngularMomentumStartingFromDoubleSupport(isInitialTransfer.getBooleanValue());
      angularMomentumGenerator.initializeForDoubleSupport(ZERO_TIME, isStanding.getBooleanValue());

      referenceCMPGenerator.initializeForTransfer(ZERO_TIME, referenceCoPGenerator.getTransferCoPTrajectories(),
                                                  referenceCoPGenerator.getSwingCoPTrajectories(),
                                                  angularMomentumGenerator.getTransferAngularMomentumTrajectories(),
                                                  angularMomentumGenerator.getSwingAngularMomentumTrajectories());
      referenceICPGenerator.initializeForTransfer(ZERO_TIME, referenceCMPGenerator.getTransferCMPTrajectories(),
                                                  referenceCMPGenerator.getSwingCMPTrajectories());
      referenceCoMGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceCoMGenerator.initializeForTransfer(ZERO_TIME, referenceCMPGenerator.getTransferCMPTrajectories(),
                                                  referenceCMPGenerator.getSwingCMPTrajectories(),
                                                  referenceICPGenerator.getICPPositionDesiredFinalList());
      referenceICPGenerator.getICPPhaseEntryCornerPoints(icpPhaseEntryCornerPoints);
      referenceICPGenerator.getICPPhaseExitCornerPoints(icpPhaseExitCornerPoints);
      updateListeners();
      // TODO implement requested hold position
      // TODO implement is done walking
   }

   /** {@inheritDoc} */
   @Override
   protected void updateSingleSupportPlan(boolean maintainContinuity)
   {
      clearPlanWithoutClearingPlannedFootsteps();
      RobotSide supportSide = this.supportSide.getEnumValue();

      // TODO set up the CoP Generator to be able to only update the current Support Feet CMPs
      referenceCoPGenerator.computeReferenceCoPsStartingFromSingleSupport(supportSide);
      referenceCMPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceICPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());

      referenceCoPGenerator.initializeForSwing(ZERO_TIME);
      referenceICPGenerator.initializeForSwingFromCoPs(referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories());
      if(maintainContinuity && adjustICPForSingleSupport)
         referenceICPGenerator.adjustDesiredTrajectoriesForInitialSmoothing();

      referenceCoMGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceCoMGenerator.initializeForSwing(ZERO_TIME, referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories(),
                                               referenceICPGenerator.getICPPositonFromCoPDesiredFinalList());

      angularMomentumGenerator.addFootstepCoPsToPlan(referenceCoPGenerator.getWaypoints(), referenceCoMGenerator.getCoMPositionDesiredInitialList(),
                                                     referenceCoMGenerator.getCoMPositionDesiredFinalList(),
                                                     referenceCoMGenerator.getCoMVelocityDesiredInitialList(),
                                                     referenceCoMGenerator.getCoMVelocityDesiredFinalList(),
                                                     referenceCoMGenerator.getCoMAccelerationDesiredInitialList(),
                                                     referenceCoMGenerator.getCoMAccelerationDesiredFinalList(),
                                                     referenceCoPGenerator.getNumberOfFootstepsRegistered());
      angularMomentumGenerator.computeReferenceAngularMomentumStartingFromSingleSupport();
      angularMomentumGenerator.initializeForSingleSupport(ZERO_TIME);

      referenceCMPGenerator.initializeForSwing(ZERO_TIME, referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories(),
                                               angularMomentumGenerator.getTransferAngularMomentumTrajectories(),
                                               angularMomentumGenerator.getSwingAngularMomentumTrajectories());
      referenceICPGenerator.initializeForSwing(ZERO_TIME, referenceCMPGenerator.getTransferCMPTrajectories(), referenceCMPGenerator.getSwingCMPTrajectories());

      referenceCoMGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceCoMGenerator.initializeForSwing(ZERO_TIME, referenceCMPGenerator.getTransferCMPTrajectories(), referenceCMPGenerator.getSwingCMPTrajectories(),
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
      if(referenceCoPGenerator.getIsPlanAvailable())
      {
         timer.startMeasurement();

         timeInCurrentState.set(time - initialTime.getDoubleValue());
         timeInCurrentStateRemaining.set(getCurrentStateDuration() - timeInCurrentState.getDoubleValue());

         double timeInCurrentState = MathTools.clamp(this.timeInCurrentState.getDoubleValue(), 0.0, referenceCoPGenerator.getCurrentStateFinalTime());

         referenceICPGenerator.compute(timeInCurrentState);
         referenceCoMGenerator.compute(timeInCurrentState);
         referenceCoPGenerator.update(timeInCurrentState);
         referenceCMPGenerator.update(timeInCurrentState);
         angularMomentumGenerator.update(timeInCurrentState);

         referenceCoPGenerator.getDesiredCenterOfPressure(desiredCoPPosition, desiredCoPVelocity);
         referenceCMPGenerator.getLinearData(desiredCMPPosition, desiredCMPVelocity);
         referenceICPGenerator.getLinearData(desiredICPPosition, desiredICPVelocity, desiredICPAcceleration);
         referenceCoMGenerator.getLinearData(desiredCoMPosition, desiredCoMVelocity, desiredCoMAcceleration);
         angularMomentumGenerator.getDesiredAngularMomentum(desiredCentroidalAngularMomentum, desiredCentroidalTorque);
         decayDesiredVelocityIfNeeded();

         if (debug)
            checkCoMDynamics(desiredCoMVelocity, desiredICPPosition, desiredCoMPosition);

         timer.stopMeasurement();
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
   }

   public void updateListeners()
   {
      referenceCoPGenerator.updateListeners();
   }

   public List<CoPPointsInFoot> getCoPWaypoints()
   {
      return referenceCoPGenerator.getWaypoints();
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
         tempFinalICP.set(getFinalDesiredCapturePointPositions().get(referenceCMPGenerator.getSwingCMPTrajectories().get(0).getNumberOfSegments() - 1));
         tempFinalICP.changeFrame(finalDesiredCapturePointPositionToPack.getReferenceFrame());
         finalDesiredCapturePointPositionToPack.set(tempFinalICP);
      }
      else
      {
         tempFinalICP.set(getFinalDesiredCapturePointPositions().get(referenceCMPGenerator.getTransferCMPTrajectories().get(0).getNumberOfSegments() - 1));
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

   public List<? extends FramePoint3DReadOnly> getInitialDesiredCapturePointPositions()
   {
      return referenceICPGenerator.getICPPositionDesiredInitialList();
   }

   public List<? extends FramePoint3DReadOnly> getFinalDesiredCapturePointPositions()
   {
      return referenceICPGenerator.getICPPositionDesiredFinalList();
   }

   public List<? extends FramePoint3DReadOnly> getInitialDesiredCenterOfMassPositions()
   {
      return referenceCoMGenerator.getCoMPositionDesiredInitialList();
   }

   public List<? extends FramePoint3DReadOnly> getFinalDesiredCenterOfMassPositions()
   {
      return referenceCoMGenerator.getCoMPositionDesiredFinalList();
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
      copPointsInFoot.get(copPointsInFoot.getCoPPointList().size() - 1).getPosition(exitCMPToPack);
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

   public void getPredictedCenterOfMassPosition(YoFramePoint3D estimatedCenterOfMassPositionToPack, double time)
   {
      angularMomentumGenerator.getPredictedCenterOfMassPosition(estimatedCenterOfMassPositionToPack, time);
   }

   public void getPredictedSwingFootPosition(YoFramePoint3D predictedSwingFootPositionToPack, double time)
   {
      angularMomentumGenerator.getPredictedFootPosition(predictedSwingFootPositionToPack, time);
   }

   public int getTotalNumberOfSegments()
   {
      return referenceICPGenerator.getTotalNumberOfSegments();
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

   private void checkCoMDynamics(FrameVector3DReadOnly comVelocityDesiredCurrent, FramePoint3DReadOnly icpPositionDesiredCurrent, FramePoint3DReadOnly comPositionDesiredCurrent)
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

   public void getDesiredCenterOfMassVelocity(FrameVector3D desiredCenterOfMassVelocityToPack)
   {
      desiredCenterOfMassVelocityToPack.setIncludingFrame(desiredCoMVelocity);
   }

   public void getDesiredCenterOfMassAcceleration(YoFrameVector3D desiredCenterOfMassAccelerationToPack)
   {
      desiredCenterOfMassAccelerationToPack.set(desiredCoMAcceleration);
   }

   public void getDesiredCenterOfMassAcceleration(FrameVector3D desiredCenterOfMassAccelerationToPack)
   {
      desiredCenterOfMassAccelerationToPack.setIncludingFrame(desiredCoMAcceleration);
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

   public void getDesiredCenterOfPressurePosition(YoFramePoint3D desiredCenterOfPressurePositionToPack)
   {
      desiredCenterOfPressurePositionToPack.set(desiredCoPPosition);
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

   public void getDesiredCentroidalAngularMomentum(FrameVector3D desiredCentroidalAngularMomentumToPack)
   {
      desiredCentroidalAngularMomentumToPack.setIncludingFrame(desiredCentroidalAngularMomentum);
   }

   public void getDesiredCentroidalTorque(YoFrameVector3D desiredCentroidalTorqueToPack)
   {
      desiredCentroidalTorqueToPack.set(desiredCentroidalTorque);
   }

   public void getDesiredCentroidalTorque(FrameVector3D desiredCentroidalTorqueToPack)
   {
      desiredCentroidalTorqueToPack.setIncludingFrame(desiredCentroidalTorque);
   }


   void ensureContinuityEnteringSingleSupport(boolean ensureContinuity)
   {
      this.adjustICPForSingleSupport = ensureContinuity;
   }

   void ensureContinuityEnteringFirstTransfer(boolean ensureContinuity)
   {
      this.adjustICPForInitialDoubleSupport = ensureContinuity;
   }

   void ensureContinuityEnteringEachTransfer(boolean ensureContinuity)
   {
      this.adjustICPForEachDoubleSupport = ensureContinuity;
   }
}
