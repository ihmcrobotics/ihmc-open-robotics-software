package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.AbstractICPPlanner;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.AMGeneration.FootstepAngularMomentumPredictor;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CMPGeneration.ReferenceCMPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoMGeneration.ReferenceCoMTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPPointsInFoot;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.CoPTrajectory;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.CoPGeneration.ReferenceCoPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration.ReferenceICPTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.configurations.ICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.SmoothCMPPlannerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
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
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class SmoothCMPBasedICPPlanner extends AbstractICPPlanner
{
   private static final boolean VISUALIZE = false;
   private static final boolean debug = false;

   private static final double ZERO_TIME = 0.0;

   private static final boolean adjustICPForSingleSupport = true;
   private static final boolean adjustICPForDoubleSupport = true;

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

   private final YoFramePoint yoSingleSupportFinalCoM;
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

      yoSingleSupportFinalCoM = new YoFramePoint(namePrefix + "SingleSupportFinalCoM", worldFrame, registry);

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
         YoFramePoint icpEntryCornerPointInWorld = icpPhaseEntryCornerPoints.get(i).buildUpdatedYoFramePointForVisualizationOnly();
         YoGraphicPosition icpEntryCornerPointsViz = new YoGraphicPosition("ICPEntryCornerPoints" + i, icpEntryCornerPointInWorld, ICP_CORNER_POINT_SIZE,
                                                                           YoAppearance.Blue(), GraphicType.SOLID_BALL);

         yoGraphicsList.add(icpEntryCornerPointsViz);
         artifactList.add(icpEntryCornerPointsViz.createArtifact());

         YoFramePoint icpExitCornerPointInWorld = icpPhaseExitCornerPoints.get(i).buildUpdatedYoFramePointForVisualizationOnly();
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
   /** {@inheritDoc} */
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

   @Override
   /** {@inheritDoc} */
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
   
   @Override
   /** {@inheritDoc} */
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
      updateTransferPlan();
   }

   @Override
   /** {@inheritDoc} */
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
      updateTransferPlan();
   }

   @Override
   /** {@inheritDoc} */
   public void computeFinalCoMPositionInTransfer()
   {
      referenceCoMGenerator.getFinalCoMPositionInTransfer(singleSupportFinalCoM);
      yoSingleSupportFinalCoM.set(singleSupportFinalCoM);
   }

   @Override
   /** {@inheritDoc} */
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
      updateSingleSupportPlan();
   }

   @Override
   /** {@inheritDoc} */
   public void computeFinalCoMPositionInSwing()
   {
      referenceCoMGenerator.getFinalCoMPositionInSwing(singleSupportFinalCoM);
      yoSingleSupportFinalCoM.set(singleSupportFinalCoM);
   }

   @Override
   /** {@inheritDoc} */
   protected void updateTransferPlan()
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
      if(adjustICPForDoubleSupport)
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

   @Override
   /** {@inheritDoc} */
   protected void updateSingleSupportPlan()
   {
      clearPlanWithoutClearingPlannedFootsteps();
      RobotSide supportSide = this.supportSide.getEnumValue();

      // TODO set up the CoP Generator to be able to only update the current Support Feet CMPs
      referenceCoPGenerator.computeReferenceCoPsStartingFromSingleSupport(supportSide);
      referenceCMPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());
      referenceICPGenerator.setNumberOfRegisteredSteps(referenceCoPGenerator.getNumberOfFootstepsRegistered());

      referenceCoPGenerator.initializeForSwing(ZERO_TIME);
      referenceICPGenerator.initializeForSwingFromCoPs(referenceCoPGenerator.getTransferCoPTrajectories(), referenceCoPGenerator.getSwingCoPTrajectories());
      if(adjustICPForSingleSupport)
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
      updateListeners();
   }

   private void printCoPTrajectories()
   {
      List<? extends CoPTrajectory> transferCoPTrajectory = referenceCoPGenerator.getTransferCoPTrajectories();
      List<? extends CoPTrajectory> swingCoPTrajectory = referenceCoPGenerator.getSwingCoPTrajectories();

      for (int i = 0; i < referenceCoPGenerator.getNumberOfFootstepsRegistered(); i++)
      {
         PrintTools.debug(transferCoPTrajectory.get(i).toString());
         PrintTools.debug(swingCoPTrajectory.get(i).toString());
      }
      PrintTools.debug(transferCoPTrajectory.get(referenceCoPGenerator.getNumberOfFootstepsRegistered()).toString());
   }

   @Override
   /** {@inheritDoc} */
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
            checkCoMDynamics(desiredCoMVelocity.getFrameVectorCopy(), desiredICPPosition.getFramePointCopy(), desiredCoMPosition.getFramePointCopy());

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

   /** {@inheritDoc} */
   public void updateListeners()
   {
      referenceCoPGenerator.updateListeners();
   }

   /** {@inheritDoc} */
   public List<CoPPointsInFoot> getCoPWaypoints()
   {
      return referenceCoPGenerator.getWaypoints();
   }

   private final FramePoint3D tempFinalICP = new FramePoint3D();

   @Override
   /** {@inheritDoc} */
   public void getFinalDesiredCapturePointPosition(FramePoint3D finalDesiredCapturePointPositionToPack)
   {
      if (isStanding.getBooleanValue())
      {
         // TODO: replace by CMP generator
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

   @Override
   /** {@inheritDoc} */
   public void getFinalDesiredCapturePointPosition(YoFramePoint2d finalDesiredCapturePointPositionToPack)
   {
      getFinalDesiredCapturePointPosition(tempFinalICP);
      finalDesiredCapturePointPositionToPack.setByProjectionOntoXYPlane(tempFinalICP);
   }

   public List<FramePoint3D> getInitialDesiredCapturePointPositions()
   {
      return referenceICPGenerator.getICPPositionDesiredInitialList();
   }

   public List<FramePoint3D> getFinalDesiredCapturePointPositions()
   {
      return referenceICPGenerator.getICPPositionDesiredFinalList();
   }

   public List<FramePoint3D> getInitialDesiredCenterOfMassPositions()
   {
      return referenceCoMGenerator.getCoMPositionDesiredInitialList();
   }

   public List<FramePoint3D> getFinalDesiredCenterOfMassPositions()
   {
      return referenceCoMGenerator.getCoMPositionDesiredFinalList();
   }

   private final FramePoint3D tempFinalCoM = new FramePoint3D();

   @Override
   /** {@inheritDoc} */
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

   @Override
   /** {@inheritDoc} */
   public void getNextExitCMP(FramePoint3D exitCMPToPack)
   {
      //CMPTrajectory nextSwingTrajectory = referenceCMPGenerator.getSwingCMPTrajectories().get(0);
      //nextSwingTrajectory.getExitCMPLocation(exitCMPToPack);

      List<CoPPointsInFoot> plannedCoPWaypoints = referenceCoPGenerator.getWaypoints();
      CoPPointsInFoot copPointsInFoot = plannedCoPWaypoints.get(1);
      copPointsInFoot.get(copPointsInFoot.getCoPPointList().size() - 1).getPosition(exitCMPToPack);
   }

   @Override
   /** {@inheritDoc} */
   public boolean isOnExitCMP()
   {
      return referenceCoPGenerator.isOnExitCoP();
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
      return referenceCoPGenerator.getNumberOfFootstepsRegistered();
   }

   public void getPredictedCenterOfMassPosition(YoFramePoint estimatedCenterOfMassPositionToPack, double time)
   {
      angularMomentumGenerator.getPredictedCenterOfMassPosition(estimatedCenterOfMassPositionToPack, time);
   }

   public void getPredictedSwingFootPosition(YoFramePoint predictedSwingFootPositionToPack, double time)
   {
      angularMomentumGenerator.getPredictedFootPosition(predictedSwingFootPositionToPack, time);
   }

   public int getTotalNumberOfSegments()
   {
      return referenceICPGenerator.getTotalNumberOfSegments();
   }

   @Override
   /** {@inheritDoc} */
   public void holdCurrentICP(FramePoint3D icpPositionToHold)
   {
      super.holdCurrentICP(icpPositionToHold);
      // Asking the CoP and ICP to be the same since we assume that holds can be requested in static conditions only
      referenceCoPGenerator.holdPosition(icpPositionToHold);
   }

   private void checkCoMDynamics(FrameVector3D comVelocityDesiredCurrent, FramePoint3D icpPositionDesiredCurrent, FramePoint3D comPositionDesiredCurrent)
   {
      FramePoint3D comVelocityDynamicsCurrent = icpPositionDesiredCurrent;
      comVelocityDynamicsCurrent.sub(comPositionDesiredCurrent);
      comVelocityDynamicsCurrent.scale(omega0.getDoubleValue());

      areCoMDynamicsSatisfied.set(comVelocityDesiredCurrent.epsilonEquals(comVelocityDynamicsCurrent, 10e-5));
   }

   public void setDefaultPhaseTimes(double defaultSwingTime, double defaultTransferTime)
   {
      referenceCoPGenerator.setDefaultPhaseTimes(defaultSwingTime, defaultTransferTime);
   }
}
