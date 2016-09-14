package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ReferenceCentroidalMomentumPivotLocationsCalculator;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator.CapturePointTools;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.geometry.*;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;

import java.util.ArrayList;

public class ICPOptimizationController
{
   private static final boolean VISUALIZE = true;
   private static final boolean referenceFromNewCMP = true;

   private static final String namePrefix = "icpOptimizationCalculator";
   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final IntegerYoVariable numberOfFootstepsToConsider = new IntegerYoVariable("numberOfFootstepsToConsider", registry);

   private final BooleanYoVariable useTwoCMPsInControl = new BooleanYoVariable("useTwoCMPsInControl", registry);
   private final BooleanYoVariable useFeedback = new BooleanYoVariable("useFeedback", registry);
   private final BooleanYoVariable useStepAdjustment = new BooleanYoVariable("useStepAdjustment", registry);
   private final BooleanYoVariable useFootstepRegularization = new BooleanYoVariable("useFootstepRegularization", registry);
   private final BooleanYoVariable useFeedbackRegularization = new BooleanYoVariable("useFeedbackRegularization", registry);
   private final BooleanYoVariable useFeedbackWeightHardening = new BooleanYoVariable("useFeedbackWeightHardening", registry);

   private final BooleanYoVariable scaleStepRegularizationWeightWithTime = new BooleanYoVariable("scaleStepRegularizationWeightWithTime", registry);
   private final BooleanYoVariable scaleFeedbackWeightWithGain = new BooleanYoVariable("scaleFeedbackWeightWithGain", registry);
   private final BooleanYoVariable scaleUpcomingStepWeights = new BooleanYoVariable("scaleUpcomingStepWeights", registry);

   private final BooleanYoVariable isStanding = new BooleanYoVariable(yoNamePrefix + "IsStanding", registry);
   private final BooleanYoVariable isInTransfer = new BooleanYoVariable(yoNamePrefix + "IsInTransfer", registry);
   private final BooleanYoVariable isInTransferEntry = new BooleanYoVariable(yoNamePrefix + "IsInTransferEntry", registry);
   private final BooleanYoVariable isInitialTransfer = new BooleanYoVariable(yoNamePrefix + "IsInitialTransfer", registry);

   private final DoubleYoVariable doubleSupportDuration = new DoubleYoVariable(yoNamePrefix + "DoubleSupportDuration", registry);
   private final DoubleYoVariable singleSupportDuration = new DoubleYoVariable(yoNamePrefix + "SingleSupportDuration", registry);
   private final DoubleYoVariable initialDoubleSupportDuration = new DoubleYoVariable(yoNamePrefix + "InitialTransferDuration", registry);
   private final DoubleYoVariable exitCMPDurationInPercentOfStepTime = new DoubleYoVariable("timeSpentOnExitCMPInPercentOfStepTime", registry);
   private final DoubleYoVariable doubleSupportSplitFraction = new DoubleYoVariable("doubleSupportSplitFraction", registry);
   private final BooleanYoVariable stepTimesInitialized = new BooleanYoVariable("stepTimesInitialized", registry);

   private final EnumYoVariable<RobotSide> transferToSide = new EnumYoVariable<>("controllerTransferToSide", registry, RobotSide.class, true);
   private final EnumYoVariable<RobotSide> supportSide = new EnumYoVariable<>("controllerSupportSide", registry, RobotSide.class, true);

   private final DoubleYoVariable initialTime = new DoubleYoVariable("initialTime", registry);
   private final DoubleYoVariable timeInCurrentState = new DoubleYoVariable("timeInCurrentState", registry);
   private final DoubleYoVariable timeRemainingInState = new DoubleYoVariable("timeRemainingInState", registry);

   private final YoFramePoint2d controllerCurrentICP = new YoFramePoint2d("controllerCurrentICP", worldFrame, registry);
   private final YoFramePoint2d controllerDesiredICP = new YoFramePoint2d("controllerDesiredICP", worldFrame, registry);
   private final YoFrameVector2d controllerDesiredICPVelocity = new YoFrameVector2d("controllerDesiredICPVelocity", worldFrame, registry);
   private final YoFramePoint2d controllerPerfectCMP = new YoFramePoint2d("controllerPerfectCMP", worldFrame, registry);

   private final YoFramePoint2d controllerFeedbackCMP = new YoFramePoint2d("controllerFeedbackCMP", worldFrame, registry);
   private final YoFrameVector2d controllerFeedbackCMPDelta = new YoFrameVector2d("controllerFeedbackCMPDelta", worldFrame, registry);

   private final DoubleYoVariable controllerCostToGo = new DoubleYoVariable("costToGo", registry);
   private final DoubleYoVariable controllerFootstepCostToGo = new DoubleYoVariable("footstepCostToGo", registry);
   private final DoubleYoVariable controllerFootstepRegularizationCostToGo = new DoubleYoVariable("footstepRegularizationCostToGo", registry);
   private final DoubleYoVariable controllerFeedbackCostToGo = new DoubleYoVariable("feedbackCostToGo", registry);
   private final DoubleYoVariable controllerFeedbackRegularizationCostToGo = new DoubleYoVariable("feedbackRegularizationCostToGo", registry);
   private final DoubleYoVariable controllerDynamicRelaxationCostToGo = new DoubleYoVariable("dynamicRelaxationCostToGo", registry);

   private final YoFramePoint2d stanceEntryCMP = new YoFramePoint2d("stanceEntryCMP", worldFrame, registry);
   private final YoFramePoint2d stanceExitCMP = new YoFramePoint2d("stanceExitCMP", worldFrame, registry);
   private final YoFramePoint2d previousStanceExitCMP = new YoFramePoint2d("previousStanceExitCMP", worldFrame, registry);
   private final YoFramePoint2d stanceCMPProjection = new YoFramePoint2d("stanceCMPProjection", worldFrame, registry);

   private final YoFramePoint2d finalICP = new YoFramePoint2d("finalICP", worldFrame, registry);
   private final YoFramePoint2d finalICPRecursion = new YoFramePoint2d("finalICPRecursion", worldFrame, registry);
   private final YoFramePoint2d cmpOffsetRecursionEffect = new YoFramePoint2d("cmpOffsetRecursionEffect", worldFrame, registry);

   private final YoFramePoint2d actualEndOfStateICP = new YoFramePoint2d("actualEndOfStateICP", worldFrame, registry);

   private final YoFramePoint2d nominalEndOfStateICP = new YoFramePoint2d("nominalEndOfStateICP", worldFrame, registry);
   private final YoFramePoint2d nominalBeginningOfStateICP = new YoFramePoint2d("nominalBeginningOfStateICP", worldFrame, registry);
   private final YoFramePoint2d nominalReferenceICP = new YoFramePoint2d("nominalReferenceICP", worldFrame, registry);
   private final YoFrameVector2d nominalReferenceICPVelocity = new YoFrameVector2d("nominalReferenceICPVelocity", worldFrame, registry);
   private final YoFramePoint2d nominalReferenceCMP = new YoFramePoint2d("nominalReferenceCMP", worldFrame, registry);

   private final YoFramePoint2d controllerReferenceICP = new YoFramePoint2d("controllerReferenceICP", worldFrame, registry);
   private final YoFrameVector2d controllerReferenceICPVelocity = new YoFrameVector2d("controllerReferenceICPVelocity", worldFrame, registry);
   private final YoFramePoint2d controllerReferenceCMP = new YoFramePoint2d("controllerReferenceCMP", worldFrame, registry);

   private final ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();
   private final ArrayList<YoFramePoint2d> upcomingFootstepLocations = new ArrayList<>();
   private final ArrayList<FrameVector2d> entryOffsets = new ArrayList<>();
   private final ArrayList<FrameVector2d> exitOffsets = new ArrayList<>();
   private final ArrayList<YoFrameVector2d> yoEntryOffsets = new ArrayList<>();
   private final ArrayList<YoFrameVector2d> yoExitOffsets = new ArrayList<>();
   private final ArrayList<YoFramePoint2d> footstepSolutions = new ArrayList<>();

   private final DoubleYoVariable footstepWeight = new DoubleYoVariable("footstepWeight", registry);
   private final DoubleYoVariable footstepRegularizationWeight = new DoubleYoVariable("footstepRegularizationWeight", registry);
   private final DoubleYoVariable feedbackWeight = new DoubleYoVariable("feedbackWeight", registry);
   private final DoubleYoVariable feedbackRegularizationWeight = new DoubleYoVariable("feedbackRegularizationWeight", registry);
   private final DoubleYoVariable scaledFootstepRegularizationWeight = new DoubleYoVariable("scaledFootstepRegularizationWeight", registry);
   private final DoubleYoVariable scaledFeedbackWeight = new DoubleYoVariable("scaledFeedbackWeight", registry);
   private final DoubleYoVariable feedbackGain = new DoubleYoVariable("feedbackGain", registry);
   private final DoubleYoVariable dynamicRelaxationWeight = new DoubleYoVariable("dynamicRelaxationWeight", registry);

   private final DoubleYoVariable maxCMPExitForward = new DoubleYoVariable("maxCMPExitForward", registry);
   private final DoubleYoVariable maxCMPExitSideways = new DoubleYoVariable("maxCMPExitSideways", registry);

   private final DoubleYoVariable omega;

   private final ICPOptimizationSolver solver;
   private final FootstepRecursionMultiplierCalculator footstepRecursionMultiplierCalculator;
   private final ReferenceCentroidalMomentumPivotLocationsCalculator referenceCMPsCalculator;

   private final CapturePointPlannerParameters icpPlannerParameters;
   private final ICPOptimizationParameters icpOptimizationParameters;
   private final BipedSupportPolygons bipedSupportPolygons;

   private final int maximumNumberOfFootstepsToConsider;

   private boolean localUseTwoCMPs;
   private boolean localUseFeedback;
   private boolean localUseFeedbackRegularization;
   private boolean localUseFeedbackWeightHardening;
   private boolean localUseStepAdjustment;
   private boolean localUseFootstepRegularization;

   private boolean localScaleUpcomingStepWeights;

   public ICPOptimizationController(CapturePointPlannerParameters icpPlannerParameters, ICPOptimizationParameters icpOptimizationParameters,
                                    BipedSupportPolygons bipedSupportPolygons, SideDependentList<? extends ContactablePlaneBody> contactableFeet, DoubleYoVariable omega,
                                    YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpPlannerParameters = icpPlannerParameters;
      this.icpOptimizationParameters = icpOptimizationParameters;
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.omega = omega;

      maximumNumberOfFootstepsToConsider = icpOptimizationParameters.getMaximumNumberOfFootstepsToConsider();
      numberOfFootstepsToConsider.set(icpOptimizationParameters.numberOfFootstepsToConsider());

      initialDoubleSupportDuration.set(icpPlannerParameters.getDoubleSupportInitialTransferDuration());

      int totalVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
         totalVertices += contactableFeet.get(robotSide).getTotalNumberOfContactPoints();

      solver = new ICPOptimizationSolver(icpOptimizationParameters, totalVertices, registry);
      referenceCMPsCalculator = new ReferenceCentroidalMomentumPivotLocationsCalculator(namePrefix, bipedSupportPolygons, contactableFeet,
            maximumNumberOfFootstepsToConsider, registry);
      referenceCMPsCalculator.initializeParameters(icpPlannerParameters);

      useTwoCMPsInControl.set(icpPlannerParameters.useTwoCMPsPerSupport());
      useFeedback.set(icpOptimizationParameters.useFeedback());
      useStepAdjustment.set(icpOptimizationParameters.useStepAdjustment());
      useFootstepRegularization.set(icpOptimizationParameters.useFootstepRegularization());
      useFeedbackRegularization.set(icpOptimizationParameters.useFeedbackRegularization());
      useFeedbackWeightHardening.set(icpOptimizationParameters.useFeedbackWeightHardening());

      scaleStepRegularizationWeightWithTime.set(icpOptimizationParameters.scaleStepRegularizationWeightWithTime());
      scaleFeedbackWeightWithGain.set(icpOptimizationParameters.scaleFeedbackWeightWithGain());
      scaleUpcomingStepWeights.set(icpOptimizationParameters.scaleUpcomingStepWeights());

      // todo set the regularization as a function of control dt
      footstepWeight.set(icpOptimizationParameters.getFootstepWeight());
      footstepRegularizationWeight.set(icpOptimizationParameters.getFootstepRegularizationWeight());
      feedbackWeight.set(icpOptimizationParameters.getFeedbackWeight());
      feedbackRegularizationWeight.set(icpOptimizationParameters.getFeedbackRegularizationWeight());
      feedbackGain.set(icpOptimizationParameters.getFeedbackGain());
      dynamicRelaxationWeight.set(icpOptimizationParameters.getDynamicRelaxationWeight());

      maxCMPExitForward.set(icpOptimizationParameters.getMaxCMPExitForward());
      maxCMPExitSideways.set(icpOptimizationParameters.getMaxCMPExitSideways());

      exitCMPDurationInPercentOfStepTime.set(icpPlannerParameters.getTimeSpentOnExitCMPInPercentOfStepTime());
      doubleSupportSplitFraction.set(icpPlannerParameters.getDoubleSupportSplitFraction());

      footstepRecursionMultiplierCalculator = new FootstepRecursionMultiplierCalculator(icpPlannerParameters, exitCMPDurationInPercentOfStepTime,
                                                                                        doubleSupportSplitFraction, omega, maximumNumberOfFootstepsToConsider, registry);

      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
      {
         entryOffsets.add(new FrameVector2d(worldFrame));
         exitOffsets.add(new FrameVector2d(worldFrame));
         yoEntryOffsets.add(new YoFrameVector2d("entryOffset" + i, worldFrame, registry));
         yoExitOffsets.add(new YoFrameVector2d("exitOffset" + i, worldFrame, registry));
         upcomingFootstepLocations.add(new YoFramePoint2d("upcomingFootstepLocation" + i, worldFrame, registry));
         footstepSolutions.add(new YoFramePoint2d("footstepSolutionLocation" + i, worldFrame, registry));
      }

      if (yoGraphicsListRegistry != null && VISUALIZE)
      {
         String name = "stanceCMPPoints";
         YoGraphicPosition previousExitCMP = new YoGraphicPosition("previousExitCMP", previousStanceExitCMP, 0.01, YoAppearance.Red(), GraphicType.SQUARE);
         YoGraphicPosition entryCMP = new YoGraphicPosition("entryCMP", stanceEntryCMP, 0.01, YoAppearance.Red(), GraphicType.SQUARE);
         YoGraphicPosition exitCMP = new YoGraphicPosition("exitCMP", stanceExitCMP, 0.01, YoAppearance.Red(), GraphicType.SQUARE);

         YoGraphicPosition beginningOfStateICP = new YoGraphicPosition("nominalBeginningOfStateICP", nominalBeginningOfStateICP, 0.005, YoAppearance.Blue(), GraphicType.SOLID_BALL);
         YoGraphicPosition actualEndOfStateICP = new YoGraphicPosition("actualEndOfStateICP", this.actualEndOfStateICP, 0.005, YoAppearance.Aquamarine(), GraphicType.SOLID_BALL);

         YoGraphicPosition nominalReferenceICP = new YoGraphicPosition("nominalReferenceICP", this.nominalReferenceICP, 0.01, YoAppearance.LightYellow(), GraphicType.BALL);
         YoGraphicPosition nominalEndOfStateICP = new YoGraphicPosition("nominalEndOfStateICP", this.nominalEndOfStateICP, 0.005, YoAppearance.Green(), GraphicType.SOLID_BALL);
         YoGraphicPosition referenceICP = new YoGraphicPosition("controllerReferenceICP", controllerReferenceICP, 0.001, YoAppearance.Yellow(), GraphicType.SOLID_BALL);
         YoGraphicPosition referenceCMP = new YoGraphicPosition("controllerReferenceCMP", controllerReferenceCMP, 0.005, YoAppearance.Beige(), GraphicType.BALL_WITH_CROSS);
         YoGraphicPosition perfectCMP = new YoGraphicPosition("perfectCMP", controllerPerfectCMP, 0.005, YoAppearance.Beige(), GraphicType.BALL_WITH_ROTATED_CROSS);

         yoGraphicsListRegistry.registerArtifact(name, previousExitCMP.createArtifact());
         yoGraphicsListRegistry.registerArtifact(name, entryCMP.createArtifact());
         yoGraphicsListRegistry.registerArtifact(name, exitCMP.createArtifact());

         yoGraphicsListRegistry.registerArtifact(name, actualEndOfStateICP.createArtifact());
         yoGraphicsListRegistry.registerArtifact(name, beginningOfStateICP.createArtifact());
         yoGraphicsListRegistry.registerArtifact(name, nominalReferenceICP.createArtifact());
         yoGraphicsListRegistry.registerArtifact(name, nominalEndOfStateICP.createArtifact());

         yoGraphicsListRegistry.registerArtifact(name, referenceICP.createArtifact());
         yoGraphicsListRegistry.registerArtifact(name, referenceCMP.createArtifact());
         yoGraphicsListRegistry.registerArtifact(name, perfectCMP.createArtifact());
      }

      parentRegistry.addChild(registry);
   }

   public void setStepDurations(double doubleSupportDuration, double singleSupportDuration)
   {
      this.doubleSupportDuration.set(doubleSupportDuration);
      this.singleSupportDuration.set(singleSupportDuration);
      stepTimesInitialized.set(true);
   }

   public void clearPlan()
   {
      upcomingFootsteps.clear();
      footstepRecursionMultiplierCalculator.reset();
      referenceCMPsCalculator.clear();
      for (int i = 0; i < maximumNumberOfFootstepsToConsider; i++)
         upcomingFootstepLocations.get(i).setToZero();
   }

   private final FramePoint2d tmpFramePoint2d = new FramePoint2d();
   public void addFootstepToPlan(Footstep footstep)
   {
      if (footstep != null)
      {
         upcomingFootsteps.add(footstep);
         footstep.getPosition2d(tmpFramePoint2d);
         upcomingFootstepLocations.get(upcomingFootsteps.size() - 1).set(tmpFramePoint2d);
         referenceCMPsCalculator.addUpcomingFootstep(footstep);

         footstepSolutions.get(upcomingFootsteps.size() - 1).set(tmpFramePoint2d);
      }
   }

   public void initializeForStanding(double initialTime)
   {
      this.initialTime.set(initialTime);
      isStanding.set(true);
      isInTransfer.set(false);
      isInTransferEntry.set(false);
      isInitialTransfer.set(true);

      footstepRecursionMultiplierCalculator.resetTimes();
   }

   public void initializeForTransfer(double initialTime, RobotSide transferToSide)
   {
      this.initialTime.set(initialTime);
      this.transferToSide.set(transferToSide);
      if (transferToSide == null)
         transferToSide = RobotSide.LEFT;
      isInTransfer.set(true);
      isInTransferEntry.set(true);

      setProblemBooleans();

      // fixme submitting these must be smarter
      footstepRecursionMultiplierCalculator.resetTimes();
      if (isInitialTransfer.getBooleanValue())
         footstepRecursionMultiplierCalculator.submitTimes(0, initialDoubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());
      else
         footstepRecursionMultiplierCalculator.submitTimes(0, doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());

      for (int i = 1; i < numberOfFootstepsToConsider.getIntegerValue() + 1; i++)
         footstepRecursionMultiplierCalculator.submitTimes(i, doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider.getIntegerValue(), isInTransfer.getBooleanValue(),
            localUseTwoCMPs);

      referenceCMPsCalculator.setUseTwoCMPsPerSupport(localUseTwoCMPs);
      referenceCMPsCalculator.computeReferenceCMPsStartingFromDoubleSupport(isStanding.getBooleanValue(), transferToSide);
      referenceCMPsCalculator.update();

      if (localUseFootstepRegularization)
         resetFootstepRegularizationTask();
      if (localUseFeedbackRegularization)
         solver.resetFeedbackRegularization();

      initializeCMPConstraintDoubleSupport();
   }

   public void initializeForSingleSupport(double initialTime, RobotSide supportSide)
   {
      this.initialTime.set(initialTime);
      this.supportSide.set(supportSide);
      isStanding.set(false);
      isInTransfer.set(false);
      isInTransferEntry.set(false);
      isInitialTransfer.set(false);

      setProblemBooleans();

      // fixme submitting these must be smarter
      footstepRecursionMultiplierCalculator.resetTimes();
      footstepRecursionMultiplierCalculator.submitTimes(0, 0.0, singleSupportDuration.getDoubleValue());

      for (int i = 1; i < numberOfFootstepsToConsider.getIntegerValue() + 1; i++)
         footstepRecursionMultiplierCalculator.submitTimes(i, doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());
      footstepRecursionMultiplierCalculator.submitTimes(numberOfFootstepsToConsider.getIntegerValue() + 1, doubleSupportDuration.getDoubleValue(), singleSupportDuration.getDoubleValue());

      footstepRecursionMultiplierCalculator.computeRecursionMultipliers(numberOfFootstepsToConsider.getIntegerValue(), isInTransfer.getBooleanValue(), localUseTwoCMPs);

      referenceCMPsCalculator.setUseTwoCMPsPerSupport(localUseTwoCMPs);
      referenceCMPsCalculator.computeReferenceCMPsStartingFromSingleSupport(supportSide);
      referenceCMPsCalculator.update();

      if (localUseFootstepRegularization)
         resetFootstepRegularizationTask();
      if (localUseFeedbackRegularization)
         solver.resetFeedbackRegularization();

      initializeCMPConstraintSingleSupport(supportSide);
   }

   private void setProblemBooleans()
   {
      localUseTwoCMPs = useTwoCMPsInControl.getBooleanValue();
      localUseFeedback = useFeedback.getBooleanValue();
      localUseStepAdjustment = useStepAdjustment.getBooleanValue();
      localUseFootstepRegularization = useFootstepRegularization.getBooleanValue();
      localUseFeedbackRegularization = useFeedbackRegularization.getBooleanValue();
      localUseFeedbackWeightHardening = useFeedbackWeightHardening.getBooleanValue();

      localScaleUpcomingStepWeights = scaleUpcomingStepWeights.getBooleanValue();
   }

   private void initializeCMPConstraintDoubleSupport()
   {
      int numberOfVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
         numberOfVertices += bipedSupportPolygons.getFootPolygonInMidFeetZUp(robotSide).getNumberOfVertices();
      solver.setNumberOfVertices(numberOfVertices);

      numberOfVertices = 0;
      for (RobotSide robotSide : RobotSide.values)
      {
         FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getFootPolygonInMidFeetZUp(robotSide);

         for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
         {
            supportPolygon.getFrameVertex(i, tempVertex);
            solver.setSupportPolygonVertex(numberOfVertices + i, tempVertex, supportPolygon.getReferenceFrame(), maxCMPExitForward.getDoubleValue(),
                  maxCMPExitSideways.getDoubleValue());
         }

         numberOfVertices += supportPolygon.getNumberOfVertices();
      }
   }

   private void initializeCMPConstraintSingleSupport(RobotSide supportSide)
   {
      FrameConvexPolygon2d supportPolygon = bipedSupportPolygons.getFootPolygonInSoleFrame(supportSide);
      solver.setNumberOfVertices(supportPolygon.getNumberOfVertices());
      for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
      {
         supportPolygon.getFrameVertex(i, tempVertex);
         solver.setSupportPolygonVertex(i, tempVertex, supportPolygon.getReferenceFrame(), maxCMPExitForward.getDoubleValue(),
               maxCMPExitSideways.getDoubleValue());
      }
   }

   private final FramePoint2d perfectCMP = new FramePoint2d();
   private final FramePoint2d desiredCMP = new FramePoint2d();
   private final FrameVector2d desiredCMPDelta = new FrameVector2d();

   public void compute(double currentTime, FramePoint2d desiredICP, FrameVector2d desiredICPVelocity, FramePoint2d currentICP)
   {
      if (!stepTimesInitialized.getBooleanValue())
         throw new RuntimeException("Step timing has never been initialized!");

      desiredICP.changeFrame(worldFrame);
      desiredICPVelocity.changeFrame(worldFrame);
      currentICP.changeFrame(worldFrame);

      controllerCurrentICP.set(currentICP);
      controllerDesiredICP.set(desiredICP);
      controllerDesiredICPVelocity.set(desiredICPVelocity);

      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredICP, desiredICPVelocity, omega.getDoubleValue(), perfectCMP);
      controllerPerfectCMP.set(perfectCMP);

      computeTimeInCurrentState(currentTime);
      computeTimeRemainingInState();

      scaleStepRegularizationWeightWithTime();
      scaleFeedbackWeightWithGain();

      if (isStanding.getBooleanValue())
         doFeedbackOnlyControl();
      else
         doControlForStepping();

      solver.getCMPFeedback(desiredCMP);
      solver.getCMPFeedbackDifference(desiredCMPDelta);

      if (referenceFromNewCMP)
      {
         controllerReferenceCMP.getFrameTuple2d(desiredCMP);
         desiredCMP.add(desiredCMPDelta);
      }

      controllerFeedbackCMP.set(desiredCMP);
      controllerFeedbackCMPDelta.set(desiredCMPDelta);

      controllerCostToGo.set(solver.getCostToGo());
      controllerFootstepCostToGo.set(solver.getFootstepCostToGo());
      controllerFootstepRegularizationCostToGo.set(solver.getFootstepRegularizationCostToGo());
      controllerFeedbackCostToGo.set(solver.getFeedbackCostToGo());
      controllerFeedbackRegularizationCostToGo.set(solver.getFeedbackRegularizationCostToGo());
      controllerDynamicRelaxationCostToGo.set(solver.getDynamicRelaxationCostToGo());
   }

   private void doControlForStanding()
   {
      doFeedbackOnlyControl();
   }

   private final FramePoint2d locationSolution = new FramePoint2d();
   private final FramePoint2d tempVertex = new FramePoint2d();

   private void doControlForStepping()
   {
      int numberOfFootstepsToConsider = clipNumberOfFootstepsToConsiderToProblem(this.numberOfFootstepsToConsider.getIntegerValue());

      solver.submitProblemConditions(numberOfFootstepsToConsider, localUseStepAdjustment, localUseFeedback, localUseTwoCMPs); //, localUseActiveCMPOptimization);

      if (localUseFeedback)
      {
         solver.setFeedbackConditions(scaledFeedbackWeight.getDoubleValue(), feedbackGain.getDoubleValue(), dynamicRelaxationWeight.getDoubleValue(),
               omega.getDoubleValue());

         if (localUseFeedbackWeightHardening)
            solver.setUseFeedbackWeightHardening();

         if (localUseFeedbackRegularization)
            solver.setFeedbackRegularizationWeight(feedbackRegularizationWeight.getDoubleValue());
      }

      if (localUseStepAdjustment)
      {
         for (int footstepIndex = 0; footstepIndex < numberOfFootstepsToConsider; footstepIndex++)
            submitFootstepConditionsToSolver(footstepIndex);

         if (localUseFootstepRegularization)
            solver.setFootstepRegularizationWeight(scaledFootstepRegularizationWeight.getDoubleValue());
      }

      computeFinalICPRecursion();
      computeStanceCMPProjection();

      FramePoint2d offsetRecursionEffect = null;
      if (localUseTwoCMPs)
      {
         computeCMPOffsetRecursionEffect();
         offsetRecursionEffect = cmpOffsetRecursionEffect.getFrameTuple2d();
      }

      solver.compute(finalICPRecursion.getFrameTuple2d(), offsetRecursionEffect, controllerCurrentICP.getFrameTuple2d(),
            controllerPerfectCMP.getFrameTuple2d(), stanceCMPProjection.getFrameTuple2d(), footstepRecursionMultiplierCalculator.getCurrentStateProjectionMultiplier());

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
      {
         solver.getFootstepSolutionLocation(i, locationSolution);
         footstepSolutions.get(i).set(locationSolution);
      }

      computeReferenceFromSolutions(numberOfFootstepsToConsider);
      computeNominalValues(numberOfFootstepsToConsider);
   }

   private final FramePoint2d tmpEndPoint = new FramePoint2d();
   //private final FramePoint2d tmpBeginningPoint = new FramePoint2d();
   private final FramePoint2d tmpReferencePoint = new FramePoint2d();
   private final FrameVector2d tmpReferenceVelocity = new FrameVector2d();
   private final FramePoint2d tmpCMP = new FramePoint2d();

   private void computeReferenceFromSolutions(int numberOfFootstepsToConsider)
   {
      stanceEntryCMP2d.set(stanceEntryCMP.getFrameTuple2d());
      stanceExitCMP2d.set(stanceExitCMP.getFrameTuple2d());

      footstepRecursionMultiplierCalculator
            .computeICPPoints(finalICP.getFrameTuple2d(), footstepSolutions, entryOffsets, exitOffsets, previousStanceExitCMP.getFrameTuple2d(),
                  stanceEntryCMP2d, stanceExitCMP2d, numberOfFootstepsToConsider, tmpEndPoint, tmpReferencePoint, tmpReferenceVelocity);

      CapturePointTools.computeDesiredCentroidalMomentumPivot(tmpReferencePoint, tmpReferenceVelocity, omega.getDoubleValue(), tmpCMP);

      actualEndOfStateICP.set(tmpEndPoint);
      controllerReferenceICP.set(tmpReferencePoint);
      controllerReferenceICPVelocity.set(tmpReferenceVelocity);
      controllerReferenceCMP.set(tmpCMP);
   }

   private void computeNominalValues(int numberOfFootstepsToConsider)
   {
      footstepRecursionMultiplierCalculator.computeICPPoints(finalICP.getFrameTuple2d(), upcomingFootstepLocations, entryOffsets, exitOffsets,
            previousStanceExitCMP.getFrameTuple2d(), stanceEntryCMP.getFrameTuple2d(), stanceExitCMP.getFrameTuple2d(), numberOfFootstepsToConsider,
            tmpEndPoint, tmpReferencePoint, tmpReferenceVelocity);

      CapturePointTools.computeDesiredCentroidalMomentumPivot(tmpReferencePoint, tmpReferenceVelocity, omega.getDoubleValue(), tmpCMP);

      nominalEndOfStateICP.set(tmpEndPoint);
      //nominalBeginningOfStateICP.set(tmpBeginningPoint);
      nominalReferenceICP.set(tmpReferencePoint);
      nominalReferenceICPVelocity.set(tmpReferenceVelocity);
      nominalReferenceCMP.set(tmpCMP);
   }

   private final FramePoint2d blankFramePoint = new FramePoint2d(worldFrame);
   private void doFeedbackOnlyControl()
   {
      // fixme include vertices
      solver.submitProblemConditions(0, false, true, false); //, false);
      solver.setFeedbackConditions(scaledFeedbackWeight.getDoubleValue(), feedbackGain.getDoubleValue(), dynamicRelaxationWeight.getDoubleValue(), omega.getDoubleValue());

      solver.compute(controllerDesiredICP.getFrameTuple2d(), null, controllerCurrentICP.getFrameTuple2d(), controllerPerfectCMP.getFrameTuple2d(), blankFramePoint, 1.0);

      CapturePointTools.computeDesiredCentroidalMomentumPivot(controllerDesiredICP.getFrameTuple2d(), controllerDesiredICPVelocity.getFrameTuple2d(), omega.getDoubleValue(),
                                                              tmpCMP);
      controllerReferenceICP.set(controllerDesiredICP);
      controllerReferenceICPVelocity.set(controllerReferenceICPVelocity);
      controllerReferenceCMP.set(tmpCMP);
      nominalReferenceICP.set(controllerDesiredICP);
      nominalReferenceICPVelocity.set(controllerReferenceICPVelocity);
      nominalReferenceCMP.set(tmpCMP);
   }

   private void resetFootstepRegularizationTask()
   {
      int numberOfFootstepsToConsider = clipNumberOfFootstepsToConsiderToProblem(this.numberOfFootstepsToConsider.getIntegerValue());

      for (int i = 0; i < numberOfFootstepsToConsider; i++)
         solver.resetFootstepRegularization(i, upcomingFootstepLocations.get(i).getFrameTuple2d());
   }

   private int clipNumberOfFootstepsToConsiderToProblem(int numberOfFootstepsToConsider)
   {
      numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, upcomingFootsteps.size());
      numberOfFootstepsToConsider = Math.min(numberOfFootstepsToConsider, maximumNumberOfFootstepsToConsider);

      if (!localUseStepAdjustment)
         numberOfFootstepsToConsider = 0;

      return numberOfFootstepsToConsider;
   }

   private void submitFootstepConditionsToSolver(int footstepIndex)
   {
      double footstepWeight = this.footstepWeight.getDoubleValue();

      if (localScaleUpcomingStepWeights)
         footstepWeight = footstepWeight / (footstepIndex + 1);

      double footstepRecursionMultiplier;
      if (localUseTwoCMPs)
      {
         double entryMutliplier = footstepRecursionMultiplierCalculator.getCMPRecursionEntryMultiplier(footstepIndex);
         double exitMutliplier = footstepRecursionMultiplierCalculator.getCMPRecursionExitMultiplier(footstepIndex);

         footstepRecursionMultiplier = entryMutliplier + exitMutliplier;
      }
      else
      {
         footstepRecursionMultiplier = footstepRecursionMultiplierCalculator.getCMPRecursionExitMultiplier(footstepIndex);
      }

      solver.setFootstepAdjustmentConditions(footstepIndex, footstepRecursionMultiplier, footstepWeight,
            upcomingFootstepLocations.get(footstepIndex).getFrameTuple2d());
   }

   private final FramePoint2d finalEndingICP2d = new FramePoint2d(worldFrame);
   private void computeFinalICPRecursion()
   {
      int stepIndexToPoll;
      if (isInTransfer.getBooleanValue())
         stepIndexToPoll = numberOfFootstepsToConsider.getIntegerValue() + 2;
      else
         stepIndexToPoll = numberOfFootstepsToConsider.getIntegerValue() + 1;

      FramePoint finalEndingICP = referenceCMPsCalculator.getEntryCMPs().get(stepIndexToPoll).getFrameTuple();
      finalEndingICP.changeFrame(worldFrame);
      finalEndingICP2d.setByProjectionOntoXYPlane(finalEndingICP);

      finalICP.set(finalEndingICP2d);
      finalICPRecursion.set(finalEndingICP2d);
      finalICPRecursion.scale(footstepRecursionMultiplierCalculator.getFinalICPRecursionMultiplier());
   }

   private final FramePoint2d previousStanceExitCMP2d = new FramePoint2d(worldFrame);
   private final FramePoint2d stanceEntryCMP2d = new FramePoint2d(worldFrame);
   private final FramePoint2d stanceExitCMP2d = new FramePoint2d(worldFrame);
   private void computeStanceCMPProjection()
   {
      footstepRecursionMultiplierCalculator.computeRemainingProjectionMultipliers(timeRemainingInState.getDoubleValue(), localUseTwoCMPs,
            isInTransfer.getBooleanValue());

      if (localUseTwoCMPs)
      {
         if (isInTransfer.getBooleanValue())
         {
            FramePoint previousStanceExitCMP = referenceCMPsCalculator.getExitCMPs().get(0).getFrameTuple();
            FramePoint stanceEntryCMP = referenceCMPsCalculator.getEntryCMPs().get(1).getFrameTuple();
            FramePoint stanceExitCMP = referenceCMPsCalculator.getExitCMPs().get(1).getFrameTuple();

            previousStanceExitCMP2d.setByProjectionOntoXYPlane(previousStanceExitCMP);
            stanceEntryCMP2d.setByProjectionOntoXYPlane(stanceEntryCMP);
            stanceExitCMP2d.setByProjectionOntoXYPlane(stanceExitCMP);

            this.previousStanceExitCMP.set(previousStanceExitCMP2d);
            this.stanceEntryCMP.set(stanceEntryCMP2d);
            this.stanceExitCMP.set(stanceExitCMP2d);
         }
         else
         {
            FramePoint stanceEntryCMP = referenceCMPsCalculator.getEntryCMPs().get(0).getFrameTuple();
            FramePoint stanceExitCMP = referenceCMPsCalculator.getExitCMPs().get(0).getFrameTuple();

            previousStanceExitCMP2d.setToZero();
            stanceEntryCMP2d.setByProjectionOntoXYPlane(stanceEntryCMP);
            stanceExitCMP2d.setByProjectionOntoXYPlane(stanceExitCMP);

            this.previousStanceExitCMP.setToNaN();
            this.stanceEntryCMP.set(stanceEntryCMP2d);
            this.stanceExitCMP.set(stanceExitCMP2d);
         }
      }
      else
      {
         if (isInTransfer.getBooleanValue())
         {
            FramePoint previousStanceExitCMP = referenceCMPsCalculator.getEntryCMPs().get(0).getFrameTuple();
            FramePoint stanceExitCMP = referenceCMPsCalculator.getEntryCMPs().get(1).getFrameTuple();

            previousStanceExitCMP2d.setByProjectionOntoXYPlane(previousStanceExitCMP);
            stanceEntryCMP2d.setToZero();
            stanceExitCMP2d.setByProjectionOntoXYPlane(stanceExitCMP);

            this.previousStanceExitCMP.set(previousStanceExitCMP2d);
            this.stanceEntryCMP.setToNaN();
            this.stanceExitCMP.set(stanceExitCMP2d);
         }
         else
         {
            FramePoint stanceExitCMP = referenceCMPsCalculator.getEntryCMPs().get(0).getFrameTuple();

            previousStanceExitCMP2d.setToZero();
            stanceEntryCMP2d.setToZero();
            stanceExitCMP2d.setByProjectionOntoXYPlane(stanceExitCMP);

            this.previousStanceExitCMP.setToNaN();
            this.stanceEntryCMP.setToNaN();
            this.stanceExitCMP.set(stanceExitCMP2d);
         }
      }

      double previousExitMultiplier = footstepRecursionMultiplierCalculator.getRemainingPreviousStanceExitCMPProjectionMultiplier();
      double entryMultiplier = footstepRecursionMultiplierCalculator.getRemainingStanceEntryCMPProjectionMultiplier();
      double exitMultiplier = footstepRecursionMultiplierCalculator.getRemainingStanceExitCMPProjectionMultiplier();

      double currentStateProjectionMultiplier = footstepRecursionMultiplierCalculator.getCurrentStateProjectionMultiplier();

      previousExitMultiplier *= currentStateProjectionMultiplier;
      entryMultiplier *= currentStateProjectionMultiplier;
      exitMultiplier *= currentStateProjectionMultiplier;

      entryMultiplier += footstepRecursionMultiplierCalculator.getStanceEntryCMPProjectionMultiplier();
      exitMultiplier += footstepRecursionMultiplierCalculator.getStanceExitCMPProjectionMultiplier();

      previousStanceExitCMP2d.scale(previousExitMultiplier);
      stanceEntryCMP2d.scale(entryMultiplier);
      stanceExitCMP2d.scale(exitMultiplier);

      stanceCMPProjection.set(previousStanceExitCMP2d);
      stanceCMPProjection.add(stanceEntryCMP2d);
      stanceCMPProjection.add(stanceExitCMP2d);
   }

   private final FramePoint2d totalOffsetEffect = new FramePoint2d();
   private void computeCMPOffsetRecursionEffect()
   {
      computeTwoCMPOffsets();

      cmpOffsetRecursionEffect.setToZero();
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         totalOffsetEffect.set(yoExitOffsets.get(i).getFrameTuple2d());
         totalOffsetEffect.scale(footstepRecursionMultiplierCalculator.getCMPRecursionExitMultiplier(i));

         cmpOffsetRecursionEffect.add(totalOffsetEffect);

         totalOffsetEffect.set(yoEntryOffsets.get(i).getFrameTuple2d());
         totalOffsetEffect.scale(footstepRecursionMultiplierCalculator.getCMPRecursionEntryMultiplier(i));

         cmpOffsetRecursionEffect.add(totalOffsetEffect);
      }
   }

   private void computeTwoCMPOffsets()
   {
      for (int i = 0; i < numberOfFootstepsToConsider.getIntegerValue(); i++)
      {
         FrameVector2d entryOffset = entryOffsets.get(i);
         FrameVector2d exitOffset = exitOffsets.get(i);

         entryOffset.setToZero(worldFrame);
         exitOffset.setToZero(worldFrame);

         entryOffset.setByProjectionOntoXYPlane(referenceCMPsCalculator.getEntryCMPs().get(i + 1).getFrameTuple());
         exitOffset.setByProjectionOntoXYPlane(referenceCMPsCalculator.getExitCMPs().get(i + 1).getFrameTuple());

         entryOffset.sub(upcomingFootstepLocations.get(i).getFrameTuple2d());
         exitOffset.sub(upcomingFootstepLocations.get(i).getFrameTuple2d());

         yoEntryOffsets.get(i).set(entryOffset);
         yoExitOffsets.get(i).set(exitOffset);
      }
   }

   private void computeTimeInCurrentState(double currentTime)
   {
      timeInCurrentState.set(currentTime - initialTime.getDoubleValue());
   }

   private void computeTimeRemainingInState()
   {
      if (isStanding.getBooleanValue())
      {
         timeRemainingInState.set(0.0);
      }
      else
      {
         double remainingTime;
         if (isInTransfer.getBooleanValue())
         {
            if (timeInCurrentState.getDoubleValue() < doubleSupportSplitFraction.getDoubleValue() * doubleSupportDuration.getDoubleValue())
               isInTransferEntry.set(true);
            else
               isInTransferEntry.set(false);

            remainingTime = doubleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue();
         }
         else
         {
            remainingTime = singleSupportDuration.getDoubleValue() - timeInCurrentState.getDoubleValue();
         }

         remainingTime = Math.max(icpOptimizationParameters.getMinimumTimeRemaining(), remainingTime);
         timeRemainingInState.set(remainingTime);
      }
   }

   private void scaleStepRegularizationWeightWithTime()
   {
      if (scaleStepRegularizationWeightWithTime.getBooleanValue())
      {
         double alpha = timeRemainingInState.getDoubleValue() / singleSupportDuration.getDoubleValue();
         scaledFootstepRegularizationWeight.set(footstepRegularizationWeight.getDoubleValue() / alpha);
      }
      else
      {
         scaledFootstepRegularizationWeight.set(footstepRegularizationWeight.getDoubleValue());
      }
   }

   private void scaleFeedbackWeightWithGain()
   {
      if (scaleFeedbackWeightWithGain.getBooleanValue())
      {
         double alpha = Math.pow(feedbackGain.getDoubleValue(), 2);
         scaledFeedbackWeight.set(feedbackWeight.getDoubleValue() / alpha);
      }
      else
      {
         scaledFeedbackWeight.set(feedbackWeight.getDoubleValue());
      }
   }

   public int getNumberOfFootstepsToConsider()
   {
      return numberOfFootstepsToConsider.getIntegerValue();
   }

   public void getDesiredCMP(FramePoint2d desiredCMPToPack)
   {
      controllerFeedbackCMP.getFrameTuple2d(desiredCMPToPack);
   }

   public void getFootstepSolution(int footstepIndex, FramePoint2d footstepSolutionToPack)
   {
      footstepSolutions.get(footstepIndex).getFrameTuple2d(footstepSolutionToPack);
   }
}
