package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.captureRegion.CaptureRegionSafetyHeuristics;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.*;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class ErrorBasedStepAdjustmentController implements StepAdjustmentController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private static final boolean VISUALIZE = true;
   private static final boolean CONTINUOUSLY_UPDATE_DESIRED_POSITION = true;
   private static final int minTicksIntoStep = 5;

   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final BooleanProvider allowStepAdjustment;
   private final DoubleProvider footstepDeadband;

   private final BooleanProvider allowCrossOverSteps;

   private final YoBoolean useStepAdjustment = new YoBoolean(yoNamePrefix + "UseStepAdjustment", registry);
   private final YoBoolean footstepIsAdjustable = new YoBoolean(yoNamePrefix + "FootstepIsAdjustable", registry);
   private final YoBoolean shouldCheckForReachability = new YoBoolean(yoNamePrefix + "ShouldCheckForReachability", registry);
   private final YoBoolean hasPlanarRegionBeenAssigned = new YoBoolean(yoNamePrefix + "HasPlanarRegionBeenAssigned", registry);

   private final YoDouble percentageToShrinkPolygon = new YoDouble(yoNamePrefix + "PercentageToShrinkPolygon", registry);
   private final YoDouble swingDuration = new YoDouble(yoNamePrefix + "SwingDuration", registry);

   private final YoInteger controlTicksIntoStep = new YoInteger(yoNamePrefix + "TicksIntoStep", registry);
   private final YoInteger stepsInQueue = new YoInteger(yoNamePrefix + "StepsInQueue", registry);
   private final YoDouble subsequentStepDuration = new YoDouble(yoNamePrefix + "SubsequentStepDuration", registry);

   private final YoFramePose3D upcomingFootstep = new YoFramePose3D(yoNamePrefix + "UpcomingFootstepPose", worldFrame, registry);
   private final YoEnum<RobotSide> upcomingFootstepSide = new YoEnum<>(yoNamePrefix + "UpcomingFootstepSide", registry, RobotSide.class);
   private final RecyclingArrayList<Point2D> upcomingFootstepContactPoints = new RecyclingArrayList<>(Point2D.class);
   private final YoFramePoint3D referenceFootstepPosition = new YoFramePoint3D(yoNamePrefix + "ReferenceFootstepPosition", worldFrame, registry);

   private final FramePoint3D tempPoint = new FramePoint3D();
   private final FramePoint2D tempPoint2D = new FramePoint2D();

   private final YoFrameVector2D footstepAdjustment = new YoFrameVector2D(yoNamePrefix + "FootstepAdjustment",
                                                                          worldFrame,
                                                                          registry);
   private final YoFrameVector2D deadbandedAdjustment = new YoFrameVector2D(yoNamePrefix + "DeadbandedAdjustment", worldFrame, registry);
   private final YoFrameVector2D totalStepAdjustment = new YoFrameVector2D(yoNamePrefix + "TotalStepAdjustment", worldFrame, registry);

   private final YoFramePoint3D previousFootstepSolution = new YoFramePoint3D(yoNamePrefix + "PreviousFootstepSolutionLocation", worldFrame, registry);
   private final YoFramePose3D footstepSolution = new YoFramePose3D(yoNamePrefix + "FootstepSolutionLocation", worldFrame, registry);
   private final YoFramePoint2D adjustedSolution = new YoFramePoint2D(yoNamePrefix + "AdjustedSolution", worldFrame, registry);

   private final YoBoolean isInSwing = new YoBoolean(yoNamePrefix + "IsInSwing", registry);
   private final YoDouble initialTime = new YoDouble(yoNamePrefix + "InitialTime", registry);
   private final YoDouble timeInCurrentState = new YoDouble(yoNamePrefix + "TimeInCurrentState", registry);
   private final YoDouble timeRemainingInState = new YoDouble(yoNamePrefix + "TimeRemainingInState", registry);

   private final YoBoolean swingSpeedUpEnabled = new YoBoolean(yoNamePrefix + "SwingSpeedUpEnabled", registry);
   private final YoDouble speedUpTime = new YoDouble(yoNamePrefix + "SpeedUpTime", registry);

   private final YoBoolean footstepWasAdjusted = new YoBoolean(yoNamePrefix + "FootstepWasAdjusted", registry);

   private final BooleanProvider resetFootstepProjectionEachTick;
   private final DoubleProvider minimumTimeForStepAdjustment;
   private final DoubleParameter supportDistanceFromFront;
   private final DoubleParameter supportDistanceFromBack;
   private final DoubleParameter supportDistanceFromInside;
   private final DoubleParameter supportDistanceFromOutside;

   private final SideDependentList<FixedFrameConvexPolygon2DBasics> allowableAreasForCoP = new SideDependentList<>();
   private final YoFrameConvexPolygon2D allowableAreaForCoP = new YoFrameConvexPolygon2D(yoNamePrefix + "AllowableAreaForCoP", worldFrame, 4, registry);

   private final StepAdjustmentReachabilityConstraint reachabilityConstraintHandler;
   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final CaptureRegionSafetyHeuristics oneStepSafetyHeuristics;
   private final MultiStepCaptureRegionCalculator multiStepCaptureRegionCalculator;
   private final EnvironmentConstraintHandler environmentConstraintProvider;

   private final ConvexPolygonTools polygonTools = new ConvexPolygonTools();
   private final FrameConvexPolygon2D captureRegionInWorld = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D reachableCaptureRegion = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D forwardCrossOverReachableCaptureRegion = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D backwardCrossOverReachableCaptureRegion = new FrameConvexPolygon2D();

   private final BipedSupportPolygons bipedSupportPolygons;

   private final FramePoint3D vertexInWorld = new FramePoint3D();
   private final FrameConvexPolygon2D allowableAreaForCoPInFoot = new FrameConvexPolygon2D();

   public ErrorBasedStepAdjustmentController(WalkingControllerParameters walkingControllerParameters,
                                             SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                             BipedSupportPolygons bipedSupportPolygons,
                                             ICPControlPolygons icpControlPolygons,
                                             SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                             YoRegistry parentRegistry,
                                             YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters,
           walkingControllerParameters.getStepAdjustmentParameters(),
           soleZUpFrames,
           bipedSupportPolygons,
           icpControlPolygons,
           contactableFeet,
           parentRegistry,
           yoGraphicsListRegistry);
   }

   public ErrorBasedStepAdjustmentController(WalkingControllerParameters walkingControllerParameters,
                                             StepAdjustmentParameters stepAdjustmentParameters,
                                             SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                             BipedSupportPolygons bipedSupportPolygons,
                                             ICPControlPolygons icpControlPolygons,
                                             SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                             YoRegistry parentRegistry,
                                             YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;

      allowStepAdjustment = new BooleanParameter(yoNamePrefix + "AllowStepAdjustment", registry, stepAdjustmentParameters.allowStepAdjustment());

      resetFootstepProjectionEachTick = new BooleanParameter(yoNamePrefix + "ResetFootstepProjectionEachTick", registry, false);
      minimumTimeForStepAdjustment = new DoubleParameter(yoNamePrefix + "minimumTimeForStepAdjustment",
                                                         registry,
                                                         stepAdjustmentParameters.getMinimumTimeForStepAdjustment());

      supportDistanceFromFront = new DoubleParameter(yoNamePrefix + "supportDistanceFromFront",
                                                     registry,
                                                     stepAdjustmentParameters.getCoPDistanceFromFrontOfFoot());
      supportDistanceFromBack = new DoubleParameter(yoNamePrefix + "supportDistanceFromBack",

                                                    registry,
                                                    stepAdjustmentParameters.getCoPDistanceFromBackOfFoot());
      supportDistanceFromInside = new DoubleParameter(yoNamePrefix + "supportDistanceFromInside",
                                                      registry,
                                                      stepAdjustmentParameters.getCoPDistanceFromInsideOfFoot());
      supportDistanceFromOutside = new DoubleParameter(yoNamePrefix + "supportDistanceFromOutside",
                                                       registry,
                                                       stepAdjustmentParameters.getCoPDistanceFromOutsideOfFoot());

      footstepDeadband = new DoubleParameter(yoNamePrefix + "FootstepDeadband", registry, stepAdjustmentParameters.getAdjustmentDeadband());
      allowCrossOverSteps = new BooleanParameter(yoNamePrefix + "AllowCrossOverSteps", registry, stepAdjustmentParameters.allowCrossOverSteps());

      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      DoubleProvider lengthLimit = new DoubleParameter(yoNamePrefix + "MaxReachabilityLength", registry, steppingParameters.getMaxStepLength());
      DoubleProvider lengthBackLimit = new DoubleParameter(yoNamePrefix + "MaxReachabilityBackwardLength",
                                                           registry,
                                                           steppingParameters.getMaxBackwardStepLength());
      DoubleProvider innerLimit = new DoubleParameter(yoNamePrefix + "MinReachabilityWidth", registry, steppingParameters.getMinStepWidth());
      DoubleProvider outerLimit = new DoubleParameter(yoNamePrefix + "MaxReachabilityWidth", registry, steppingParameters.getMaxStepWidth());
      DoubleProvider inPlaceWidth = new DoubleParameter(yoNamePrefix + "InPlaceWidth", registry, steppingParameters.getInPlaceWidth());

      reachabilityConstraintHandler = new StepAdjustmentReachabilityConstraint(soleZUpFrames,
                                                                               lengthLimit,
                                                                               lengthBackLimit,
                                                                               innerLimit,
                                                                               outerLimit,
                                                                               inPlaceWidth,
                                                                               stepAdjustmentParameters.getCrossOverReachabilityParameters(),
                                                                               yoNamePrefix,
                                                                               VISUALIZE,
                                                                               registry,
                                                                               yoGraphicsListRegistry);

      // the 1.5 multiplier is important so that the capture region is bigger than reachable
      captureRegionCalculator = new OneStepCaptureRegionCalculator(steppingParameters.getFootWidth(),
                                                                   () -> 1.5 * lengthLimit.getValue(),
                                                                   soleZUpFrames,
                                                                   false,
                                                                   yoNamePrefix,
                                                                   registry,
                                                                   yoGraphicsListRegistry);
      oneStepSafetyHeuristics = new CaptureRegionSafetyHeuristics(lengthLimit, registry, null);
      multiStepCaptureRegionCalculator = new MultiStepCaptureRegionCalculator(reachabilityConstraintHandler,
                                                                              allowCrossOverSteps,
                                                                              registry,
                                                                              yoGraphicsListRegistry);
      environmentConstraintProvider = new EnvironmentConstraintHandler(contactableFeet,
                                                                       yoNamePrefix,
                                                                       registry,
                                                                       yoGraphicsListRegistry);

      for (RobotSide robotSide : RobotSide.values)
      {
         FixedFrameConvexPolygon2DBasics allowableAreaForCoP = new YoFrameConvexPolygon2D(robotSide.getCamelCaseName() + "AllowableAreaForCoP",
                                                                                          soleZUpFrames.get(robotSide),
                                                                                          4,
                                                                                          registry);
         allowableAreaForCoP.set(bipedSupportPolygons.getFootPolygonInSoleFrame(robotSide));
         allowableAreasForCoP.put(robotSide, allowableAreaForCoP);
      }

      if (walkingControllerParameters != null)
         swingSpeedUpEnabled.set(walkingControllerParameters.allowDisturbanceRecoveryBySpeedingUpSwing());

      if (yoGraphicsListRegistry != null)
         setupVisualizers(yoGraphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   private void setupVisualizers(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      ArtifactList artifactList = new ArtifactList(getClass().getSimpleName());

      YoGraphicPosition clippedFootstepSolution = new YoGraphicPosition(yoNamePrefix + "FootstepSolution",
                                                                        this.footstepSolution.getPosition(),
                                                                        0.005,
                                                                        YoAppearance.DarkRed(),
                                                                        YoGraphicPosition.GraphicType.BALL);

      artifactList.add(clippedFootstepSolution.createArtifact());

      artifactList.setVisible(VISUALIZE);

      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   @Override
   public void reset()
   {
      reachabilityConstraintHandler.reset();
      isInSwing.set(false);
      upcomingFootstep.setToNaN();
      footstepSolution.setToNaN();
      previousFootstepSolution.setToNaN();
      footstepWasAdjusted.set(false);
      hasPlanarRegionBeenAssigned.set(false);
      captureRegionCalculator.hideCaptureRegion();
      oneStepSafetyHeuristics.reset();
      multiStepCaptureRegionCalculator.reset();
      environmentConstraintProvider.reset();
      controlTicksIntoStep.set(0);
      this.stepsInQueue.set(0);
      this.subsequentStepDuration.set(Double.NaN);
   }

   @Override
   public void setFootstepQueueInformation(int numberOfStepsInQueue, double subsequentStepDuration)
   {
      stepsInQueue.set(numberOfStepsInQueue);
      this.subsequentStepDuration.set(subsequentStepDuration);
   }


   @Override
   public void setFootstepToAdjust(SimpleFootstep footstep, double swingDuration)
   {
      FramePose3DReadOnly footstepPose = footstep.getSoleFramePose();
      footstepPose.checkReferenceFrameMatch(worldFrame);
      if (!footstepPose.containsNaN())
      {
         upcomingFootstep.set(footstepPose);
         upcomingFootstepSide.set(footstep.getRobotSide());
         upcomingFootstepContactPoints.clear();
         ConvexPolygon2DReadOnly foothold = footstep.getFoothold();
         for (int i = 0; i < foothold.getNumberOfVertices(); i++)
         {
            upcomingFootstepContactPoints.add().set(foothold.getVertex(i));
         }

         referenceFootstepPosition.set(footstepPose.getPosition());
         footstepSolution.set(footstepPose);
         previousFootstepSolution.set(footstepSolution.getPosition());

         this.swingDuration.set(swingDuration);

         footstepIsAdjustable.set(footstep.getIsAdjustable());
         shouldCheckForReachability.set(footstep.getShouldCheckReachability());
         useStepAdjustment.set(allowStepAdjustment.getValue() && footstepIsAdjustable.getBooleanValue());
      }
      else
      {
         LogTools.warn("Received bad footstep: " + footstep);
      }
   }

   @Override
   public void submitSwingSpeedUpUnderDisturbance(double swingSpeedUp)
   {
      if (swingSpeedUpEnabled.getBooleanValue() && swingSpeedUp > speedUpTime.getValue())
      {
         this.speedUpTime.set(swingSpeedUp);
      }
   }

   @Override
   public void setStepConstraintRegions(List<StepConstraintRegion> stepConstraintRegion)
   {
      environmentConstraintProvider.setStepConstraintRegions(stepConstraintRegion);
   }

   @Override
   public List<StepConstraintRegion> getStepConstraintRegions()
   {
      return environmentConstraintProvider.getStepConstraintRegions();
   }

   @Override
   public void initialize(double initialTime, RobotSide supportSide)
   {
      isInSwing.set(true);
      this.initialTime.set(initialTime);
      reachabilityConstraintHandler.initializeReachabilityConstraint(supportSide);
      speedUpTime.set(0.0);
      footstepSolution.set(upcomingFootstep);
      previousFootstepSolution.set(footstepSolution.getPosition());
      totalStepAdjustment.setToZero();
      controlTicksIntoStep.set(0);
   }

   @Override
   public void compute(double currentTime,
                       FramePoint2DReadOnly desiredICP,
                       FramePoint2DReadOnly currentICP,
                       double omega0,
                       FramePoint2DReadOnly copToShrinkAbout,
                       double percentageToShrinkPolygon)
   {
      this.percentageToShrinkPolygon.set(percentageToShrinkPolygon);
      footstepWasAdjusted.set(false);

      if (!isInSwing.getBooleanValue())
         return;

      controlTicksIntoStep.increment();

      computeTimeInCurrentState(currentTime);
      computeTimeRemainingInState();

      previousFootstepSolution.set(footstepSolution.getPosition());

      if (controlTicksIntoStep.getIntegerValue() <= minTicksIntoStep)
         upcomingFootstep.getPosition().set(referenceFootstepPosition);

      if (timeRemainingInState.getValue() < minimumTimeForStepAdjustment.getValue())
         return;

      computeLimitedAreaForCoP(copToShrinkAbout, percentageToShrinkPolygon);
      RobotSide swingSide = upcomingFootstepSide.getEnumValue();
      RobotSide stanceSide = swingSide.getOppositeSide();
      double timeToPrject = Math.max(timeRemainingInState.getDoubleValue(), 0.0);
      captureRegionCalculator.calculateCaptureRegion(swingSide, timeToPrject, currentICP, omega0, allowableAreaForCoP);
      oneStepSafetyHeuristics.computeCaptureRegionWithSafetyHeuristics(stanceSide,
                                                                       currentICP,
                                                                       allowableAreaForCoP.getCentroid(),
                                                                       captureRegionCalculator.getCaptureRegion());
      FrameConvexPolygon2DReadOnly singleStepRegion = oneStepSafetyHeuristics.getCaptureRegionWithSafetyMargin();
      // Steps in queue accounts for the current step, so for it to hold value, it has to be at least 2.
      multiStepCaptureRegionCalculator.compute(stanceSide,
                                               singleStepRegion,
                                               stepsInQueue.getIntegerValue() < 2 ? Double.NaN : subsequentStepDuration.getDoubleValue(),
                                               omega0,
                                               stepsInQueue.getIntegerValue());

      FramePoint3DReadOnly pointToProject = resetFootstepProjectionEachTick.getValue() ? referenceFootstepPosition : upcomingFootstep.getPosition();

      if (!useStepAdjustment.getBooleanValue())
      {
         if (shouldCheckForReachability.getValue())
         {
            boolean wasAdjusted = projectAdjustedStepIntoReachability(pointToProject);
            footstepWasAdjusted.set(wasAdjusted);

            if (wasAdjusted)
            {
               footstepSolution.getPosition().set(adjustedSolution, upcomingFootstep.getPosition().getZ());

               if (CONTINUOUSLY_UPDATE_DESIRED_POSITION)
                  upcomingFootstep.set(footstepSolution);
            }
         }
         return;
      }

      // actually apply the adjustment
      projectAdjustedStepIntoCaptureRegion(pointToProject);
      boolean wasAdjusted = deadbandAndApplyStepAdjustment();

      environmentConstraintProvider.setReachabilityRegion(reachabilityConstraintHandler.getReachabilityConstraint());
      environmentConstraintProvider.updateActiveConstraintRegionToUse(footstepSolution, multiStepCaptureRegionCalculator.getCaptureRegion());

      if (environmentConstraintProvider.hasStepConstraintRegion() && (wasAdjusted || !hasPlanarRegionBeenAssigned.getBooleanValue()))
      {
         if (environmentConstraintProvider.validateConvexityOfPlanarRegion())
         {
            boolean environmentallyConstrained = environmentConstraintProvider.applyEnvironmentConstraintToFootstep(upcomingFootstepSide.getEnumValue(),
                                                                                                                    footstepSolution,
                                                                                                                    upcomingFootstepContactPoints);
            wasAdjusted |= environmentallyConstrained;
            hasPlanarRegionBeenAssigned.set(environmentConstraintProvider.foundSolution());

            // ok, force it back to be reachable
            if (environmentallyConstrained)
            {
               tempPoint2D.set(footstepSolution.getPosition());
               FrameConvexPolygon2DReadOnly reachability = getBestReachabilityConstraintToUseWhenNotIntersecting();
               if (!reachability.isPointInside(tempPoint2D))
                  reachability.orthogonalProjection(tempPoint2D);
               footstepSolution.getPosition().set(tempPoint2D);
            }
         }
      }

      footstepWasAdjusted.set(wasAdjusted || previousFootstepSolution.distance(footstepSolution.getPosition()) > 1e-3);

      // Don't update the "upcoming footstep" if we're at the start of the control state. At this point, we haven't had our swing duration adjusted, so we want
      // to converge down to an intelligent time
      if (wasFootstepAdjusted() && CONTINUOUSLY_UPDATE_DESIRED_POSITION && controlTicksIntoStep.getIntegerValue() > minTicksIntoStep)
         upcomingFootstep.set(footstepSolution);
   }

   private final FramePoint2D finalInFoot = new FramePoint2D();

   private void computeLimitedAreaForCoP(FramePoint2DReadOnly copToShrinkAbout, double percentageToShrinkPolygon)
   {
      RobotSide supportSide = upcomingFootstepSide.getEnumValue().getOppositeSide();
      FixedFrameConvexPolygon2DBasics shrunkSupport = allowableAreasForCoP.get(supportSide);
      FrameConvexPolygon2DReadOnly supportPolygon = bipedSupportPolygons.getFootPolygonInSoleZUpFrame(supportSide);
      finalInFoot.setMatchingFrame(copToShrinkAbout);
      finalInFoot.changeFrame(shrunkSupport.getReferenceFrame());

      if (!Double.isNaN(percentageToShrinkPolygon) && percentageToShrinkPolygon <= 1e-5)
      {
         shrunkSupport.clear();
         shrunkSupport.addVertex(finalInFoot);
         shrunkSupport.update();
         allowableAreaForCoPInFoot.setIncludingFrame(shrunkSupport);
      }
      else
      {
         shrunkSupport.set(supportPolygon);
         if (!Double.isNaN(percentageToShrinkPolygon))
            shrunkSupport.scale(finalInFoot, percentageToShrinkPolygon);

         for (int i = 0; i < shrunkSupport.getNumberOfVertices(); i++)
         {
            FixedFramePoint2DBasics shrunkPoint = shrunkSupport.getVertexUnsafe(i);
            FramePoint2DReadOnly supportPoint = supportPolygon.getVertexUnsafe(i);
            if (supportPoint.getX() > 0.0)
            {
               // This point is towards the toe
               double maxToeX = Math.max(supportPoint.getX() - supportDistanceFromFront.getValue(), finalInFoot.getX());
               if (maxToeX < shrunkPoint.getX())
                  shrunkPoint.setX(maxToeX);
            }
            else
            {
               // This point is towards the heel
               double minHeelX = Math.min(supportPoint.getX() + supportDistanceFromBack.getValue(), finalInFoot.getX());
               if (minHeelX > shrunkPoint.getX())
                  shrunkPoint.setX(minHeelX);
            }

            if (supportSide == RobotSide.LEFT)
            {
               if (supportPoint.getY() > 0)
               {
                  // This point is towards the outside
                  double maxOutsideY = Math.max(supportPoint.getY() - supportDistanceFromOutside.getValue(), finalInFoot.getY());
                  if (maxOutsideY < shrunkPoint.getY())
                     shrunkPoint.setY(maxOutsideY);
               }
               else
               {
                  // This point is towards the inside
                  double minInsideY = Math.min(supportPoint.getY() + supportDistanceFromInside.getValue(), finalInFoot.getY());
                  if (minInsideY > shrunkPoint.getY())
                     shrunkPoint.setY(minInsideY);
               }
            }
            else
            {
               if (supportPoint.getY() > 0)
               {
                  // This point is towards the inside
                  double maxInsideY = Math.max(supportPoint.getY() - supportDistanceFromInside.getValue(), finalInFoot.getY());
                  if (maxInsideY < shrunkPoint.getY())
                     shrunkPoint.setY(maxInsideY);
               }
               else
               {
                  // This point is towards the outside
                  double minOutsideY = Math.min(supportPoint.getY() + supportDistanceFromOutside.getValue(), finalInFoot.getY());
                  if (minOutsideY > shrunkPoint.getY())
                     shrunkPoint.setY(minOutsideY);
               }
            }
         }

         allowableAreaForCoPInFoot.setIncludingFrame(shrunkSupport);
      }

      allowableAreaForCoP.setMatchingFrame(allowableAreaForCoPInFoot, false);
   }

   private void projectAdjustedStepIntoCaptureRegion(FramePoint3DReadOnly pointToProject)
   {
      adjustedSolution.set(pointToProject);

      captureRegionInWorld.setIncludingFrame(multiStepCaptureRegionCalculator.getCaptureRegion());
      captureRegionInWorld.changeFrameAndProjectToXYPlane(worldFrame);

      if (!isTheCaptureRegionReachable())
      {
         captureRegionInWorld.orthogonalProjection(adjustedSolution);
         getBestReachabilityConstraintToUseWhenNotIntersecting().orthogonalProjection(adjustedSolution);
      }
      else
      {
         getBestReachabilityConstraintToUseWhenIntersecting().orthogonalProjection(adjustedSolution);
      }

      footstepAdjustment.set(adjustedSolution);
      footstepAdjustment.sub(previousFootstepSolution.getX(), previousFootstepSolution.getY());
   }

   private boolean projectAdjustedStepIntoReachability(FramePoint3DReadOnly pointToProject)
   {
      adjustedSolution.set(pointToProject);

      FrameConvexPolygon2DReadOnly reachabilityPolygon = reachabilityConstraintHandler.getTotalReachabilityHull(upcomingFootstepSide.getEnumValue()
                                                                                                                                    .getOppositeSide());
      if (!reachabilityPolygon.isPointInside(adjustedSolution))
      {
         reachabilityConstraintHandler.getReachabilityConstraint().orthogonalProjection(adjustedSolution);
         return true;
      }

      return false;
   }

   private boolean isTheCaptureRegionReachable()
   {
      boolean intersect = polygonTools.computeIntersectionOfPolygons(captureRegionInWorld,
                                                                     reachabilityConstraintHandler.getReachabilityConstraint(),
                                                                     reachableCaptureRegion);
      if (allowCrossOverSteps.getValue())
      {
         intersect |= polygonTools.computeIntersectionOfPolygons(captureRegionInWorld,
                                                                 reachabilityConstraintHandler.getForwardCrossOverPolygon(),
                                                                 forwardCrossOverReachableCaptureRegion);
         intersect |= polygonTools.computeIntersectionOfPolygons(captureRegionInWorld,
                                                                 reachabilityConstraintHandler.getBackwardCrossOverPolygon(),
                                                                 backwardCrossOverReachableCaptureRegion);
      }

      return intersect;
   }

   private FrameConvexPolygon2DReadOnly getBestReachabilityConstraintToUseWhenNotIntersecting()
   {
      if (!allowCrossOverSteps.getValue())
         return reachabilityConstraintHandler.getReachabilityConstraint();

      double distanceToForward = reachabilityConstraintHandler.getForwardCrossOverPolygon().distance(adjustedSolution);
      double distanceToBackward = reachabilityConstraintHandler.getBackwardCrossOverPolygon().distance(adjustedSolution);
      double distanceToNominal = reachabilityConstraintHandler.getReachabilityConstraint().distance(adjustedSolution);

      boolean forwardIsCloser = distanceToForward < distanceToBackward;

      if (forwardIsCloser)
      {
         if (distanceToNominal < distanceToForward)
            return reachabilityConstraintHandler.getReachabilityConstraint();
         else
            return reachabilityConstraintHandler.getForwardCrossOverPolygon();
      }
      else if (distanceToNominal < distanceToBackward)
      {
         return reachabilityConstraintHandler.getReachabilityConstraint();
      }
      else
      {
         return reachabilityConstraintHandler.getBackwardCrossOverPolygon();
      }
   }

   private FrameConvexPolygon2DReadOnly getBestReachabilityConstraintToUseWhenIntersecting()
   {
      if (!allowCrossOverSteps.getValue())
         return reachableCaptureRegion;

      double forwardArea = forwardCrossOverReachableCaptureRegion.getArea();
      double backwardArea = backwardCrossOverReachableCaptureRegion.getArea();
      double reachableArea = reachableCaptureRegion.getArea();

      forwardArea = Double.isNaN(forwardArea) ? Double.NEGATIVE_INFINITY : forwardArea;
      backwardArea = Double.isNaN(backwardArea) ? Double.NEGATIVE_INFINITY : backwardArea;
      reachableArea = Double.isNaN(reachableArea) ? Double.NEGATIVE_INFINITY : reachableArea;

      boolean forwardIsLargestCrossoverArea = forwardArea > backwardArea;

      if (forwardIsLargestCrossoverArea)
      {
         if (forwardArea > 2.0 * reachableArea)
            return forwardCrossOverReachableCaptureRegion;
         else
            return reachableCaptureRegion;
      }
      else if (backwardArea > 2.0 * reachableArea)
      {
         return backwardCrossOverReachableCaptureRegion;
      }
      else
      {
         return reachableCaptureRegion;
      }
   }

   private boolean deadbandAndApplyStepAdjustment()
   {
      boolean adjusted;
      if (footstepAdjustment.containsNaN() || footstepAdjustment.norm() < footstepDeadband.getValue())
      {
         adjusted = false;
         deadbandedAdjustment.setToZero();
      }
      else
      {
         adjusted = true;
         deadbandedAdjustment.set(footstepAdjustment);
      }

      adjustedSolution.set(previousFootstepSolution);
      adjustedSolution.add(deadbandedAdjustment);

      totalStepAdjustment.add(deadbandedAdjustment);
      footstepSolution.getPosition().set(adjustedSolution, upcomingFootstep.getPosition().getZ());

      return adjusted;
   }

   @Override
   public FramePose3DReadOnly getFootstepSolution()
   {
      return footstepSolution;
   }

   @Override
   public boolean wasFootstepAdjusted()
   {
      return footstepWasAdjusted.getBooleanValue();
   }

   @Override
   public boolean useStepAdjustment()
   {
      return useStepAdjustment.getBooleanValue();
   }

   private void computeTimeInCurrentState(double currentTime)
   {
      timeInCurrentState.set(currentTime - initialTime.getDoubleValue() + speedUpTime.getDoubleValue());
   }

   private void computeTimeRemainingInState()
   {
      timeRemainingInState.set(swingDuration.getDoubleValue() - timeInCurrentState.getDoubleValue());
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(reachabilityConstraintHandler.getSCS2YoGraphics());
      group.addChild(environmentConstraintProvider.getSCS2YoGraphics());
//      group.addChild(captureRegionCalculator.getSCS2YoGraphics());
//      group.addChild(oneStepSafetyHeuristics.getSCS2YoGraphics());
      group.addChild(multiStepCaptureRegionCalculator.getSCS2YoGraphics());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D(yoNamePrefix + "FootstepSolution",
                                                                    footstepSolution.getPosition(),
                                                                    0.01,
                                                                    ColorDefinitions.DarkRed(),
                                                                    DefaultPoint2DGraphic.CIRCLE));
//      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("Allowable Area for CoP", allowableAreaForCoP, ColorDefinitions.Red()));

      group.setVisible(VISUALIZE);
      return group;
   }
}
