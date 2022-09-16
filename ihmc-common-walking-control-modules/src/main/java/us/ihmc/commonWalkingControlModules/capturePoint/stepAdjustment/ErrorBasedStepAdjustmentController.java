package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.captureRegion.CaptureRegionSafetyHeuristics;
import us.ihmc.commonWalkingControlModules.captureRegion.MultiStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.List;

public class ErrorBasedStepAdjustmentController implements StepAdjustmentController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private static final boolean VISUALIZE = true;
   private static final boolean CONTINUOUSLY_UPDATE_DESIRED_POSITION = true;

   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final BooleanProvider allowStepAdjustment;
   private final DoubleProvider footstepDeadband;
   private final DoubleProvider transferDurationSplitFraction;

   private final DoubleProvider minimumFootstepMultiplier;
   private final DoubleProvider minICPErrorForStepAdjustment;
   private final DoubleProvider maximumTimeFromTransfer;
   private final BooleanProvider useActualErrorInsteadOfResidual;
   private final BooleanProvider considerErrorInAdjustment;
   private final BooleanProvider allowCrossOverSteps;

   private SimpleFootstep nextFootstep;
   private FootstepTiming nextFootstepTiming;

   private final YoBoolean useStepAdjustment = new YoBoolean(yoNamePrefix + "UseStepAdjustment", registry);
   private final YoBoolean footstepIsAdjustable = new YoBoolean(yoNamePrefix + "FootstepIsAdjustable", registry);
   private final YoBoolean shouldCheckForReachability = new YoBoolean(yoNamePrefix + "ShouldCheckForReachability", registry);
   private final YoBoolean hasPlanarRegionBeenAssigned = new YoBoolean(yoNamePrefix + "HasPlanarRegionBeenAssigned", registry);

   private final YoDouble swingDuration = new YoDouble(yoNamePrefix + "SwingDuration", registry);
   private final YoDouble nextTransferDuration = new YoDouble(yoNamePrefix + "NextTransferDuration", registry);
   private final YoDouble footstepMultiplier = new YoDouble(yoNamePrefix + "TotalFootstepMultiplier", registry);

   private final YoFramePose3D upcomingFootstep = new YoFramePose3D(yoNamePrefix + "UpcomingFootstepPose", worldFrame, registry);
   private final YoEnum<RobotSide> upcomingFootstepSide = new YoEnum<>(yoNamePrefix + "UpcomingFootstepSide", registry, RobotSide.class);
   private final RecyclingArrayList<Point2D> upcomingFootstepContactPoints = new RecyclingArrayList<>(Point2D.class);

   private final FramePoint3D referencePositionInControlPlane = new FramePoint3D();
   private final FramePoint3D tempPoint = new FramePoint3D();

   private final YoFrameVector2D footstepAdjustmentFromErrorInControlPlane = new YoFrameVector2D(yoNamePrefix + "footstepAdjustmentFromErrorInControlPlane",
                                                                                                 worldFrame,
                                                                                                 registry);
   private final YoFrameVector2D footstepAdjustmentInControlPlane = new YoFrameVector2D(yoNamePrefix + "footstepAdjustmentInControlPlane",
                                                                                                 worldFrame,
                                                                                                 registry);
   private final YoFrameVector2D deadbandedAdjustment = new YoFrameVector2D(yoNamePrefix + "DeadbandedAdjustment", worldFrame, registry);
   private final YoFrameVector2D totalStepAdjustment = new YoFrameVector2D(yoNamePrefix + "TotalStepAdjustment", worldFrame, registry);

   private final YoFramePose3D footstepSolution = new YoFramePose3D(yoNamePrefix + "FootstepSolutionLocation", worldFrame, registry);
   private final YoFramePoint2D adjustedSolutionInControlPlane = new YoFramePoint2D(yoNamePrefix + "adjustedSolutionInControlPlane", worldFrame, registry);

   private final YoBoolean isInSwing = new YoBoolean(yoNamePrefix + "IsInSwing", registry);
   private final YoDouble initialTime = new YoDouble(yoNamePrefix + "InitialTime", registry);
   private final YoDouble timeInCurrentState = new YoDouble(yoNamePrefix + "TimeInCurrentState", registry);
   private final YoDouble timeRemainingInState = new YoDouble(yoNamePrefix + "TimeRemainingInState", registry);

   private final YoDouble recursionTime = new YoDouble(yoNamePrefix + "RecursionTime", registry);
   private final YoDouble recursionMultiplier = new YoDouble(yoNamePrefix + "RecursionMultiplier", registry);

   private final YoBoolean swingSpeedUpEnabled = new YoBoolean(yoNamePrefix + "SwingSpeedUpEnabled", registry);
   private final YoDouble speedUpTime = new YoDouble(yoNamePrefix + "SpeedUpTime", registry);

   private final YoFrameVector2D icpError = new YoFrameVector2D(yoNamePrefix + "ICPError", "", worldFrame, registry);
   private final YoBoolean footstepWasAdjusted = new YoBoolean(yoNamePrefix + "FootstepWasAdjusted", registry);

   private final BooleanProvider useICPControlPlaneInStepAdjustment;
   private final DoubleProvider minimumTimeForStepAdjustment;
   private final DoubleParameter supportDistanceFromFront;
   private final DoubleParameter supportDistanceFromBack;
   private final DoubleParameter supportDistanceFromInside;
   private final DoubleParameter supportDistanceFromOutside;

   private final SideDependentList<FixedFrameConvexPolygon2DBasics> allowableAreasForCoP = new SideDependentList<>();

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

   private final ICPControlPlane icpControlPlane;
   private final BipedSupportPolygons bipedSupportPolygons;

   private final FramePoint3D vertexInWorld = new FramePoint3D();
   private final FrameConvexPolygon2D allowableAreaForCoPInFoot = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D allowableAreaForCoP = new FrameConvexPolygon2D();

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
      this.icpControlPlane = icpControlPolygons.getIcpControlPlane();
      this.bipedSupportPolygons = bipedSupportPolygons;

      allowStepAdjustment = new BooleanParameter(yoNamePrefix + "AllowStepAdjustment", registry, stepAdjustmentParameters.allowStepAdjustment());

      minimumFootstepMultiplier = new DoubleParameter(yoNamePrefix + "MinimumFootstepMultiplier",
                                                      registry,
                                                      stepAdjustmentParameters.getMinimumFootstepMultiplier());
      maximumTimeFromTransfer = new DoubleParameter(yoNamePrefix + "MaximumTimeFromTransfer",
                                                    registry,
                                                    stepAdjustmentParameters.maximumTimeFromTransferInFootstepMultiplier());
      minICPErrorForStepAdjustment = new DoubleParameter(yoNamePrefix + "MinICPErrorForStepAdjustment",
                                                         registry,
                                                         stepAdjustmentParameters.getMinICPErrorForStepAdjustment());

      transferDurationSplitFraction = new DoubleParameter(yoNamePrefix + "TransferDurationSplitFraction",
                                                          registry,
                                                          stepAdjustmentParameters.getTransferSplitFraction());

      useICPControlPlaneInStepAdjustment = new BooleanParameter(yoNamePrefix + "useICPControlPlaneInStepAdjustment",
                                                                registry,
                                                                stepAdjustmentParameters.useICPControlPlane());
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
      DoubleProvider lengthBackLimit = new DoubleParameter(yoNamePrefix + "MaxReachabilityBackwardLength", registry, steppingParameters.getMaxBackwardStepLength());
      DoubleProvider innerLimit = new DoubleParameter(yoNamePrefix + "MinReachabilityWidth", registry, steppingParameters.getMinStepWidth());
      DoubleProvider outerLimit = new DoubleParameter(yoNamePrefix + "MaxReachabilityWidth", registry, steppingParameters.getMaxStepWidth());
      DoubleProvider inPlaceWidth = new DoubleParameter(yoNamePrefix + "InPlaceWidth", registry, steppingParameters.getInPlaceWidth());

      useActualErrorInsteadOfResidual = new BooleanParameter("useActualErrorInsteadOfResidual", registry, false);
      considerErrorInAdjustment = new BooleanParameter(yoNamePrefix + "considerErrorInAdjustment",
                                                       registry,
                                                       stepAdjustmentParameters.considerICPErrorForStepAdjustment());
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
                                                                   null);
      oneStepSafetyHeuristics = new CaptureRegionSafetyHeuristics(lengthLimit, registry, null);
      multiStepCaptureRegionCalculator = new MultiStepCaptureRegionCalculator(reachabilityConstraintHandler, allowCrossOverSteps, registry, yoGraphicsListRegistry);
      environmentConstraintProvider = new EnvironmentConstraintHandler(icpControlPlane, contactableFeet, useICPControlPlaneInStepAdjustment,
                                                                       yoNamePrefix, registry, yoGraphicsListRegistry);

      for (RobotSide robotSide : RobotSide.values)
      {
         FixedFrameConvexPolygon2DBasics allowableAreaForCoP = new YoFrameConvexPolygon2D(robotSide.getCamelCaseName() + "AllowableAreaForCoP", soleZUpFrames.get(robotSide), 4, registry);
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
      footstepWasAdjusted.set(false);
      hasPlanarRegionBeenAssigned.set(false);
      captureRegionCalculator.hideCaptureRegion();
      oneStepSafetyHeuristics.reset();
      multiStepCaptureRegionCalculator.reset();
      environmentConstraintProvider.reset();
      nextFootstep = null;
      nextFootstepTiming = null;
   }

   @Override
   public void setFootstepAfterTheCurrentOne(SimpleFootstep nextFootstep, FootstepTiming nextFootstepTiming)
   {
      this.nextFootstep = nextFootstep;
      this.nextFootstepTiming = nextFootstepTiming;
   }

   @Override
   public void setFootstepToAdjust(SimpleFootstep footstep, double swingDuration, double nextTransferDuration)
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

         footstepSolution.set(footstepPose);

         this.swingDuration.set(swingDuration);

         footstepIsAdjustable.set(footstep.getIsAdjustable());
         shouldCheckForReachability.set(footstep.getShouldCheckReachability());
         useStepAdjustment.set(allowStepAdjustment.getValue() && footstepIsAdjustable.getBooleanValue());
         this.nextTransferDuration.set(nextTransferDuration);
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
         this.speedUpTime.add(swingSpeedUp);
      }
   }

   @Override
   public void setStepConstraintRegions(List<StepConstraintRegion> stepConstraintRegion)
   {
      environmentConstraintProvider.setStepConstraintRegions(stepConstraintRegion);
   }

   @Override
   public void initialize(double initialTime, RobotSide supportSide)
   {
      isInSwing.set(true);
      this.initialTime.set(initialTime);
      reachabilityConstraintHandler.initializeReachabilityConstraint(supportSide, upcomingFootstep);
      speedUpTime.set(0.0);
      footstepSolution.set(upcomingFootstep);
      totalStepAdjustment.setToZero();
   }

   @Override
   public void compute(double currentTime,
                       FramePoint2DReadOnly desiredICP,
                       FramePoint2DReadOnly currentICP,
                       FrameVector2DReadOnly residualICPError,
                       double omega0)
   {
      if (!isInSwing.getBooleanValue())
         return;

      footstepWasAdjusted.set(false);

      computeTimeInCurrentState(currentTime);
      computeTimeRemainingInState();

      if (timeRemainingInState.getValue() < minimumTimeForStepAdjustment.getValue())
         return;

      computeLimitedAreaForCoP();
      RobotSide swingSide = upcomingFootstepSide.getEnumValue();
      RobotSide stanceSide = swingSide.getOppositeSide();
      captureRegionCalculator.calculateCaptureRegion(swingSide,
                                                     Math.max(timeRemainingInState.getDoubleValue(), 0.0),
                                                     currentICP,
                                                     omega0,
                                                     allowableAreaForCoP);
      oneStepSafetyHeuristics.computeCaptureRegionWithSafetyHeuristics(stanceSide,
                                                                       currentICP,
                                                                       allowableAreaForCoP.getCentroid(),
                                                                       captureRegionCalculator.getCaptureRegion());
      multiStepCaptureRegionCalculator.compute(stanceSide,
                                               oneStepSafetyHeuristics.getCaptureRegionWithSafetyMargin(),
//                                               captureRegionCalculator.getCaptureRegion(),
                                               nextFootstepTiming == null ? Double.NaN : nextFootstepTiming.getStepTime(),
                                               omega0,
                                               nextFootstep == null ? 1 : 2); // fixme hardcoded.

      if (!useStepAdjustment.getBooleanValue())
      {
         if (shouldCheckForReachability.getValue())
         {
            boolean wasAdjusted = projectAdjustedStepIntoReachability();
            footstepWasAdjusted.set(wasAdjusted);

            if (wasAdjusted)
            {
               tempPoint.set(adjustedSolutionInControlPlane, upcomingFootstep.getPosition().getZ());
               footstepSolution.getPosition().set(tempPoint);

               if (CONTINUOUSLY_UPDATE_DESIRED_POSITION)
                  upcomingFootstep.set(footstepSolution);
            }
         }
         return;
      }

      icpError.sub(desiredICP, currentICP);


      boolean errorAboveThreshold = icpError.lengthSquared() > MathTools.square(minICPErrorForStepAdjustment.getValue());

      if (useICPControlPlaneInStepAdjustment.getValue())
         icpControlPlane.projectPointOntoControlPlane(worldFrame, upcomingFootstep.getPosition(), referencePositionInControlPlane);
      else
         referencePositionInControlPlane.set(upcomingFootstep.getPosition());

      if (errorAboveThreshold && considerErrorInAdjustment.getValue())
         computeStepAdjustmentFromError(residualICPError, omega0);
      else
         footstepAdjustmentFromErrorInControlPlane.setToZero();

      projectAdjustedStepIntoCaptureRegion();
      boolean wasAdjusted = deadbandAndApplyStepAdjustment();

      environmentConstraintProvider.setReachabilityRegion(reachabilityConstraintHandler.getReachabilityConstraint());
      environmentConstraintProvider.updateActiveConstraintRegionToUse(footstepSolution, multiStepCaptureRegionCalculator.getCaptureRegion());

      if (environmentConstraintProvider.hasStepConstraintRegion() && (wasAdjusted || !hasPlanarRegionBeenAssigned.getBooleanValue()))
      {
         if (environmentConstraintProvider.validateConvexityOfPlanarRegion())
         {
            wasAdjusted |= environmentConstraintProvider.applyEnvironmentConstraintToFootstep(upcomingFootstepSide.getEnumValue(),
                                                                                              footstepSolution,
                                                                                              upcomingFootstepContactPoints);
            hasPlanarRegionBeenAssigned.set(environmentConstraintProvider.foundSolution());
         }
      }

      footstepWasAdjusted.set(wasAdjusted);

      if (wasFootstepAdjusted() && CONTINUOUSLY_UPDATE_DESIRED_POSITION)
         upcomingFootstep.set(footstepSolution);
   }

   private void computeLimitedAreaForCoP()
   {
      RobotSide supportSide = upcomingFootstepSide.getEnumValue().getOppositeSide();
      FixedFrameConvexPolygon2DBasics shrunkSupport = allowableAreasForCoP.get(supportSide);
      shrunkSupport.setMatchingFrame(bipedSupportPolygons.getFootPolygonInSoleFrame(supportSide), false);

      for (int i = 0; i < shrunkSupport.getNumberOfVertices(); i++)
      {
         FixedFramePoint2DBasics point = shrunkSupport.getVertexUnsafe(i);
         if (point.getX() > 0.0)
            point.setX(Math.max(point.getX() - supportDistanceFromFront.getValue(), 0.0));
         else
            point.setX(Math.min(point.getX() + supportDistanceFromBack.getValue(), 0.0));

         if (supportSide == RobotSide.LEFT)
         {
            if (point.getY() > 0)
               point.setY(Math.max(point.getY() - supportDistanceFromOutside.getValue(), 0.0));
            else
               point.setY(Math.min(point.getY() + supportDistanceFromInside.getValue(), 0.0));
         }
         else
         {
            if (point.getY() > 0)
               point.setY(Math.max(point.getY() - supportDistanceFromInside.getValue(), 0.0));
            else
               point.setY(Math.min(point.getY() + supportDistanceFromOutside.getValue(), 0.0));
         }
      }

      allowableAreaForCoPInFoot.setIncludingFrame(shrunkSupport);

      if (useICPControlPlaneInStepAdjustment.getValue())
      {
         allowableAreaForCoP.clear();

         for (int i = 0; i < allowableAreaForCoPInFoot.getNumberOfVertices(); i++)
         {
            vertexInWorld.setMatchingFrame(allowableAreaForCoPInFoot.getVertex(i), 0.0);
            icpControlPlane.projectPointOntoControlPlane(worldFrame, vertexInWorld, tempPoint);
            allowableAreaForCoP.addVertex(tempPoint);
         }
         allowableAreaForCoP.update();
      }
      else
      {
         allowableAreaForCoP.setMatchingFrame(allowableAreaForCoPInFoot, false);
      }
   }

   private void computeStepAdjustmentFromError(FrameVector2DReadOnly residualICPError, double omega0)
   {
      footstepMultiplier.set(computeFootstepAdjustmentMultiplier(omega0));
      if (useActualErrorInsteadOfResidual.getValue())
      {
         footstepAdjustmentFromErrorInControlPlane.set(icpError);
         footstepAdjustmentFromErrorInControlPlane.negate();
      }
      else
      {
         footstepAdjustmentFromErrorInControlPlane.set(residualICPError);
      }
      footstepAdjustmentFromErrorInControlPlane.scale(1.0 / footstepMultiplier.getDoubleValue());
   }

   private void projectAdjustedStepIntoCaptureRegion()
   {
      adjustedSolutionInControlPlane.set(referencePositionInControlPlane);
      adjustedSolutionInControlPlane.add(footstepAdjustmentFromErrorInControlPlane);

      captureRegionInWorld.setIncludingFrame(multiStepCaptureRegionCalculator.getCaptureRegion());
      captureRegionInWorld.changeFrameAndProjectToXYPlane(worldFrame);

      if (!isTheCaptureRegionReachable())
      {
         captureRegionInWorld.orthogonalProjection(adjustedSolutionInControlPlane);
         getBestReachabilityConstraintToUseWhenNotIntersecting().orthogonalProjection(adjustedSolutionInControlPlane);
      }
      else
      {
         getBestReachabilityConstraintToUseWhenIntersecting().orthogonalProjection(adjustedSolutionInControlPlane);
      }

      footstepAdjustmentInControlPlane.set(adjustedSolutionInControlPlane);
      footstepAdjustmentInControlPlane.sub(referencePositionInControlPlane.getX(), referencePositionInControlPlane.getY());
   }

   private boolean projectAdjustedStepIntoReachability()
   {
      adjustedSolutionInControlPlane.set(referencePositionInControlPlane);

      FrameConvexPolygon2DReadOnly reachabilityPolygon = reachabilityConstraintHandler.getTotalReachabilityHull(upcomingFootstepSide.getEnumValue().getOppositeSide());
      if (!reachabilityPolygon.isPointInside(adjustedSolutionInControlPlane))
      {
         reachabilityConstraintHandler.getReachabilityConstraint().orthogonalProjection(adjustedSolutionInControlPlane);
         return true;
      }

      return false;
   }


   private boolean isTheCaptureRegionReachable()
   {
      boolean intersect = polygonTools.computeIntersectionOfPolygons(captureRegionInWorld, reachabilityConstraintHandler.getReachabilityConstraint(), reachableCaptureRegion);
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

      double distanceToForward = reachabilityConstraintHandler.getForwardCrossOverPolygon().distance(adjustedSolutionInControlPlane);
      double distanceToBackward = reachabilityConstraintHandler.getBackwardCrossOverPolygon().distance(adjustedSolutionInControlPlane);
      double distanceToNominal = reachabilityConstraintHandler.getReachabilityConstraint().distance(adjustedSolutionInControlPlane);

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
      if (footstepAdjustmentInControlPlane.length() < footstepDeadband.getValue())
      {
         adjusted = false;
         deadbandedAdjustment.setToZero();
      }
      else
      {
         adjusted = true;
         deadbandedAdjustment.set(footstepAdjustmentInControlPlane);
      }

      adjustedSolutionInControlPlane.set(referencePositionInControlPlane);
      adjustedSolutionInControlPlane.add(deadbandedAdjustment);

      totalStepAdjustment.add(deadbandedAdjustment);

      if (useICPControlPlaneInStepAdjustment.getValue())
         icpControlPlane.projectPointFromControlPlaneOntoSurface(worldFrame, adjustedSolutionInControlPlane, tempPoint, upcomingFootstep.getPosition().getZ());
      else
         tempPoint.set(adjustedSolutionInControlPlane, upcomingFootstep.getPosition().getZ());

      footstepSolution.getPosition().set(tempPoint);

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

   private double computeFootstepAdjustmentMultiplier(double omega0)
   {
      double timeInTransferForShifting = Math.min(maximumTimeFromTransfer.getValue(),
                                                  transferDurationSplitFraction.getValue() * nextTransferDuration.getDoubleValue());
      recursionTime.set(Math.max(timeRemainingInState.getDoubleValue(), 0.0) + timeInTransferForShifting);
      recursionMultiplier.set(Math.exp(-omega0 * recursionTime.getDoubleValue()));

      // This is the maximum possible multiplier
      double finalRecursionMultiplier = Math.exp(-omega0 * timeInTransferForShifting);

      // The recursion multiplier is guaranteed to be between the max and min values. This forces it to interpolate between those two.
      double minimumFootstepMultiplier = Math.min(this.minimumFootstepMultiplier.getValue(), finalRecursionMultiplier);
      return minimumFootstepMultiplier + (1.0 - minimumFootstepMultiplier / finalRecursionMultiplier) * recursionMultiplier.getDoubleValue();
   }
}
