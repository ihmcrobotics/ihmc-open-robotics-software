package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.ArtifactList;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.DefaultPoint2DGraphic;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class CaptureRegionStepAdjustmentController implements StepAdjustmentController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private static final boolean VISUALIZE = true;
   private static final boolean CONTINUOUSLY_UPDATE_DESIRED_POSITION = true;

   private static final String yoNamePrefix = "controller";
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final BooleanProvider allowStepAdjustment;

   private final YoBoolean useStepAdjustment = new YoBoolean(yoNamePrefix + "UseStepAdjustment", registry);
   private final YoBoolean footstepIsAdjustable = new YoBoolean(yoNamePrefix + "FootstepIsAdjustable", registry);

   private final YoDouble swingDuration = new YoDouble(yoNamePrefix + "SwingDuration", registry);

   private final YoFramePose3D upcomingFootstep = new YoFramePose3D(yoNamePrefix + "UpcomingFootstepPose", worldFrame, registry);
   private final YoEnum<RobotSide> upcomingFootstepSide = new YoEnum<>(yoNamePrefix + "UpcomingFootstepSide", registry, RobotSide.class);
   private final RecyclingArrayList<Point2D> upcomingFootstepContactPoints = new RecyclingArrayList<>(Point2D.class);

   private final YoFramePose3D footstepSolution = new YoFramePose3D(yoNamePrefix + "FootstepSolutionLocation", worldFrame, registry);
   private final YoFramePoint2D adjustedSolutionInControlPlane = new YoFramePoint2D(yoNamePrefix + "adjustedSolutionInControlPlane", worldFrame, registry);

   private final YoBoolean isInSwing = new YoBoolean(yoNamePrefix + "IsInSwing", registry);
   private final YoDouble initialTime = new YoDouble(yoNamePrefix + "InitialTime", registry);
   private final YoDouble timeInCurrentState = new YoDouble(yoNamePrefix + "TimeInCurrentState", registry);
   private final YoDouble timeRemainingInState = new YoDouble(yoNamePrefix + "TimeRemainingInState", registry);

   private final YoBoolean swingSpeedUpEnabled = new YoBoolean(yoNamePrefix + "SwingSpeedUpEnabled", registry);
   private final YoDouble speedUpTime = new YoDouble(yoNamePrefix + "SpeedUpTime", registry);

   private final YoBoolean footstepWasAdjusted = new YoBoolean(yoNamePrefix + "FootstepWasAdjusted", registry);

   private final StepAdjustmentReachabilityConstraint reachabilityConstraintHandler;
   private final OneStepCaptureRegionCalculator captureRegionCalculator;

   private final FrameConvexPolygon2D captureRegionInWorld = new FrameConvexPolygon2D();

   private final DoubleProvider distanceInsideToScaleSupportPolygon = new DoubleParameter(yoNamePrefix + "distanceInsideToScaleSupportPolygon", registry, 0.02);
   private final FrameConvexPolygon2D allowableAreaForCoP = new FrameConvexPolygon2D();

   private final ConvexPolygonScaler polygonScaler = new ConvexPolygonScaler();

   private final BipedSupportPolygons bipedSupportPolygons;

   public CaptureRegionStepAdjustmentController(WalkingControllerParameters walkingControllerParameters,
                                                SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                                BipedSupportPolygons bipedSupportPolygons,
                                                YoRegistry parentRegistry,
                                                YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(walkingControllerParameters,
           walkingControllerParameters.getStepAdjustmentParameters(),
           soleZUpFrames,
           bipedSupportPolygons,
           parentRegistry,
           yoGraphicsListRegistry);
   }

   public CaptureRegionStepAdjustmentController(WalkingControllerParameters walkingControllerParameters,
                                                StepAdjustmentParameters stepAdjustmentParameters,
                                                SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                                BipedSupportPolygons bipedSupportPolygons,
                                                YoRegistry parentRegistry,
                                                YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.bipedSupportPolygons = bipedSupportPolygons;

      allowStepAdjustment = new BooleanParameter(yoNamePrefix + "AllowStepAdjustment", registry, stepAdjustmentParameters.allowStepAdjustment());

      reachabilityConstraintHandler = new StepAdjustmentReachabilityConstraint(soleZUpFrames,
                                                                               walkingControllerParameters.getSteppingParameters(),
                                                                               walkingControllerParameters.getStepAdjustmentParameters()
                                                                                                          .getCrossOverReachabilityParameters(),
                                                                               yoNamePrefix,
                                                                               VISUALIZE,
                                                                               registry,
                                                                               yoGraphicsListRegistry);

      captureRegionCalculator = new OneStepCaptureRegionCalculator(soleZUpFrames,
                                                                   walkingControllerParameters,
                                                                   false,
                                                                   yoNamePrefix,
                                                                   registry,
                                                                   yoGraphicsListRegistry);

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
      captureRegionCalculator.hideCaptureRegion();
   }

   @Override
   public void setFootstepQueueInformation(int numberOfStepsInQueue, double subsequentStepDuration)
   {
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

         footstepSolution.set(footstepPose);

         this.swingDuration.set(swingDuration);

         footstepIsAdjustable.set(footstep.getIsAdjustable());
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
      if (swingSpeedUpEnabled.getBooleanValue() && swingSpeedUp > speedUpTime.getDoubleValue())
      {
         this.speedUpTime.add(swingSpeedUp);
      }
   }

   @Override
   public void setStepConstraintRegions(List<StepConstraintRegion> stepConstraintRegions)
   {
   }

   @Override
   public List<StepConstraintRegion> getStepConstraintRegions()
   {
      return null;
   }

   @Override
   public void initialize(double initialTime, RobotSide supportSide)
   {
      isInSwing.set(true);
      this.initialTime.set(initialTime);
      reachabilityConstraintHandler.initializeReachabilityConstraint(supportSide);
      speedUpTime.set(0.0);
      footstepSolution.set(upcomingFootstep);
   }

   @Override
   public void compute(double currentTime,
                       FramePoint2DReadOnly desiredICP,
                       FramePoint2DReadOnly currentICP,
                       double omega0,
                       FramePoint2DReadOnly copToShrinkAbout,
                       double percentageToShrinkPolygon)
   {
      if (!isInSwing.getBooleanValue())
         return;

      computeTimeInCurrentState(currentTime);
      computeTimeRemainingInState();

      computeLimitedAreaForCoP();

      captureRegionCalculator.calculateCaptureRegion(upcomingFootstepSide.getEnumValue(),
                                                     timeRemainingInState.getDoubleValue(),
                                                     currentICP,
                                                     omega0,
                                                     allowableAreaForCoP);

      if (!useStepAdjustment.getBooleanValue())
         return;

      boolean wasAdjusted = adjustStepForError();

      footstepWasAdjusted.set(wasAdjusted);

      if (wasFootstepAdjusted() && CONTINUOUSLY_UPDATE_DESIRED_POSITION)
         upcomingFootstep.set(footstepSolution);
   }

   private boolean adjustStepForError()
   {
      adjustedSolutionInControlPlane.set(upcomingFootstep.getPosition());
      captureRegionInWorld.setIncludingFrame(captureRegionCalculator.getCaptureRegion());
      captureRegionInWorld.changeFrameAndProjectToXYPlane(worldFrame);

      boolean adjusted = captureRegionInWorld.orthogonalProjection(adjustedSolutionInControlPlane);
      reachabilityConstraintHandler.getReachabilityConstraint().orthogonalProjection(adjustedSolutionInControlPlane);

      footstepSolution.getPosition().set(adjustedSolutionInControlPlane);

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

   private void computeLimitedAreaForCoP()
   {
      FrameConvexPolygon2DReadOnly supportPolygon = bipedSupportPolygons.getFootPolygonInWorldFrame(upcomingFootstepSide.getEnumValue().getOppositeSide());
      polygonScaler.scaleConvexPolygon(supportPolygon, distanceInsideToScaleSupportPolygon.getValue(), allowableAreaForCoP);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(reachabilityConstraintHandler.getSCS2YoGraphics());
      group.addChild(captureRegionCalculator.getSCS2YoGraphics());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint2D(yoNamePrefix + "FootstepSolution",
                                                                    this.footstepSolution.getPosition(),
                                                                    0.01,
                                                                    ColorDefinitions.DarkRed(),
                                                                    DefaultPoint2DGraphic.CIRCLE));
      group.setVisible(VISUALIZE);
      return group;
   }
}
