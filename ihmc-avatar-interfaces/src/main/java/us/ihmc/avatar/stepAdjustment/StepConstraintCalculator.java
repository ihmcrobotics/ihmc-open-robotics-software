package us.ihmc.avatar.stepAdjustment;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

import java.awt.Color;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.CapturabilityBasedPlanarRegionDecider;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class StepConstraintCalculator
{
   private final SteppableRegionsCalculator steppableRegionsCalculator;
   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
   private final DoubleProvider timeProvider;

   private final CapturabilityBasedPlanarRegionDecider planarRegionDecider;
   private final ReachabilityConstraintCalculator reachabilityConstraintCalculator;

   private final FrameConvexPolygon2D reachabilityRegionInConstraintPlane = new FrameConvexPolygon2D();
   private final YoFrameConvexPolygon2D yoReachabilityRegion = new YoFrameConvexPolygon2D("reachabilityPolygon", worldFrame, 10, registry);

   private static final double defaultMinimumTimeRemainingForSwitch = 0.05;
   private final YoDouble minimumTimeRemainingForSwitch = new YoDouble("minimumTimeRemainingForSwitch", registry);

   private final YoDouble timeRemainingInState = new YoDouble("timeRemainingInState", registry);
   private final YoFrameConvexPolygon2D supportPolygon = new YoFrameConvexPolygon2D("supportPolygon", worldFrame, 6, registry);
   private final YoFramePose3D supportPose = new YoFramePose3D("supportPose", worldFrame, registry);
   private final YoFramePoint2D capturePoint = new YoFramePoint2D("capturePoint", worldFrame, registry);

   private final SideDependentList<FrameConvexPolygon2D> supportPolygons = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());
   private final SideDependentList<? extends ReferenceFrame> soleZUpFrames;
   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>();

   private SimpleStep currentStep;
   private StepConstraintRegion stepConstraintRegion = null;

   private double omega = 3.0;

   public StepConstraintCalculator(WalkingControllerParameters walkingControllerParameters,
                                   SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                   DoubleProvider timeProvider)
   {
      this(soleZUpFrames,
           walkingControllerParameters.getSteppingParameters().getFootLength(),
           walkingControllerParameters.getSteppingParameters().getFootWidth(),
           walkingControllerParameters.getSteppingParameters().getMaxStepLength(),
           walkingControllerParameters.getSteppingParameters().getMaxBackwardStepLength(),
           walkingControllerParameters.getSteppingParameters().getMinStepWidth(),
           walkingControllerParameters.getSteppingParameters().getMaxStepWidth(),
           timeProvider);
   }

   public StepConstraintCalculator(SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                   double footLength,
                                   double footWidth,
                                   double kinematicStepRange,
                                   double maxBackwardStepLength,
                                   double minStepWidth,
                                   double maxStepWidth,
                                   DoubleProvider timeProvider)
   {
      this.timeProvider = timeProvider;
      this.soleZUpFrames = soleZUpFrames;
      this.steppableRegionsCalculator = new SteppableRegionsCalculator(kinematicStepRange, registry);
      this.captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth, kinematicStepRange, soleZUpFrames, registry, graphicsListRegistry);
      this.planarRegionDecider = new CapturabilityBasedPlanarRegionDecider(registry, graphicsListRegistry);
      this.reachabilityConstraintCalculator = new ReachabilityConstraintCalculator(soleZUpFrames,
                                                                                   footLength,
                                                                                   footWidth,
                                                                                   kinematicStepRange,
                                                                                   maxBackwardStepLength,
                                                                                   minStepWidth,
                                                                                   maxStepWidth,
                                                                                   registry);

      minimumTimeRemainingForSwitch.set(defaultMinimumTimeRemainingForSwitch);

      YoGraphicPosition capturePointViz = new YoGraphicPosition("CapturePoint", capturePoint, 0.02, YoAppearance.Yellow(), GraphicType.BALL_WITH_CROSS);
      YoArtifactPolygon supportPolygonArtifact = new YoArtifactPolygon("SupportPolygon", supportPolygon, Color.GREEN, false);
      YoArtifactPolygon reachabilityArtifact = new YoArtifactPolygon("ReachabilityPolygon", yoReachabilityRegion, Color.BLUE, false);

      graphicsListRegistry.registerYoGraphic("Constraint Calculator", capturePointViz);
      graphicsListRegistry.registerArtifact("Constraint Calculator", capturePointViz.createArtifact());
      graphicsListRegistry.registerArtifact("Constraint Calculator", supportPolygonArtifact);
      graphicsListRegistry.registerArtifact("Constraint Calculator", reachabilityArtifact);
   }

   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return graphicsListRegistry;
   }

   public void setLeftFootSupportPolygon(List<? extends Point3DReadOnly> footVertices)
   {
      setFootSupportPolygon(RobotSide.LEFT, footVertices);
   }

   public void setRightFootSupportPolygon(List<? extends Point3DReadOnly> footVertices)
   {
      setFootSupportPolygon(RobotSide.RIGHT, footVertices);
   }

   public void setFootSupportPolygon(RobotSide supportSide, List<? extends Point3DReadOnly> footVertices)
   {
      FrameConvexPolygon2DBasics supportPolygon = supportPolygons.get(supportSide);
      supportPolygon.clear();
      footVertices.forEach(supportPolygon::addVertex);
      supportPolygon.update();
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public void setCapturePoint(Point3DReadOnly capturePointInWorld)
   {
      capturePoint.set(capturePointInWorld);
   }

   public void setCurrentStep(SimpleStep simpleStep)
   {
      currentStep = simpleStep;
   }

   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      planarRegionsList.set(planarRegions);
   }

   public void setSwitchPlanarRegionConstraintsAutomatically(boolean switchAutomatically)
   {
      planarRegionDecider.setSwitchPlanarRegionConstraintsAutomatically(switchAutomatically);
   }

   public void reset()
   {
      currentStep = null;
      planarRegionDecider.reset();
      supportPolygon.clearAndUpdate();
      capturePoint.setToNaN();
      supportPose.setToNaN();
   }

   public void update()
   {
      if (currentStep == null)
      {
         captureRegionCalculator.hideCaptureRegion();

         stepConstraintRegion = null;
      }
      else
      {
         updateCaptureRegion(currentStep);

         steppableRegionsCalculator.setPlanarRegions(planarRegionsList.get().getPlanarRegionsAsList());
         FramePoint3D supportFoot = new FramePoint3D(soleZUpFrames.get(currentStep.getSwingSide().getOppositeSide()));
         supportFoot.changeFrame(worldFrame);
         supportPose.setToZero();
         supportPose.getPosition().set(supportFoot);
         steppableRegionsCalculator.setStanceFootPosition(supportFoot);

         List<StepConstraintRegion> steppableRegions = steppableRegionsCalculator.computeSteppableRegions();

         updateReachabilityRegion(currentStep.getSwingSide().getOppositeSide());

         if (timeRemainingInState.getDoubleValue() < minimumTimeRemainingForSwitch.getDoubleValue())
            return;

         planarRegionDecider.setConstraintRegions(steppableRegions);
         planarRegionDecider.setCaptureRegion(captureRegionCalculator.getCaptureRegion());
         planarRegionDecider.updatePlanarRegionConstraintForStep(currentStep.getStepPose(), yoReachabilityRegion);

         updateReachabilityRegionInControlPlane();

         stepConstraintRegion = computeConstraintRegion();
      }
   }

   private void updateCaptureRegion(SimpleStep simpleStep)
   {
      double timeInState = timeProvider.getValue() - simpleStep.getStartTime();
      double timeRemaining = simpleStep.getSwingDuration() - timeInState;
      timeRemainingInState.set(timeRemaining);
      RobotSide swingSide = simpleStep.getSwingSide();

      supportPolygon.set(supportPolygons.get(swingSide.getOppositeSide()));
      captureRegionCalculator.calculateCaptureRegion(swingSide, timeRemaining, capturePoint, omega, supportPolygons.get(swingSide.getOppositeSide()));
   }

   public void updateReachabilityRegion(RobotSide supportSide)
   {
      FrameConvexPolygon2DReadOnly reachabilityRegion = reachabilityConstraintCalculator.getReachabilityPolygon(supportSide);
      reachabilityRegionInConstraintPlane.clear();
      reachabilityRegionInConstraintPlane.setIncludingFrame(reachabilityRegion);
      reachabilityRegionInConstraintPlane.changeFrame(worldFrame);
      yoReachabilityRegion.set(reachabilityRegionInConstraintPlane);
   }

   public void updateReachabilityRegionInControlPlane()
   {
      StepConstraintRegion stepConstraintRegion = planarRegionDecider.getConstraintRegion();

      if (stepConstraintRegion == null)
         return;

      reachabilityRegionInConstraintPlane.set(yoReachabilityRegion);
      reachabilityRegionInConstraintPlane.applyTransform(stepConstraintRegion.getTransformToLocal(), false);
   }

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   private StepConstraintRegion computeConstraintRegion()
   {
      StepConstraintRegion planarRegionForConstraint = planarRegionDecider.getConstraintRegion();

      if (planarRegionForConstraint == null)
         return null;

      ConvexPolygon2D constraintConvexHull = new ConvexPolygon2D();
      convexPolygonTools.computeIntersectionOfPolygons(planarRegionForConstraint.getConvexHullInConstraintRegion(), reachabilityRegionInConstraintPlane, constraintConvexHull);

      return new StepConstraintRegion(planarRegionForConstraint.getTransformToWorld(), constraintConvexHull);
   }

   public boolean constraintRegionChanged()
   {
      return planarRegionDecider.constraintRegionChanged();
   }

   public StepConstraintRegion getConstraintRegion()
   {
      return stepConstraintRegion;
   }

   public StepConstraintRegion pollStepConstraintRegion()
   {
      if (planarRegionDecider.constraintRegionChanged())
      {
         StepConstraintRegion regionToReturn = stepConstraintRegion;
         planarRegionDecider.setConstraintRegionChanged(false);
         stepConstraintRegion = null;
         return regionToReturn;
      }
      else
      {
         return null;
      }
   }
}
