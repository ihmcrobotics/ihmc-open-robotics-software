package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import org.ojalgo.function.multiary.MultiaryFunction.Convex;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class StepConstraintCalculator
{
   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleProvider timeProvider;

   private final CapturabilityBasedPlanarRegionDecider planarRegionDecider;
   private final ReachabilityConstraintCalculator reachabilityConstraintCalculator;

   private final ConvexPolygon2D reachabilityRegionInConstraintPlane = new ConvexPolygon2D();

   private final FramePoint2D capturePoint = new FramePoint2D();

   private final SideDependentList<FrameConvexPolygon2D> supportPolygons = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());

   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>();

   private SimpleStep currentStep;
   private StepConstraintRegion stepConstraintRegion = null;

   private double omega = 3.0;

   public StepConstraintCalculator(WalkingControllerParameters walkingControllerParameters,
                                   SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                   ReferenceFrame centerOfMassFrame,
                                   DoubleProvider timeProvider,
                                   double gravityZ)
   {
      this(soleZUpFrames,
           centerOfMassFrame,
           walkingControllerParameters.getSteppingParameters().getFootWidth(),
           walkingControllerParameters.getSteppingParameters().getMaxStepLength(),
           walkingControllerParameters.getSteppingParameters().getMaxBackwardStepLength(),
           walkingControllerParameters.getSteppingParameters().getMinStepWidth(),
           walkingControllerParameters.getSteppingParameters().getMaxStepWidth(),
           timeProvider,
           gravityZ);
   }

   public StepConstraintCalculator(SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                   ReferenceFrame centerOfMassFrame,
                                   double footWidth,
                                   double kinematicStepRange,
                                   double maxBackwardStepLength,
                                   double minStepWidth,
                                   double maxStepWidth,
                                   DoubleProvider timeProvider,
                                   double gravityZ)
   {
      this.timeProvider = timeProvider;
      this.captureRegionCalculator = new OneStepCaptureRegionCalculator(footWidth, kinematicStepRange, soleZUpFrames, registry, null);
      this.planarRegionDecider = new CapturabilityBasedPlanarRegionDecider(centerOfMassFrame, gravityZ, registry, null);
      this.reachabilityConstraintCalculator = new ReachabilityConstraintCalculator(soleZUpFrames,
                                                                                   kinematicStepRange,
                                                                                   maxBackwardStepLength,
                                                                                   minStepWidth,
                                                                                   maxStepWidth,
                                                                                   registry);
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
      supportPolygon.addVertices(Vertex3DSupplier.asVertex3DSupplier(footVertices));
      supportPolygon.update();
   }

   public void setOmega(double omega)
   {
      this.omega = omega;
   }

   public void setCapturePoint(Point3DReadOnly capturePointInWorld)
   {
      capturePoint.setIncludingFrame(ReferenceFrame.getWorldFrame(), capturePointInWorld);
   }

   public void setCurrentStep(SimpleStep simpleStep)
   {
      currentStep = simpleStep;
   }

   public void setPlanarRegions(PlanarRegionsList planarRegions)
   {
      planarRegionsList.set(planarRegions);
      planarRegionDecider.setPlanarRegions(planarRegions.getPlanarRegionsAsList());
   }

   public void setSwitchPlanarRegionConstraintsAutomatically(boolean switchAutomatically)
   {
      planarRegionDecider.setSwitchPlanarRegionConstraintsAutomatically(switchAutomatically);
   }

   public void reset()
   {
      currentStep = null;
      planarRegionDecider.reset();
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

         planarRegionDecider.setOmega0(omega);
         planarRegionDecider.setCaptureRegion(captureRegionCalculator.getCaptureRegion());
         planarRegionDecider.updatePlanarRegionConstraintForStep(currentStep.getStepPose());

         updateReachabilityRegionInControlPlane(currentStep.getSwingSide().getOppositeSide());

         stepConstraintRegion = computeConstraintRegion();
      }
   }

   private void updateCaptureRegion(SimpleStep simpleStep)
   {
      double timeInState = timeProvider.getValue() - simpleStep.getStartTime();
      double timeRemaining = simpleStep.getSwingDuration() - timeInState;
      RobotSide swingSide = simpleStep.getSwingSide();

      captureRegionCalculator.calculateCaptureRegion(swingSide, timeRemaining, capturePoint, omega, supportPolygons.get(swingSide.getOppositeSide()));
   }

   public void updateReachabilityRegionInControlPlane(RobotSide supportSide)
   {
      PlanarRegion planarRegionForConstraint = planarRegionDecider.getConstraintRegion();

      if (planarRegionForConstraint == null)
         return;

      FrameConvexPolygon2DReadOnly reachabilityRegion = reachabilityConstraintCalculator.getReachabilityPolygon(supportSide);
      reachabilityRegionInConstraintPlane.clear();
      reachabilityRegionInConstraintPlane.set(reachabilityRegion);
      reachabilityRegionInConstraintPlane.applyTransform(planarRegionForConstraint.getTransformToLocal(), false);
   }

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   private StepConstraintRegion computeConstraintRegion()
   {
      PlanarRegion planarRegionForConstraint = planarRegionDecider.getConstraintRegion();

      if (planarRegionForConstraint == null)
         return null;

      ConvexPolygon2D convexHull = new ConvexPolygon2D(planarRegionForConstraint.getConvexHull());
      convexHull.applyTransform(planarRegionForConstraint.getTransformToWorld(), false);

      ConvexPolygon2D constraintConvexHull = new ConvexPolygon2D();
      convexPolygonTools.computeIntersectionOfPolygons(convexHull, reachabilityRegionInConstraintPlane, constraintConvexHull);

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
}
