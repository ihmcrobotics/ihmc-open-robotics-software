package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class StepConstraintCalculator
{
   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final ICPControlPlane icpControlPlane;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleProvider timeProvider;
   private final CapturabilityPlanarRegionDecider planarRegionDecider;

   private final FramePoint2D capturePoint = new FramePoint2D();

   private final SideDependentList<FrameConvexPolygon2D> supportPolygons = new SideDependentList<>(new FrameConvexPolygon2D(), new FrameConvexPolygon2D());

   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>();

   private SimpleStep currentStep;

   private double omega = 3.0;

   public StepConstraintCalculator(WalkingControllerParameters walkingControllerParameters,
                                   SideDependentList<? extends ReferenceFrame> soleZUpFrames,
                                   ReferenceFrame centerOfMassFrame,
                                   DoubleProvider timeProvider,
                                   double gravityZ)
   {
      this.timeProvider = timeProvider;
      this.captureRegionCalculator = new OneStepCaptureRegionCalculator(soleZUpFrames, walkingControllerParameters, registry, null);
      this.icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravityZ, registry);
      this.planarRegionDecider = new CapturabilityPlanarRegionDecider(captureRegionCalculator, icpControlPlane, registry, null);
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

   public void reset()
   {
      currentStep = null;
   }

   public void update()
   {
      icpControlPlane.setOmega0(omega);

      if (currentStep == null)
      {
         captureRegionCalculator.hideCaptureRegion();
      }
      else
      {
         updateCaptureRegion(currentStep);
         planarRegionDecider.updatePlanarRegionConstraintForStep(currentStep.getStepPose());
      }
   }

   private void updateCaptureRegion(SimpleStep simpleStep)
   {
      double timeInState = timeProvider.getValue() - simpleStep.getStartTime();
      double timeRemaining = simpleStep.getSwingDuration() - timeInState;
      RobotSide swingSide = simpleStep.getSwingSide();

      captureRegionCalculator.calculateCaptureRegion(swingSide, timeRemaining, capturePoint, omega, supportPolygons.get(swingSide.getOppositeSide()));
   }

   public boolean constraintRegionChanged()
   {
      return planarRegionDecider.constraintRegionChanged();
   }

   public PlanarRegion getConstraintRegion()
   {
      return planarRegionDecider.getConstraintRegion();
   }
}
