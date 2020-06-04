package us.ihmc.avatar.stepAdjustment;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

public class CapturabilityBasedPlanarRegionDecider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double minimumIntersectionForSearch = 0.005;

   private final List<StepConstraintRegion> stepConstraintRegions = new ArrayList<>();

   private final YoBoolean switchPlanarRegionConstraintsAutomatically;

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   private final YoBoolean constraintRegionChanged;

   private final ConvexPolygon2D convexHullConstraintInControlPlane = new ConvexPolygon2D();

   private final FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D();
   private final YoFrameConvexPolygon2D yoConvexHullConstraint;
   private final YoFrameConvexPolygon2D yoConvexHullConstraintInControlPlane;

   private final ICPControlPlane icpControlPlane;

   private StepConstraintRegion planarRegionToConstrainTo = null;

   public CapturabilityBasedPlanarRegionDecider(ReferenceFrame centerOfMassFrame,
                                                double gravityZ,
                                                YoVariableRegistry registry,
                                                YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravityZ, registry);

      constraintRegionChanged = new YoBoolean("constraintRegionChanged", registry);

      switchPlanarRegionConstraintsAutomatically = new YoBoolean("switchPlanarRegionConstraintsAutomatically", registry);

      yoConvexHullConstraint = new YoFrameConvexPolygon2D("convexHullConstraint", "", worldFrame, 12, registry);
      yoConvexHullConstraintInControlPlane = new YoFrameConvexPolygon2D("convexHullConstraintInControlPlane", "", worldFrame, 12, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon activePlanarRegionViz = new YoArtifactPolygon("ConvexHullConstraint", yoConvexHullConstraint, Color.RED, false);
         YoArtifactPolygon activePlanarRegionInControlPlaneViz = new YoArtifactPolygon("ConvexHullConstraintInControlPlane",
                                                                                       yoConvexHullConstraintInControlPlane,
                                                                                       Color.PINK,
                                                                                       false);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionViz);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionInControlPlaneViz);
      }
   }

   public void setConstraintRegions(List<StepConstraintRegion> constraintRegions)
   {
      stepConstraintRegions.clear();
      stepConstraintRegions.addAll(constraintRegions);
   }

   public void setCaptureRegion(FrameConvexPolygon2DReadOnly captureRegion)
   {
      this.captureRegion.setIncludingFrame(captureRegion);
   }

   public void setOmega0(double omega)
   {
      icpControlPlane.setOmega0(omega);
   }

   public void setSwitchPlanarRegionConstraintsAutomatically(boolean switchAutomatically)
   {
      switchPlanarRegionConstraintsAutomatically.set(switchAutomatically);
   }

   public void reset()
   {
      planarRegionToConstrainTo = null;
      yoConvexHullConstraint.clear();
      yoConvexHullConstraintInControlPlane.clear();
   }

   private void computeShrunkAndProjectedConvexHulls(StepConstraintRegion constraintRegion)
   {
      yoConvexHullConstraint.set(constraintRegion.getConvexHullInConstraintRegion());
      yoConvexHullConstraint.applyTransform(constraintRegion.getTransformToWorld(), false);

      icpControlPlane.projectVerticesOntoControlPlane(constraintRegion.getConvexHullInConstraintRegion(), constraintRegion.getTransformToWorld(), yoConvexHullConstraintInControlPlane);
   }

   public StepConstraintRegion updatePlanarRegionConstraintForStep(FramePose3DReadOnly footstepPose)
   {
      constraintRegionChanged.set(false);

      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      if (planarRegionToConstrainTo == null)
      {
         planarRegionToConstrainTo = findPlanarRegionUnderFoothold(footstepPose.getPosition());

         if (planarRegionToConstrainTo != null)
         {
            computeShrunkAndProjectedConvexHulls(planarRegionToConstrainTo);
            constraintRegionChanged.set(true);
         }
      }

      if (switchPlanarRegionConstraintsAutomatically.getBooleanValue() && !checkIfCurrentPlanarRegionIsValid(captureRegion))
      {
         StepConstraintRegion betterRegion = findBestPlanarRegionToStepTo(captureRegion);
         if (betterRegion != null)
         {
            planarRegionToConstrainTo = betterRegion;
            computeShrunkAndProjectedConvexHulls(planarRegionToConstrainTo);
            constraintRegionChanged.set(true);
         }
      }

      return planarRegionToConstrainTo;
   }

   public boolean constraintRegionChanged()
   {
      return constraintRegionChanged.getBooleanValue();
   }

   public StepConstraintRegion getConstraintRegion()
   {
      return planarRegionToConstrainTo;
   }

   private StepConstraintRegion findPlanarRegionUnderFoothold(FramePoint3DReadOnly foothold)
   {
      StepConstraintRegion highestRegionUnderFoot = new StepConstraintRegion();
      Point3D projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(foothold, stepConstraintRegions, highestRegionUnderFoot);

      return projectedPoint == null ? null : highestRegionUnderFoot;
   }

   private boolean checkIfCurrentPlanarRegionIsValid(FrameConvexPolygon2DReadOnly captureRegion)
   {
      double intersectionArea = convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegion, yoConvexHullConstraintInControlPlane);

      return intersectionArea > minimumIntersectionForSearch;
   }

   private StepConstraintRegion findBestPlanarRegionToStepTo(FrameConvexPolygon2DReadOnly captureRegion)
   {
      double maxArea = 0.0;
      StepConstraintRegion activePlanarRegion = null;

      for (int regionIndex = 0; regionIndex < stepConstraintRegions.size(); regionIndex++)
      {
         StepConstraintRegion constraintRegion = stepConstraintRegions.get(regionIndex);

         double intersectionArea = findIntersectionAreaWithCaptureRegion(captureRegion, constraintRegion);

         if (intersectionArea > maxArea)
         {
            maxArea = intersectionArea;
            activePlanarRegion = constraintRegion;
         }
      }

      return activePlanarRegion;
   }

   private double findIntersectionAreaWithCaptureRegion(FrameConvexPolygon2DReadOnly captureRegionInControlPlane, StepConstraintRegion constraintRegion)
   {
      icpControlPlane.projectVerticesOntoControlPlane(constraintRegion.getConcaveHull(), constraintRegion.getTransformToWorld(), convexHullConstraintInControlPlane);
      double intersectionArea = convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegionInControlPlane, convexHullConstraintInControlPlane);

      for (ConcavePolygon2DReadOnly convexPolygon : constraintRegion.getHolesInConstraintRegion())
      {
         icpControlPlane.projectVerticesOntoControlPlane(convexPolygon, constraintRegion.getTransformToWorld(), convexHullConstraintInControlPlane);
         intersectionArea -= convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegionInControlPlane, convexHullConstraintInControlPlane);
      }

      return intersectionArea;
   }

}
