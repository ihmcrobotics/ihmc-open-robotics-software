package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.RegionInWorldInterface;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class CapturabilityBasedPlanarRegionDecider<T extends RegionInWorldInterface<T>>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double defaultMinimumIntersectionForSearch = 0.015;
   private static final double defaultAreaImprovementToSwitch = 0.02;
   private static final double defaultInflationToCurrentArea = 0.1;

   private final Supplier<T> regionProvider;

   private final List<T> stepConstraintRegions = new ArrayList<>();

   private final YoBoolean switchPlanarRegionConstraintsAutomatically;
   private final YoDouble minimumIntersectionForSearch;
   private final YoDouble areaImprovementToSwitch;
   private final YoDouble inflationToCurrentArea;

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   private final YoBoolean constraintRegionChanged;
   private final YoDouble intersectionAreaWithCurrentRegion;

   private final ConvexPolygon2D convexHullConstraintInControlPlane = new ConvexPolygon2D();

   private final FrameConvexPolygon2D reachableCaptureRegion = new FrameConvexPolygon2D();
   private final YoFrameConvexPolygon2D yoConvexHullConstraint;

   private final ICPControlPlane icpControlPlane;

   private T planarRegionToConstrainTo = null;

   public CapturabilityBasedPlanarRegionDecider(ReferenceFrame centerOfMassFrame,
                                                double gravityZ,
                                                Supplier<T> regionProvider,
                                                YoRegistry registry,
                                                YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravityZ, registry);
      this.regionProvider = regionProvider;

      constraintRegionChanged = new YoBoolean("constraintRegionChanged", registry);
      intersectionAreaWithCurrentRegion = new YoDouble("intersectionAreaWithCurrentRegion", registry);

      minimumIntersectionForSearch = new YoDouble("minimumIntersectionForSearch", registry);
      areaImprovementToSwitch = new YoDouble("areaImprovementToSwitch", registry);
      inflationToCurrentArea = new YoDouble("inflationToCurrentArea", registry);
      minimumIntersectionForSearch.set(defaultMinimumIntersectionForSearch);
      areaImprovementToSwitch.set(defaultAreaImprovementToSwitch);
      inflationToCurrentArea.set(defaultInflationToCurrentArea);

      switchPlanarRegionConstraintsAutomatically = new YoBoolean("switchPlanarRegionConstraintsAutomatically", registry);

      yoConvexHullConstraint = new YoFrameConvexPolygon2D("convexHullConstraint", "", worldFrame, 12, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon activePlanarRegionViz = new YoArtifactPolygon("ConvexHullConstraint", yoConvexHullConstraint, Color.RED, false);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionViz);
      }
   }

   public void setConstraintRegions(List<T> constraintRegions)
   {
      stepConstraintRegions.clear();
      stepConstraintRegions.addAll(constraintRegions);
   }

   public void setReachableCaptureRegion(FrameConvexPolygon2DReadOnly reachableCaptureRegion)
   {
      this.reachableCaptureRegion.setIncludingFrame(reachableCaptureRegion);
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
      intersectionAreaWithCurrentRegion.set(0.0);
      yoConvexHullConstraint.clear();
   }

   private void computeProjectedConvexHull(T constraintRegion)
   {
      yoConvexHullConstraint.set(constraintRegion.getConvexHull());
      yoConvexHullConstraint.applyTransform(constraintRegion.getTransformToWorld(), false);

      icpControlPlane.projectVerticesOntoControlPlane(constraintRegion.getConvexHull().getPolygonVerticesView(), constraintRegion.getTransformToWorld(), convexHullConstraintInControlPlane);
   }

   public T updatePlanarRegionConstraintForStep(FramePoint3DReadOnly footstepPosition)
   {
      constraintRegionChanged.set(false);

      reachableCaptureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      if (planarRegionToConstrainTo == null && footstepPosition != null)
      {
         planarRegionToConstrainTo = findPlanarRegionUnderFoothold(footstepPosition);

         if (planarRegionToConstrainTo != null)
         {
            computeProjectedConvexHull(planarRegionToConstrainTo);
            constraintRegionChanged.set(true);
         }
      }

      if (switchPlanarRegionConstraintsAutomatically.getBooleanValue() && !checkIfCurrentPlanarRegionIsValid(reachableCaptureRegion))
      {
         T betterRegion = findBestPlanarRegionToStepTo(reachableCaptureRegion);
         if (betterRegion != null && betterRegion != planarRegionToConstrainTo)
         {
            planarRegionToConstrainTo = betterRegion;
            computeProjectedConvexHull(planarRegionToConstrainTo);
            constraintRegionChanged.set(true);
         }
      }

      return planarRegionToConstrainTo;
   }

   public boolean constraintRegionChanged()
   {
      return constraintRegionChanged.getBooleanValue();
   }

   public void setConstraintRegionChanged(boolean constraintRegionChanged)
   {
      this.constraintRegionChanged.set(constraintRegionChanged);
   }

   public T getConstraintRegion()
   {
      return planarRegionToConstrainTo;
   }

   private T findPlanarRegionUnderFoothold(FramePoint3DReadOnly foothold)
   {
      T highestRegionUnderFoot = regionProvider.get();
      Point3D projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(foothold, stepConstraintRegions, highestRegionUnderFoot);

      return projectedPoint == null ? null : highestRegionUnderFoot;
   }

   private boolean checkIfCurrentPlanarRegionIsValid(FrameConvexPolygon2DReadOnly reachableCaptureRegion)
   {
      if (planarRegionToConstrainTo == null)
         return false;

      computeProjectedConvexHull(planarRegionToConstrainTo);

      intersectionAreaWithCurrentRegion.set(convexPolygonTools.computeIntersectionAreaOfPolygons(reachableCaptureRegion, convexHullConstraintInControlPlane));

      return intersectionAreaWithCurrentRegion.getDoubleValue() > minimumIntersectionForSearch.getDoubleValue();
   }

   private T findBestPlanarRegionToStepTo(FrameConvexPolygon2DReadOnly reachableCaptureRegion)
   {
      double maxArea = intersectionAreaWithCurrentRegion.getDoubleValue();
      if (maxArea > 0.0)
         maxArea = (1.0 + inflationToCurrentArea.getDoubleValue()) * maxArea + areaImprovementToSwitch.getDoubleValue();

      T activePlanarRegion = planarRegionToConstrainTo;

      for (int regionIndex = 0; regionIndex < stepConstraintRegions.size(); regionIndex++)
      {
         T constraintRegion = stepConstraintRegions.get(regionIndex);
         if (constraintRegion == activePlanarRegion)
            continue;

         double intersectionArea = findIntersectionAreaWithCaptureRegion(reachableCaptureRegion, constraintRegion);

         if (intersectionArea > maxArea)
         {
            maxArea = intersectionArea;
            activePlanarRegion = constraintRegion;
         }
      }

      return activePlanarRegion;
   }

   private double findIntersectionAreaWithCaptureRegion(FrameConvexPolygon2DReadOnly reachableCaptureRegionInControlPlane, T constraintRegion)
   {
      icpControlPlane.projectVerticesOntoControlPlane(constraintRegion.getConcaveHull(), constraintRegion.getTransformToWorld(), convexHullConstraintInControlPlane);

      double intersectionArea = convexPolygonTools.computeIntersectionAreaOfPolygons(reachableCaptureRegionInControlPlane, convexHullConstraintInControlPlane);

      if (constraintRegion instanceof StepConstraintRegion)
      {
         for (ConcavePolygon2DReadOnly convexPolygon : ((StepConstraintRegion) constraintRegion).getHolesInConstraintRegion())
         {
            icpControlPlane.projectVerticesOntoControlPlane(convexPolygon.getPolygonVerticesView(), constraintRegion.getTransformToWorld(), convexHullConstraintInControlPlane);

            intersectionArea -= convexPolygonTools.computeIntersectionAreaOfPolygons(reachableCaptureRegionInControlPlane, convexHullConstraintInControlPlane);
         }
      }

      return intersectionArea;
   }

}
