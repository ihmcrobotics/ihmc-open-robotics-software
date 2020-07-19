package us.ihmc.avatar.stepAdjustment;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

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
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class CapturabilityBasedPlanarRegionDecider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double defaultMinimumIntersectionForSearch = 0.015;
   private static final double defaultAreaImprovementToSwitch = 0.02;
   private static final double defaultInflationToCurrentArea = 0.1;

   private final List<StepConstraintRegion> stepConstraintRegions = new ArrayList<>();

   private final YoBoolean switchPlanarRegionConstraintsAutomatically;
   private final YoDouble minimumIntersectionForSearch;
   private final YoDouble areaImprovementToSwitch;
   private final YoDouble inflationToCurrentArea;

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   private final YoBoolean constraintRegionChanged;
   private final YoDouble intersectionAreaWithCurrentRegion;

   private final ConvexPolygon2D convexHullConstraintInControlPlane = new ConvexPolygon2D();
   private final ConvexPolygon2D possibleArea = new ConvexPolygon2D();

   private final FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D();
   private final YoFrameConvexPolygon2D yoConvexHullConstraint;

   private final ICPControlPlane icpControlPlane;

   private StepConstraintRegion planarRegionToConstrainTo = null;

   public CapturabilityBasedPlanarRegionDecider(ReferenceFrame centerOfMassFrame,
                                                double gravityZ,
                                                YoRegistry registry,
                                                YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravityZ, registry);

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
      intersectionAreaWithCurrentRegion.set(0.0);
      yoConvexHullConstraint.clear();
   }

   private void computeProjectedConvexHull(StepConstraintRegion constraintRegion)
   {
      yoConvexHullConstraint.set(constraintRegion.getConvexHullInConstraintRegion());
      yoConvexHullConstraint.applyTransform(constraintRegion.getTransformToWorld(), false);

      icpControlPlane.projectVerticesOntoControlPlane(constraintRegion.getConvexHullInConstraintRegion(), constraintRegion.getTransformToWorld(), convexHullConstraintInControlPlane);
   }

   public StepConstraintRegion updatePlanarRegionConstraintForStep(FramePose3DReadOnly footstepPose, ConvexPolygon2DReadOnly reachabilityInControlPlane)
   {
      constraintRegionChanged.set(false);

      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      if (planarRegionToConstrainTo == null)
      {
         planarRegionToConstrainTo = findPlanarRegionUnderFoothold(footstepPose.getPosition());

         if (planarRegionToConstrainTo != null)
         {
            computeProjectedConvexHull(planarRegionToConstrainTo);
            constraintRegionChanged.set(true);
         }
      }

      if (switchPlanarRegionConstraintsAutomatically.getBooleanValue() && !checkIfCurrentPlanarRegionIsValid(captureRegion, reachabilityInControlPlane))
      {
         StepConstraintRegion betterRegion = findBestPlanarRegionToStepTo(captureRegion, reachabilityInControlPlane);
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

   private boolean checkIfCurrentPlanarRegionIsValid(FrameConvexPolygon2DReadOnly captureRegion, ConvexPolygon2DReadOnly reachabilityRegion)
   {
      if (planarRegionToConstrainTo == null)
         return false;

      computeProjectedConvexHull(planarRegionToConstrainTo);
      if (reachabilityRegion != null)
         convexPolygonTools.computeIntersectionOfPolygons(convexHullConstraintInControlPlane, reachabilityRegion, possibleArea);
      else
         possibleArea.set(convexHullConstraintInControlPlane);

      intersectionAreaWithCurrentRegion.set(convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegion, possibleArea));

      return intersectionAreaWithCurrentRegion.getDoubleValue() > minimumIntersectionForSearch.getDoubleValue();
   }

   private StepConstraintRegion findBestPlanarRegionToStepTo(FrameConvexPolygon2DReadOnly captureRegion, ConvexPolygon2DReadOnly reachabilityRegion)
   {
      double maxArea = intersectionAreaWithCurrentRegion.getDoubleValue();
      if (maxArea > 0.0)
         maxArea = (1.0 + inflationToCurrentArea.getDoubleValue()) * maxArea + areaImprovementToSwitch.getDoubleValue();

      StepConstraintRegion activePlanarRegion = planarRegionToConstrainTo;

      for (int regionIndex = 0; regionIndex < stepConstraintRegions.size(); regionIndex++)
      {
         StepConstraintRegion constraintRegion = stepConstraintRegions.get(regionIndex);
         if (constraintRegion == activePlanarRegion)
            continue;

         double intersectionArea = findIntersectionAreaWithCaptureRegion(captureRegion, reachabilityRegion, constraintRegion);

         if (intersectionArea > maxArea)
         {
            maxArea = intersectionArea;
            activePlanarRegion = constraintRegion;
         }
      }

      return activePlanarRegion;
   }

   private double findIntersectionAreaWithCaptureRegion(FrameConvexPolygon2DReadOnly captureRegionInControlPlane, ConvexPolygon2DReadOnly reachabilityRegion, StepConstraintRegion constraintRegion)
   {
      icpControlPlane.projectVerticesOntoControlPlane(constraintRegion.getConcaveHull(), constraintRegion.getTransformToWorld(), convexHullConstraintInControlPlane);
      if (reachabilityRegion != null)
         convexPolygonTools.computeIntersectionOfPolygons(convexHullConstraintInControlPlane, reachabilityRegion, possibleArea);
      else
         possibleArea.set(convexHullConstraintInControlPlane);

      double intersectionArea = convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegionInControlPlane, possibleArea);

      for (ConcavePolygon2DReadOnly convexPolygon : constraintRegion.getHolesInConstraintRegion())
      {
         icpControlPlane.projectVerticesOntoControlPlane(convexPolygon, constraintRegion.getTransformToWorld(), convexHullConstraintInControlPlane);
         if (reachabilityRegion != null)
            convexPolygonTools.computeIntersectionOfPolygons(convexHullConstraintInControlPlane, reachabilityRegion, possibleArea);
         else
            possibleArea.set(convexHullConstraintInControlPlane);

         intersectionArea -= convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegionInControlPlane, possibleArea);
      }

      return intersectionArea;
   }

}
