package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.providers.BooleanProvider;
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

   private final ConvexPolygon2D convexHullConstraint = new ConvexPolygon2D();
   private final ConvexPolygon2D reachableEnvironmentalArea = new ConvexPolygon2D();

   private final FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D();
   private final YoFrameConvexPolygon2D yoConvexHullConstraint;

   private final BooleanProvider useControlPlane;
   private final ICPControlPlane icpControlPlane;

   private StepConstraintRegion planarRegionToConstrainTo = null;

   public CapturabilityBasedPlanarRegionDecider(ReferenceFrame centerOfMassFrame,
                                                double gravityZ,
                                                BooleanProvider useControlPlane,
                                                YoRegistry registry,
                                                YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(new ICPControlPlane(centerOfMassFrame, gravityZ, registry), useControlPlane, registry, yoGraphicsListRegistry);
   }

   public CapturabilityBasedPlanarRegionDecider(ICPControlPlane icpControlPlane,
                                                BooleanProvider useControlPlane,
                                                YoRegistry registry,
                                                YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.useControlPlane = useControlPlane;
      this.icpControlPlane = icpControlPlane;

      constraintRegionChanged = new YoBoolean("constraintRegionChanged", registry);
      intersectionAreaWithCurrentRegion = new YoDouble("intersectionAreaWithCurrentRegion", registry);

      minimumIntersectionForSearch = new YoDouble("minimumIntersectionForSearch", registry);
      areaImprovementToSwitch = new YoDouble("areaImprovementToSwitch", registry);
      inflationToCurrentArea = new YoDouble("inflationToCurrentArea", registry);
      minimumIntersectionForSearch.set(defaultMinimumIntersectionForSearch);
      areaImprovementToSwitch.set(defaultAreaImprovementToSwitch);
      inflationToCurrentArea.set(defaultInflationToCurrentArea);

      switchPlanarRegionConstraintsAutomatically = new YoBoolean("switchPlanarRegionConstraintsAutomatically", registry);
      switchPlanarRegionConstraintsAutomatically.set(true);

      yoConvexHullConstraint = new YoFrameConvexPolygon2D("convexHullConstraint", "", worldFrame, 12, registry);

      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon activePlanarRegionViz = new YoArtifactPolygon("ConvexHullConstraint", yoConvexHullConstraint, Color.RED, false);
         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionViz);
      }
   }

   public void setConstraintRegions(List<StepConstraintRegion> constraintRegions)
   {
      reset();
      for (int i = 0; i < constraintRegions.size(); i++)
         stepConstraintRegions.add(constraintRegions.get(i));
   }

   public void setCaptureRegion(FrameConvexPolygon2DReadOnly captureRegion)
   {
      this.captureRegion.setIncludingFrame(captureRegion);
   }

   public void setOmega0(double omega)
   {
      if (useControlPlane.getValue() && icpControlPlane != null)
         icpControlPlane.setOmega0(omega);
   }

   public void setSwitchPlanarRegionConstraintsAutomatically(boolean switchAutomatically)
   {
      switchPlanarRegionConstraintsAutomatically.set(switchAutomatically);
   }

   public void reset()
   {
      planarRegionToConstrainTo = null;
      stepConstraintRegions.clear();
      intersectionAreaWithCurrentRegion.set(0.0);
      yoConvexHullConstraint.clear();
   }

   private void computeProjectedConvexHull(StepConstraintRegion constraintRegion)
   {
      yoConvexHullConstraint.set(constraintRegion.getConvexHullInConstraintRegion());
      yoConvexHullConstraint.applyTransform(constraintRegion.getTransformToWorld(), false);

      if (useControlPlane.getValue() && icpControlPlane != null)
      {
         icpControlPlane.projectVerticesOntoControlPlane(constraintRegion.getConvexHullInConstraintRegion(),
                                                         constraintRegion.getTransformToWorld(),
                                                         convexHullConstraint);
      }
      else
      {
         convexHullConstraint.set(yoConvexHullConstraint);
      }
   }

   public StepConstraintRegion updatePlanarRegionConstraintForStep(FramePose3DReadOnly footstepPose, ConvexPolygon2DReadOnly reachableRegion)
   {
      constraintRegionChanged.set(false);

      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      // if we don't have any guess, just snap the foot vertically down onto the highest planar region under it.
      if (planarRegionToConstrainTo == null)
      {
         planarRegionToConstrainTo = findPlanarRegionUnderFoothold(footstepPose.getPosition());

         // update the convex hull of the constraint
         if (planarRegionToConstrainTo != null)
         {
            computeProjectedConvexHull(planarRegionToConstrainTo);
            constraintRegionChanged.set(true);
         }
      }

      // if we can switch, and there's a better region to be using, switch
      if (switchPlanarRegionConstraintsAutomatically.getBooleanValue() && !checkIfCurrentConstraintRegionIsValid(captureRegion, reachableRegion))
      {
         StepConstraintRegion betterRegion = findBestPlanarRegionToStepTo(captureRegion, reachableRegion);
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

   private final StepConstraintRegion highestRegionUnderFoot = new StepConstraintRegion();
   private final FramePoint3D projectedFoothold = new FramePoint3D();

   private StepConstraintRegion findPlanarRegionUnderFoothold(FramePoint3DReadOnly foothold)
   {
      if (!PlanarRegionTools.projectPointToPlanesVertically(foothold, stepConstraintRegions, projectedFoothold, highestRegionUnderFoot))
         return null;

      return highestRegionUnderFoot;
   }

   /**
    * Checks to see if the current constraint region is still valid for using. This is determined by making sure that the intersection of the environmental
    * constraint, the reachable area, and the capture region is above some minimum threshold.
    */
   private boolean checkIfCurrentConstraintRegionIsValid(FrameConvexPolygon2DReadOnly captureRegion, ConvexPolygon2DReadOnly reachabilityRegion)
   {
      if (planarRegionToConstrainTo == null)
         return false;

      computeProjectedConvexHull(planarRegionToConstrainTo);
      if (reachabilityRegion != null)
         convexPolygonTools.computeIntersectionOfPolygons(convexHullConstraint, reachabilityRegion, reachableEnvironmentalArea);
      else
         reachableEnvironmentalArea.set(convexHullConstraint);

      // if the current reachable portion of the environmental constraint has a large enough intersecting area with the capture region, it is still valid.
      intersectionAreaWithCurrentRegion.set(convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegion, reachableEnvironmentalArea));

      return intersectionAreaWithCurrentRegion.getDoubleValue() > minimumIntersectionForSearch.getDoubleValue();
   }

   private StepConstraintRegion findBestPlanarRegionToStepTo(FrameConvexPolygon2DReadOnly captureRegion, ConvexPolygon2DReadOnly reachabilityRegion)
   {
      double currentAreaOfBestIntersection = intersectionAreaWithCurrentRegion.getDoubleValue();
      if (currentAreaOfBestIntersection > 0.0) // if we do have some intersection, we want to weight more heavily continued use of that. We increase the current area weight, as well as require a larger area to switch
         currentAreaOfBestIntersection = (1.0 + inflationToCurrentArea.getDoubleValue()) * currentAreaOfBestIntersection + areaImprovementToSwitch.getDoubleValue();

      StepConstraintRegion activePlanarRegion = planarRegionToConstrainTo;

      for (int regionIndex = 0; regionIndex < stepConstraintRegions.size(); regionIndex++)
      {
         StepConstraintRegion constraintRegion = stepConstraintRegions.get(regionIndex);
         // this is the current constrained planar region, so move on
         if (constraintRegion == activePlanarRegion)
            continue;

         // get the area of intersection with this new candidate
         double intersectionArea = findIntersectionAreaWithCaptureRegion(captureRegion, reachabilityRegion, constraintRegion);

         // if this new candidate has more intersecting area, switch the regions
         if (intersectionArea > currentAreaOfBestIntersection)
         {
            currentAreaOfBestIntersection = intersectionArea;
            activePlanarRegion = constraintRegion;
         }
      }

      return activePlanarRegion;
   }

   private double findIntersectionAreaWithCaptureRegion(FrameConvexPolygon2DReadOnly captureRegionInControlPlane,
                                                        ConvexPolygon2DReadOnly reachabilityRegion,
                                                        StepConstraintRegion constraintRegion)
   {
      if (useControlPlane.getValue() && icpControlPlane != null)
      {
         icpControlPlane.projectVerticesOntoControlPlane(constraintRegion.getConcaveHull(), constraintRegion.getTransformToWorld(), convexHullConstraint);
      }
      else
      {
         convexHullConstraint.set(constraintRegion.getConcaveHull());
         convexHullConstraint.applyTransform(constraintRegion.getTransformToWorld(), false);
      }

      if (reachabilityRegion != null)
         convexPolygonTools.computeIntersectionOfPolygons(convexHullConstraint, reachabilityRegion, reachableEnvironmentalArea);
      else
         reachableEnvironmentalArea.set(convexHullConstraint);

      double intersectionArea = convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegionInControlPlane, reachableEnvironmentalArea);

      for (ConcavePolygon2DReadOnly convexPolygon : constraintRegion.getHolesInConstraintRegion())
      {
         if (useControlPlane.getValue() && icpControlPlane != null)
         {
            icpControlPlane.projectVerticesOntoControlPlane(convexPolygon, constraintRegion.getTransformToWorld(), convexHullConstraint);
         }
         else
         {
            convexHullConstraint.set(constraintRegion.getConcaveHull());
            convexHullConstraint.applyTransform(constraintRegion.getTransformToWorld());
         }

         if (reachabilityRegion != null)
            convexPolygonTools.computeIntersectionOfPolygons(convexHullConstraint, reachabilityRegion, reachableEnvironmentalArea);
         else
            reachableEnvironmentalArea.set(convexHullConstraint);

         intersectionArea -= convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegionInControlPlane, reachableEnvironmentalArea);
      }

      return intersectionArea;
   }

}
