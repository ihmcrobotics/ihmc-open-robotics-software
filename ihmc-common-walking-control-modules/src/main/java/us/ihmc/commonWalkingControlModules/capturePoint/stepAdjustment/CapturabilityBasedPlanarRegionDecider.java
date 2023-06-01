package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameConvexPolygon2DBasics;
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
import us.ihmc.yoVariables.variable.YoInteger;

public class CapturabilityBasedPlanarRegionDecider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double defaultMinimumCaptureAreaForSearch = 0.015;
   private static final double defaultCaptureAreaImprovementToSwitch = 0.015;
   private static final double defaultWeightToCurrentCaptureArea = 1.1;
   private static final double defaultDistanceInsideToRemoveRegionConstraint = 0.75;

   private final List<StepConstraintRegion> stepConstraintRegions = new ArrayList<>();

   private final YoBoolean currentStepConstraintIsStillValid;
   private final YoBoolean switchPlanarRegionConstraintsAutomatically;
   private final YoDouble minimumCaptureAreaThreshold;
   private final YoDouble captureAreaImprovementToSwitch;
   private final YoDouble weightToCurrentCaptureArea;

   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   private final YoBoolean constraintRegionChanged;
   private final YoBoolean hasConstraintRegion;
   private final YoInteger constraintRegionId;
   private final YoBoolean stepIsFarEnoughInsideToIgnoreConstraint;
   private final YoDouble captureAreaWithNextBestRegion;
   private final YoDouble captureAreaWithCurrentRegion;
   private final YoDouble minimumCaptureAreaToSwitch;
   private final YoDouble distanceInsideToRemoveRegionConstraint;

   private final ConvexPolygon2D convexHullConstraint = new ConvexPolygon2D();
   private final ConvexPolygon2D reachableEnvironmentalArea = new ConvexPolygon2D();
   private final ConvexPolygon2D candidateConvexHullConstraint = new ConvexPolygon2D();

   private final FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D();

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

      currentStepConstraintIsStillValid = new YoBoolean("currentStepConstraintIsStillValid", registry);
      constraintRegionChanged = new YoBoolean("constraintRegionChanged", registry);
      hasConstraintRegion = new YoBoolean("hasConstraintRegion", registry);
      constraintRegionId = new YoInteger("constraintRegionId", registry);
      stepIsFarEnoughInsideToIgnoreConstraint = new YoBoolean("stepIsFarEnoughInsideToIgnoreConstraint", registry);
      captureAreaWithCurrentRegion = new YoDouble("captureAreaWithCurrentRegion", registry);
      captureAreaWithNextBestRegion = new YoDouble("captureAreaWithNextBestRegion", registry);
      minimumCaptureAreaToSwitch = new YoDouble("minimumCaptureAreaToSwitch", registry);

      minimumCaptureAreaThreshold = new YoDouble("minimumCaptureAreaThreshold", registry);
      captureAreaImprovementToSwitch = new YoDouble("captureAreaImprovementToSwitch", registry);
      weightToCurrentCaptureArea = new YoDouble("weightToCurrentCaptureArea", registry);
      minimumCaptureAreaThreshold.set(defaultMinimumCaptureAreaForSearch);
      captureAreaImprovementToSwitch.set(defaultCaptureAreaImprovementToSwitch);
      weightToCurrentCaptureArea.set(defaultWeightToCurrentCaptureArea);

      distanceInsideToRemoveRegionConstraint = new YoDouble("distanceInsideToRemoveRegionConstraint", registry);
      distanceInsideToRemoveRegionConstraint.set(defaultDistanceInsideToRemoveRegionConstraint);

      switchPlanarRegionConstraintsAutomatically = new YoBoolean("switchPlanarRegionConstraintsAutomatically", registry);
      switchPlanarRegionConstraintsAutomatically.set(false);
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
      captureAreaWithCurrentRegion.set(0.0);
      convexHullConstraint.clearAndUpdate();
   }

   private void computeProjectedConvexHull(StepConstraintRegion constraintRegion)
   {
      if (useControlPlane.getValue() && icpControlPlane != null)
      {
         icpControlPlane.projectVerticesOntoControlPlane(constraintRegion.getConvexHullInConstraintRegion(),
                                                         constraintRegion.getTransformToWorld(),
                                                         convexHullConstraint);
      }
      else
      {
         convexHullConstraint.set(constraintRegion.getConvexHullInConstraintRegion());
         convexHullConstraint.applyTransform(constraintRegion.getTransformToWorld(), false);
         convexHullConstraint.update();
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
         else
         {
            convexHullConstraint.clearAndUpdate();
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
      else
      {
         minimumCaptureAreaToSwitch.setToNaN();
         captureAreaWithNextBestRegion.setToNaN();
      }

      hasConstraintRegion.set(planarRegionToConstrainTo != null);
      constraintRegionId.set(planarRegionToConstrainTo == null ? -1 : planarRegionToConstrainTo.getRegionId());
      checkDistanceInsideOfRegion(footstepPose.getPosition());

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

   public boolean isStepFarEnoughInsideToIgnoreConstraint()
   {
      return stepIsFarEnoughInsideToIgnoreConstraint.getBooleanValue();
   }

   private final FramePoint3D projectedFoothold = new FramePoint3D();
   private final FramePoint2D tempPoint = new FramePoint2D();

   private StepConstraintRegion findPlanarRegionUnderFoothold(FramePoint3DReadOnly foothold)
   {
      StepConstraintRegion highestRegionUnderFoot = PlanarRegionTools.projectPointToPlanesVertically(foothold, stepConstraintRegions, projectedFoothold, null);
      return highestRegionUnderFoot;
   }

   /**
    * Checks to see if the current constraint region is still valid for using. This is determined by making sure that the intersection of the environmental
    * constraint, the reachable area, and the capture region is above some minimum threshold.
    */
   private boolean checkIfCurrentConstraintRegionIsValid(FrameConvexPolygon2DReadOnly captureRegion, ConvexPolygon2DReadOnly reachabilityRegion)
   {
      if (planarRegionToConstrainTo == null)
      {
         currentStepConstraintIsStillValid.set(false);
      }
      else
      {

         computeProjectedConvexHull(planarRegionToConstrainTo);
         if (reachabilityRegion != null)
            convexPolygonTools.computeIntersectionOfPolygons(convexHullConstraint, reachabilityRegion, reachableEnvironmentalArea);
         else
            reachableEnvironmentalArea.set(convexHullConstraint);

         // if the current reachable portion of the environmental constraint has a large enough intersecting area with the capture region, it is still valid.
         captureAreaWithCurrentRegion.set(convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegion, reachableEnvironmentalArea));

         currentStepConstraintIsStillValid.set(captureAreaWithCurrentRegion.getDoubleValue() > minimumCaptureAreaThreshold.getDoubleValue());
      }
      return currentStepConstraintIsStillValid.getBooleanValue();
   }

   private StepConstraintRegion findBestPlanarRegionToStepTo(FrameConvexPolygon2DReadOnly captureRegion, ConvexPolygon2DReadOnly reachabilityRegion)
   {
      double currentAreaOfBestIntersection = captureAreaWithCurrentRegion.getDoubleValue();
      if (currentAreaOfBestIntersection > 0.0) // if we do have some intersection, we want to weight more heavily continued use of that. We increase the current area weight, as well as require a larger area to switch
         currentAreaOfBestIntersection = weightToCurrentCaptureArea.getDoubleValue() * currentAreaOfBestIntersection + captureAreaImprovementToSwitch.getDoubleValue();

      minimumCaptureAreaToSwitch.set(currentAreaOfBestIntersection);
      captureAreaWithNextBestRegion.setToNaN();
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
            captureAreaWithNextBestRegion.set(intersectionArea);
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
         icpControlPlane.projectVerticesOntoControlPlane(constraintRegion.getConcaveHull(), constraintRegion.getTransformToWorld(), candidateConvexHullConstraint);
      }
      else
      {
         candidateConvexHullConstraint.set(constraintRegion.getConcaveHull());
         candidateConvexHullConstraint.applyTransform(constraintRegion.getTransformToWorld(), false);
         candidateConvexHullConstraint.update();
      }

      if (reachabilityRegion != null)
         convexPolygonTools.computeIntersectionOfPolygons(candidateConvexHullConstraint, reachabilityRegion, reachableEnvironmentalArea);
      else
         reachableEnvironmentalArea.set(candidateConvexHullConstraint);

      double intersectionArea = convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegionInControlPlane, reachableEnvironmentalArea);

      for (int i = 0; i < constraintRegion.getHolesInConstraintRegion().size(); i++)
      {
         ConcavePolygon2DReadOnly convexPolygon = constraintRegion.getHoleInConstraintRegion(i);

         if (useControlPlane.getValue() && icpControlPlane != null)
         {
            icpControlPlane.projectVerticesOntoControlPlane(convexPolygon, constraintRegion.getTransformToWorld(), candidateConvexHullConstraint);
         }
         else
         {
            candidateConvexHullConstraint.set(constraintRegion.getConcaveHull());
            candidateConvexHullConstraint.applyTransform(constraintRegion.getTransformToWorld());
            candidateConvexHullConstraint.update();
         }

         if (reachabilityRegion != null)
            convexPolygonTools.computeIntersectionOfPolygons(candidateConvexHullConstraint, reachabilityRegion, reachableEnvironmentalArea);
         else
            reachableEnvironmentalArea.set(candidateConvexHullConstraint);

         intersectionArea -= convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegionInControlPlane, reachableEnvironmentalArea);
      }

      return intersectionArea;
   }

   private void checkDistanceInsideOfRegion(FramePoint3DReadOnly stepPosition)
   {
      tempPoint.set(stepPosition);
      stepIsFarEnoughInsideToIgnoreConstraint.set(convexHullConstraint.signedDistance(tempPoint) < distanceInsideToRemoveRegionConstraint.getDoubleValue());
   }
}
