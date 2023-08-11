package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import java.awt.*;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlPlane;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class CapturabilityBasedPlanarRegionDecider implements SCS2YoGraphicHolder
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double defaultMinimumCaptureAreaForSearch = 0.015;
   private static final double defaultCaptureAreaImprovementToSwitch = 0.015;
   private static final double defaultWeightToCurrentCaptureArea = 1.1;
   private static final double defaultDistanceInsideToRemoveRegionConstraint = 0.75;

   private final YoBoolean currentStepConstraintIsStillValid;
   private final YoBoolean switchPlanarRegionConstraintsAutomatically;
   private final YoDouble minimumCaptureAreaThreshold;
   private final YoDouble captureAreaImprovementToSwitch;
   private final YoDouble weightToCurrentCaptureArea;

   private final YoBoolean constraintRegionChanged;
   private final YoBoolean hasConstraintRegion;
   private final YoInteger constraintRegionId;
   private final YoInteger numberOfConstraintRegions;
   private final YoBoolean stepIsFarEnoughInsideToIgnoreConstraint;
   private final YoDouble captureAreaWithNextBestRegion;
   private final YoDouble captureAreaWithCurrentRegion;
   private final YoDouble minimumCaptureAreaToSwitch;
   private final YoDouble distanceInsideToRemoveRegionConstraint;

   private final YoBoolean isBoundingBoxVisualized;

   /**************** State variables as inputs ********************/
   private final FrameConvexPolygon2D captureRegion = new FrameConvexPolygon2D();
   private final List<StepConstraintRegion> stepConstraintRegions = new ArrayList<>();

   /**************** State variables as outputs ********************/
   /** This is the region that the step is constrained to */
   private StepConstraintRegion planarRegionToConstrainTo = null;
   /** This is the convex hull of the region that the step is constrained to */
   private final ConvexPolygon2D environmentConvexHull = new ConvexPolygon2D();
   private final YoFrameConvexPolygon2D yoEnvironmentConvexHull;
   /** This is the intersection of the capture region, the reachable region, and the region that the step is constrained to **/
   private final ConvexPolygon2D stepConstraintPolygon = new ConvexPolygon2D();

   /**************** Computation variables ********************/
   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   private final ConvexPolygon2D candidateStepConstraintPolygon = new ConvexPolygon2D();
   private final ConvexPolygon2D candidateEnvironmentConvexHull = new ConvexPolygon2D();

   private final ConvexPolygon2D tempHoleConvexHull = new ConvexPolygon2D();
   private final ConvexPolygon2D tempEnvironmentConvexHull = new ConvexPolygon2D();
   private final ConvexPolygon2D tempReachableEnvironment = new ConvexPolygon2D();
   private final ConvexPolygon2D tempReachableHoleConvexHull = new ConvexPolygon2D();
   private final ConvexPolygon2D tempStepConstraintPolygon = new ConvexPolygon2D();

   private final FramePoint3D projectedFoothold = new FramePoint3D();
   private final FramePoint2D tempPoint = new FramePoint2D();

   public CapturabilityBasedPlanarRegionDecider(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      currentStepConstraintIsStillValid = new YoBoolean("currentStepConstraintIsStillValid", registry);
      constraintRegionChanged = new YoBoolean("constraintRegionChanged", registry);
      hasConstraintRegion = new YoBoolean("hasConstraintRegion", registry);
      constraintRegionId = new YoInteger("constraintRegionId", registry);
      numberOfConstraintRegions = new YoInteger("numberOfConstraintRegions", registry);
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
      switchPlanarRegionConstraintsAutomatically.set(true);

      isBoundingBoxVisualized = new YoBoolean("isBoundingBoxVisualized", registry);

      yoEnvironmentConvexHull = new YoFrameConvexPolygon2D("environmentConvexHull", "", worldFrame, 40, registry);
      if (yoGraphicsListRegistry != null)
      {
         YoArtifactPolygon activePlanarRegionViz = new YoArtifactPolygon("Environmental Convex Hull Constraint", yoEnvironmentConvexHull, Color.RED, false);

         yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), activePlanarRegionViz);
      }
   }

   public void setCaptureRegion(FrameConvexPolygon2DReadOnly captureRegion)
   {
      this.captureRegion.setIncludingFrame(captureRegion);
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
      stepConstraintPolygon.clearAndUpdate();

      constraintRegionId.set(-1);
      hasConstraintRegion.set(false);
      numberOfConstraintRegions.set(-1);
   }

   public void setConstraintRegions(List<StepConstraintRegion> constraintRegions)
   {
      reset();
      for (int i = 0; i < constraintRegions.size(); i++)
         stepConstraintRegions.add(constraintRegions.get(i));
      numberOfConstraintRegions.set(stepConstraintRegions.size());
   }

   public void setConstraintRegionChanged(boolean constraintRegionChanged)
   {
      this.constraintRegionChanged.set(constraintRegionChanged);
   }

   public boolean constraintRegionChanged()
   {
      return constraintRegionChanged.getBooleanValue();
   }

   public List<StepConstraintRegion> getStepConstraintRegions()
   {
      return stepConstraintRegions;
   }

   public StepConstraintRegion getConstraintRegion()
   {
      return planarRegionToConstrainTo;
   }

   public ConvexPolygon2DReadOnly getStepConstraintPolygon()
   {
      return stepConstraintPolygon;
   }

   public ConvexPolygon2DReadOnly getEnvironmentConvexHull()
   {
      return environmentConvexHull;
   }

   public boolean isStepFarEnoughInsideToIgnoreConstraint()
   {
      return stepIsFarEnoughInsideToIgnoreConstraint.getBooleanValue();
   }

   public void updatePlanarRegionConstraintForStep(FramePose3DReadOnly footstepPose, ConvexPolygon2DReadOnly reachabilityRegion)
   {
      constraintRegionChanged.set(false);

      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      // if we don't have any guess, just snap the foot vertically down onto the highest planar region under it.
      if (planarRegionToConstrainTo == null)
      {
         planarRegionToConstrainTo = findPlanarRegionUnderFoothold(footstepPose.getPosition(), projectedFoothold);

         // We now have a region, so the constraint region changed.
         if (planarRegionToConstrainTo != null)
            constraintRegionChanged.set(true);
      }

      // Update the step constraint polygon and environment convex hull
      if (planarRegionToConstrainTo != null)
      {
         computeEnvironmentConvexHull(planarRegionToConstrainTo, environmentConvexHull);
         computeStepConstraintPolygon(environmentConvexHull, reachabilityRegion, captureRegion, stepConstraintPolygon);
      }
      else
      {
         environmentConvexHull.clearAndUpdate();
         stepConstraintPolygon.clearAndUpdate();
      }

      // if we can switch, and there's a better region to be using, switch
      if (switchPlanarRegionConstraintsAutomatically.getBooleanValue() && !checkIfCurrentConstraintRegionIsStillValid(planarRegionToConstrainTo, stepConstraintPolygon))
      {
         StepConstraintRegion betterRegion = findBestPlanarRegionToStepTo(captureRegion, reachabilityRegion, candidateEnvironmentConvexHull, candidateStepConstraintPolygon);
         if (betterRegion != null && betterRegion != planarRegionToConstrainTo)
         {
            planarRegionToConstrainTo = betterRegion;
            environmentConvexHull.set(candidateEnvironmentConvexHull);
            stepConstraintPolygon.set(candidateStepConstraintPolygon);
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
      updateConstraintRegionVisualizer();
   }

   /**
    * This computes the convex hull of the step constraint region in the world frame.
    */
   private void computeEnvironmentConvexHull(StepConstraintRegion constraintRegion, ConvexPolygon2DBasics environmentConvexHullToPack)
   {
      environmentConvexHullToPack.set(constraintRegion.getConvexHullInConstraintRegion());
      environmentConvexHullToPack.applyTransform(constraintRegion.getTransformToWorld(), false);
      environmentConvexHullToPack.update();
   }

   /**
    * This computes the intersection between the environment, the reachable region, and the capture region.
    */
   private void computeStepConstraintPolygon(ConvexPolygon2DReadOnly environmentConvexHull,
                                             ConvexPolygon2DReadOnly reachabilityRegion,
                                             ConvexPolygon2DReadOnly captureRegion,
                                             ConvexPolygon2DBasics stepConstraintPolygonToPack)
   {
      if (reachabilityRegion != null)
         convexPolygonTools.computeIntersectionOfPolygons(environmentConvexHull, reachabilityRegion, tempReachableEnvironment);
      else
         tempReachableEnvironment.set(environmentConvexHull);

      convexPolygonTools.computeIntersectionOfPolygons(captureRegion, tempReachableEnvironment, stepConstraintPolygonToPack);
   }

   private StepConstraintRegion findPlanarRegionUnderFoothold(FramePoint3DReadOnly foothold, FramePoint3DBasics projectedFootholdToPack)
   {
      return PlanarRegionTools.projectPointToPlanesVertically(foothold, stepConstraintRegions, projectedFootholdToPack, null);
   }

   /**
    * Checks to see if the current constraint region is still valid for using. This is determined by making sure that the intersection of the environmental
    * constraint, the reachable area, and the capture region is above some minimum threshold.
    */
   private boolean checkIfCurrentConstraintRegionIsStillValid(StepConstraintRegion planarRegionToConstrainTo, ConvexPolygon2DBasics stepConstraintPolygon)
   {
      if (planarRegionToConstrainTo == null)
      {
         currentStepConstraintIsStillValid.set(false);
      }
      else
      {
         // if the current reachable portion of the environmental constraint has a large enough intersecting area with the capture region, it is still valid.
         captureAreaWithCurrentRegion.set(stepConstraintPolygon.getArea());
         currentStepConstraintIsStillValid.set(captureAreaWithCurrentRegion.getDoubleValue() > minimumCaptureAreaThreshold.getDoubleValue());
      }
      return currentStepConstraintIsStillValid.getBooleanValue();
   }

   private StepConstraintRegion findBestPlanarRegionToStepTo(FrameConvexPolygon2DReadOnly captureRegion,
                                                             ConvexPolygon2DReadOnly reachabilityRegion,
                                                             ConvexPolygon2DBasics environmentConvexHullToPack,
                                                             ConvexPolygon2DBasics stepConstraintPolygonToPack)
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
         double intersectionArea = findIntersectionAreaWithCaptureRegion(captureRegion, reachabilityRegion, constraintRegion, tempEnvironmentConvexHull, tempStepConstraintPolygon);

         // if this new candidate has more intersecting area, switch the regions
         if (intersectionArea > currentAreaOfBestIntersection)
         {
            captureAreaWithNextBestRegion.set(intersectionArea);
            currentAreaOfBestIntersection = intersectionArea;
            activePlanarRegion = constraintRegion;
            environmentConvexHullToPack.set(tempEnvironmentConvexHull);
            stepConstraintPolygonToPack.set(tempStepConstraintPolygon);
         }
      }

      return activePlanarRegion;
   }

   /**
    * This finds the intersection area between the capture region, the environment, and the reachable region. It then removes the area of any holes.
    */
   private double findIntersectionAreaWithCaptureRegion(FrameConvexPolygon2DReadOnly captureRegionInControlPlane,
                                                        ConvexPolygon2DReadOnly reachabilityRegion,
                                                        StepConstraintRegion constraintRegion,
                                                        ConvexPolygon2DBasics environmentHullToPack,
                                                        ConvexPolygon2DBasics intersectingRegionToPack)
   {
      computeEnvironmentConvexHull(constraintRegion, environmentHullToPack);
      computeStepConstraintPolygon(environmentHullToPack, reachabilityRegion, captureRegionInControlPlane, intersectingRegionToPack);

      double intersectionArea = intersectingRegionToPack.getArea();

      // remove the holes from the intersection area
      for (int i = 0; i < constraintRegion.getHolesInConstraintRegion().size(); i++)
      {
         ConcavePolygon2DReadOnly hole = constraintRegion.getHoleInConstraintRegion(i);

         tempHoleConvexHull.set(hole);
         tempHoleConvexHull.applyTransform(constraintRegion.getTransformToWorld());
         tempHoleConvexHull.update();

         computeStepConstraintPolygon(tempHoleConvexHull, reachabilityRegion, captureRegionInControlPlane, tempReachableHoleConvexHull);
         intersectionArea -= tempReachableHoleConvexHull.getArea();
      }

      return intersectionArea;
   }

   private void checkDistanceInsideOfRegion(FramePoint3DReadOnly stepPosition)
   {
      tempPoint.set(stepPosition);
      stepIsFarEnoughInsideToIgnoreConstraint.set(stepConstraintPolygon.signedDistance(tempPoint) < -distanceInsideToRemoveRegionConstraint.getDoubleValue());
   }

   private void updateConstraintRegionVisualizer()
   {
      if (environmentConvexHull.getNumberOfVertices() <= yoEnvironmentConvexHull.getMaxNumberOfVertices())
      {
         yoEnvironmentConvexHull.set(environmentConvexHull);
         isBoundingBoxVisualized.set(false);
      }
      else
      {
         // we don't have enough yo variables to view the convex hull, so instead let's view it's bounding box
         yoEnvironmentConvexHull.clear();
         BoundingBox3DReadOnly boundingBox = planarRegionToConstrainTo.getBoundingBox3dInWorld();
         yoEnvironmentConvexHull.addVertex(boundingBox.getMaxX(), boundingBox.getMaxY());
         yoEnvironmentConvexHull.addVertex(boundingBox.getMaxX(), boundingBox.getMinY());
         yoEnvironmentConvexHull.addVertex(boundingBox.getMinY(), boundingBox.getMinY());
         yoEnvironmentConvexHull.addVertex(boundingBox.getMinY(), boundingBox.getMaxY());
         yoEnvironmentConvexHull.update();
         isBoundingBoxVisualized.set(true);
      }
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygon2D("Environmental Convex Hull Constraint", yoEnvironmentConvexHull, ColorDefinitions.Red()));
      return group;
   }
}
