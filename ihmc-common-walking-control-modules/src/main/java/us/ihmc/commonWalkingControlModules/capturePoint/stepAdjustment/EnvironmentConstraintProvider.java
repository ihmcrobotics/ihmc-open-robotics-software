package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.captureRegion.OneStepCaptureRegionCalculator;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class EnvironmentConstraintProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double maxNormalAngleFromVertical = 0.3;
   private static final double minimumAreaToConsider = 0.01;

   private static final double minimumIntersectionForSearch = 0.01;

   private final DoubleProvider maxAngleForSteppable;
   private final DoubleProvider minimumAreaForSteppable;

   private final RecyclingArrayList<PlanarRegion> allPlanarRegionsThatAreSteppable = new RecyclingArrayList<>(PlanarRegion.class);
   private final YoInteger numberOfPlanarListsToConsider;

   private final YoBoolean switchPlanarRegionConstraintsAutomatically;

   private final OneStepCaptureRegionCalculator captureRegionCalculator;
   private final ConvexPolygonTools convexPolygonTools = new ConvexPolygonTools();

   private final YoBoolean constraintRegionChanged;
   private final YoBoolean usePlanarRegionConstraints;

   private final YoFrameConvexPolygon2D convexHullConstraint;

   private PlanarRegion planarRegionToConstrainTo = null;

   public EnvironmentConstraintProvider(ICPOptimizationParameters optimizationParameters,
                                        OneStepCaptureRegionCalculator captureRegionCalculator,
                                        String yoNamePrefix,
                                        YoVariableRegistry registry)
   {
      this.captureRegionCalculator = captureRegionCalculator;

      maxAngleForSteppable = new DoubleParameter(yoNamePrefix + "MaxAngleForSteppable", registry, maxNormalAngleFromVertical);
      minimumAreaForSteppable = new DoubleParameter(yoNamePrefix + "MinimumAreaForSteppable", registry, minimumAreaToConsider);

      numberOfPlanarListsToConsider = new YoInteger(yoNamePrefix + "NumberOfPlanarListsToConsider", registry);
      constraintRegionChanged = new YoBoolean(yoNamePrefix + "ConstraintRegionChanged", registry);
      usePlanarRegionConstraints = new YoBoolean(yoNamePrefix + "UsePlanarRegionConstraints", registry);

      switchPlanarRegionConstraintsAutomatically = new YoBoolean(yoNamePrefix + "SwitchPlanarRegionConstraintsAutomatically", registry);
      switchPlanarRegionConstraintsAutomatically.set(optimizationParameters.switchPlanarRegionConstraintsAutomatically());

      convexHullConstraint = new YoFrameConvexPolygon2D(yoNamePrefix + "ConvexHullConstraint", "", worldFrame, 12, registry);
   }

   private final Vector3D planeNormal = new Vector3D();
   private final Vector3D verticalAxis = new Vector3D(0.0, 0.0, 1.0);

   public void setPlanarRegions(List<PlanarRegion> planarRegions)
   {
      allPlanarRegionsThatAreSteppable.clear();
      numberOfPlanarListsToConsider.set(0);

      for (int i = 0; i < planarRegions.size(); i++)
      {
         PlanarRegion planarRegion = planarRegions.get(i);

         if (isRegionValidForStepping(planarRegion))
         {
            allPlanarRegionsThatAreSteppable.add().set(planarRegions.get(i));
            numberOfPlanarListsToConsider.increment();
         }
      }
   }

   public void reset()
   {
      planarRegionToConstrainTo = null;
      convexHullConstraint.clear();
   }

   public FrameConvexPolygon2DReadOnly updatePlanarRegionConstraintForStep(FramePose3DReadOnly footPosition)
   {
      constraintRegionChanged.set(false);

      if (!usePlanarRegionConstraints.getBooleanValue())
         return null;

      boolean planarRegionNeedsUpdating = true;

      if (planarRegionToConstrainTo == null)
      {
         planarRegionToConstrainTo = findPlanarRegionUnderFoothold(footPosition);
         convexHullConstraint.set(planarRegionToConstrainTo.getConvexHull());
         convexHullConstraint.applyTransform(planarRegionToConstrainTo.getTransformToWorld(), false);
      }

      if (switchPlanarRegionConstraintsAutomatically.getBooleanValue())
      {
         if (planarRegionToConstrainTo != null)
            planarRegionNeedsUpdating = checkIfCurrentPlanarRegionIsValid();

         if (planarRegionNeedsUpdating)
         {
            PlanarRegion betterRegion = findBestPlanarRegionToStepTo();
            if (betterRegion != null)
            {
               planarRegionToConstrainTo = betterRegion;
               constraintRegionChanged.set(true);
               convexHullConstraint.set(planarRegionToConstrainTo.getConvexHull());
               convexHullConstraint.applyTransform(planarRegionToConstrainTo.getTransformToWorld(), false);
            }
         }
      }

      return planarRegionToConstrainTo != null ? convexHullConstraint : null;
   }

   private boolean isRegionValidForStepping(PlanarRegion planarRegion)
   {
      planarRegion.getNormal(planeNormal);

      double angle = planeNormal.angle(verticalAxis);

      if (angle > maxAngleForSteppable.getValue())
         return false;

      // TODO switch to the concave hull
      return planarRegion.getConvexHull().getArea() > minimumAreaForSteppable.getValue();
   }

   private PlanarRegion findPlanarRegionUnderFoothold(FramePose3DReadOnly foothold)
   {
      PlanarRegion highestRegionUnderFoot = null;
      double highestPoint = Double.NEGATIVE_INFINITY;
      for (int regionIndex = 0; regionIndex < allPlanarRegionsThatAreSteppable.size(); regionIndex++)
      {
         PlanarRegion planarRegion = allPlanarRegionsThatAreSteppable.get(regionIndex);

         if (!planarRegion.isPointInWorld2DInside(foothold.getPosition()))
            continue;

         double height = planarRegion.getPlaneZGivenXY(foothold.getPosition().getX(), foothold.getPosition().getY());
         if (height >= highestPoint)
         {
            highestPoint = height;
            highestRegionUnderFoot = planarRegion;
         }
      }

      return highestRegionUnderFoot;
   }

   private boolean checkIfCurrentPlanarRegionIsValid()
   {
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();
      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      double intersectionArea = convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegion, convexHullConstraint);

      return intersectionArea > minimumIntersectionForSearch;
   }

   private final ConvexPolygon2D convexHullInWorld = new ConvexPolygon2D();

   private PlanarRegion findBestPlanarRegionToStepTo()
   {
      FrameConvexPolygon2D captureRegion = captureRegionCalculator.getCaptureRegion();
      captureRegion.changeFrameAndProjectToXYPlane(worldFrame);

      double maxArea = 0.0;
      PlanarRegion activePlanarRegion = null;

      for (int regionIndex = 0; regionIndex < allPlanarRegionsThatAreSteppable.size(); regionIndex++)
      {
         PlanarRegion planarRegion = allPlanarRegionsThatAreSteppable.get(regionIndex);
         convexHullInWorld.set(planarRegion.getConvexHull());
         convexHullInWorld.applyTransform(planarRegion.getTransformToWorld(), false);

         double intersectionArea = convexPolygonTools.computeIntersectionAreaOfPolygons(captureRegion, convexHullInWorld);

         if (intersectionArea > maxArea)
         {
            maxArea = intersectionArea;
            activePlanarRegion = planarRegion;
         }
      }

      return activePlanarRegion;
   }
}
