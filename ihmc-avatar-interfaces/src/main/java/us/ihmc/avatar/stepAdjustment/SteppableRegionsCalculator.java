package us.ihmc.avatar.stepAdjustment;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ClusterType;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ExtrusionHull;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.robotics.RegionInWorldInterface;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;
import us.ihmc.robotics.geometry.concavePolygon2D.clippingAndMerging.PolygonClippingAndMerging;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;
import java.util.stream.Collectors;

public class SteppableRegionsCalculator
{
   private static final double POPPING_MULTILINE_POINTS_THRESHOLD = MathTools.square(0.10);

   private static final double maxNormalAngleFromVertical = 0.4;
   private static final double minimumAreaToConsider = 0.01;
   private static final double defaultCanDuckUnderHeight = 2.0;
   private static final double defaultCanEasilyStepOverHeight = 0.03;
   private static final double defaultOrthogonalAngle = Math.toRadians(75.0);
   private static final double defaultMinimumDistanceFromCliffBottoms = 0.1;

   private static final Vector3D verticalAxis = new Vector3D(0.0, 0.0, 1.0);

   private final YoDouble maxAngleForSteppable;
   private final YoDouble minimumAreaForSteppable;
   private final YoDouble maximumStepReach;

   private final YoDouble canDuckUnderHeight;
   private final YoDouble canEasilyStepOverHeight;

   private final YoDouble orthogonalAngle;
   private final YoDouble minimumDistanceFromCliffBottoms;

   private HashMap<RegionInWorldInterface, List<ConcavePolygon2DBasics>> obstacleExtrusions = new HashMap<>();
   private List<StepConstraintRegion> steppableRegions = new ArrayList<>();
   private List<PlanarRegion> allPlanarRegions = new ArrayList<>();
   private List<PlanarRegion> tooSmallRegions = new ArrayList<>();
   private List<PlanarRegion> tooSteepRegions = new ArrayList<>();
   private List<PlanarRegion> maskedRegions = new ArrayList<>();
   private HashMap<RegionInWorldInterface, List<ConcavePolygon2DBasics>> maskedRegionsExtrusions = new HashMap<>();

   private final FramePoint2D stanceFootPosition = new FramePoint2D();
   private final Random random = new Random(1738L);

   private final ObstacleRegionFilter obstacleRegionFilter = new ObstacleRegionFilter()
   {
      @Override
      public boolean isRegionValidObstacle(PlanarRegion potentialObstacleRegion, PlanarRegion navigableRegion)
      {
         if (potentialObstacleRegion == navigableRegion)
            return false;

         if (!PlanarRegionTools.isRegionAOverlappingWithRegionB(potentialObstacleRegion, navigableRegion, minimumDistanceFromCliffBottoms.getDoubleValue()))
            return false;

         boolean isCeiling = potentialObstacleRegion.getBoundingBox3dInWorld().getMinZ()
                             > navigableRegion.getBoundingBox3dInWorld().getMaxZ() + canDuckUnderHeight.getDoubleValue();
         if (isCeiling)
            return false;

         return PlanarRegionTools.isPlanarRegionAAbovePlanarRegionB(potentialObstacleRegion, navigableRegion, canEasilyStepOverHeight.getDoubleValue());
      }
   };

   private final ObstacleExtrusionDistanceCalculator obstacleExtrusionDistanceCalculator = new ObstacleExtrusionDistanceCalculator()
   {
      @Override
      public double computeExtrusionDistance(Point2DReadOnly pointToExtrude, double obstacleHeight)
      {
         if (obstacleHeight < 0.0)
         {
            return 0.0;
         }
         else if (obstacleHeight < canEasilyStepOverHeight.getDoubleValue())
         {
            //            return 0.01;
            double alpha = obstacleHeight / canEasilyStepOverHeight.getDoubleValue();
            return InterpolationTools.linearInterpolate(0.0, minimumDistanceFromCliffBottoms.getDoubleValue(), alpha);
         }
         else
         {
            return minimumDistanceFromCliffBottoms.getDoubleValue();
         }
      }
   };

   public SteppableRegionsCalculator(double maximumReach, YoRegistry registry)
   {
      maxAngleForSteppable = new YoDouble("maxAngleForSteppable", registry);
      minimumAreaForSteppable = new YoDouble("minimumAreaForSteppable", registry);
      maximumStepReach = new YoDouble("maximumStepReach", registry);
      canDuckUnderHeight = new YoDouble("canDuckUnderHeight", registry);
      canEasilyStepOverHeight = new YoDouble("canEasyStepOverHeight", registry);
      orthogonalAngle = new YoDouble("orthogonalAngle", registry);
      minimumDistanceFromCliffBottoms = new YoDouble("tooHighToStepDistance", registry);

      maxAngleForSteppable.set(maxNormalAngleFromVertical);
      minimumAreaForSteppable.set(minimumAreaToConsider);
      maximumStepReach.set(maximumReach);
      canDuckUnderHeight.set(defaultCanDuckUnderHeight);
      canEasilyStepOverHeight.set(defaultCanEasilyStepOverHeight);
      orthogonalAngle.set(defaultOrthogonalAngle);
      minimumDistanceFromCliffBottoms.set(defaultMinimumDistanceFromCliffBottoms);
   }

   public void setPlanarRegions(List<PlanarRegion> planarRegions)
   {
      allPlanarRegions = planarRegions;
   }

   public void setStanceFootPosition(FramePoint3DReadOnly stanceFootPosition)
   {
      this.stanceFootPosition.set(stanceFootPosition);
   }

   public void setCanEasilyStepOverHeight(double canEasilyStepOverHeight)
   {
      this.canEasilyStepOverHeight.set(canEasilyStepOverHeight);
   }

   public void setMinimumDistanceFromCliffBottoms(double minimumDistanceFromCliffBottoms)
   {
      this.minimumDistanceFromCliffBottoms.set(minimumDistanceFromCliffBottoms);
   }

   public void setOrthogonalAngle(double orthogonalAngle)
   {
      this.orthogonalAngle.set(orthogonalAngle);
   }

   public List<StepConstraintRegion> computeSteppableRegions()
   {
      tooSmallRegions = new ArrayList<>();
      tooSteepRegions = new ArrayList<>();
      maskedRegions = new ArrayList<>();
      List<PlanarRegion> candidateRegions = allPlanarRegions.stream().filter(this::isRegionValidForStepping).collect(Collectors.toList());

      steppableRegions = new ArrayList<>();
      obstacleExtrusions = new HashMap<>();
      maskedRegionsExtrusions = new HashMap<>();
      for (PlanarRegion candidateRegion : candidateRegions)
      {
         List<StepConstraintRegion> regions = createSteppableRegionsFromPlanarRegion(candidateRegion, allPlanarRegions);
         if (regions != null)
         {
            for (StepConstraintRegion region : regions)
            {
               if (candidateRegion.getRegionId() != -1)
                  region.setRegionId(candidateRegion.getRegionId());
               steppableRegions.add(region);
            }
         }
      }

      for (StepConstraintRegion stepConstraintRegion : steppableRegions)
      {
         if (stepConstraintRegion.getRegionId() == -1)
            stepConstraintRegion.setRegionId(random.nextInt());
      }

      return steppableRegions;
   }

   public HashMap<RegionInWorldInterface, List<ConcavePolygon2DBasics>> getObstacleExtrusions()
   {
      return obstacleExtrusions;
   }

   public List<PlanarRegion> getTooSmallRegions()
   {
      return tooSmallRegions;
   }

   public List<PlanarRegion> getTooSteepRegions()
   {
      return tooSteepRegions;
   }

   public List<PlanarRegion> getMaskedRegions()
   {
      return maskedRegions;
   }

   public HashMap<RegionInWorldInterface, List<ConcavePolygon2DBasics>> getMaskedRegionsObstacleExtrusions()
   {
      return maskedRegionsExtrusions;
   }

   private boolean isRegionValidForStepping(PlanarRegion planarRegion)
   {
      double angle = planarRegion.getNormal().angle(verticalAxis);

      if (angle > maxAngleForSteppable.getValue())
      {
         tooSteepRegions.add(planarRegion);
         return false;
      }

      if (PlanarRegionTools.computePlanarRegionArea(planarRegion) < minimumAreaForSteppable.getValue())
      {
         tooSmallRegions.add(planarRegion);
         return false;
      }

      if (stanceFootPosition.containsNaN())
         return true;

      return isRegionWithinReach(stanceFootPosition, maximumStepReach.getDoubleValue(), planarRegion);
   }

   private static boolean isRegionWithinReach(Point2DReadOnly point, double reach, PlanarRegion planarRegion)
   {
      // TODO do a check on the bounding box distance first

      Point2D pointInRegion = new Point2D(point);
      planarRegion.getTransformToLocal().transform(pointInRegion, false);
      if (planarRegion.getConvexHull().distance(pointInRegion) > reach)
         return false;

      boolean closeEnough = false;
      for (ConvexPolygon2DReadOnly convexPolygon : planarRegion.getConvexPolygons())
      {
         if (convexPolygon.distance(pointInRegion) < reach)
         {
            closeEnough = true;
            break;
         }
      }

      return closeEnough;
   }

   private List<StepConstraintRegion> createSteppableRegionsFromPlanarRegion(PlanarRegion candidateRegion, List<PlanarRegion> allOtherRegions)
   {
      ConcavePolygon2D candidateConstraintRegion = new ConcavePolygon2D();
      candidateConstraintRegion.addVertices(Vertex2DSupplier.asVertex2DSupplier(candidateRegion.getConcaveHull()));
      candidateConstraintRegion.update();

      List<PlanarRegion> obstacleRegions = allOtherRegions.stream()
                                                          .filter(candidate -> obstacleRegionFilter.isRegionValidObstacle(candidate, candidateRegion))
                                                          .collect(Collectors.toList());

      List<ConcavePolygon2DBasics> obstacleExtrusions = createObstacleExtrusions(candidateRegion, obstacleRegions);

      if (obstacleExtrusions.stream().anyMatch(region -> isRegionMasked(candidateConstraintRegion, region)))
      {
         maskedRegions.add(candidateRegion);
         maskedRegionsExtrusions.put(candidateRegion, obstacleExtrusions);
         return null;
      }

      List<StepConstraintRegion> stepConstraintRegions = createSteppableRegions(candidateRegion.getTransformToWorld(),
                                                                                candidateConstraintRegion,
                                                                                obstacleExtrusions);

      for (StepConstraintRegion stepConstraintRegion : stepConstraintRegions)
      {
         this.obstacleExtrusions.put(stepConstraintRegion, obstacleExtrusions);
      }

      return stepConstraintRegions;
   }

   // FIXME by not merging the obstacles, the crop sequence does matter. That is, a crop may make a hole not a hole
   private List<StepConstraintRegion> createSteppableRegions(RigidBodyTransformReadOnly transformToWorld,
                                                             ConcavePolygon2DBasics uncroppedPolygon,
                                                             List<ConcavePolygon2DBasics> obstacleExtrusions)
   {
      List<ConcavePolygon2DBasics> extrusionsCopy = new ArrayList<>(obstacleExtrusions);

      List<ConcavePolygon2DBasics> croppedPolygons = new ArrayList<>();
      croppedPolygons.add(uncroppedPolygon);

      // apply all the extrusions that clip, removing them as they are applied.
      // This has to been done via a brute force search, as applying one clip can cause a hole to create a clip on the next pass.
      int i = 0;
      while (i < extrusionsCopy.size())
      {
         if (applyExtrusionClip(extrusionsCopy.get(i), croppedPolygons))
         {
            extrusionsCopy.remove(i);
            i = 0;
         }
         else
         {
            i++;
         }
      }

      List<ConcavePolygon2DBasics> listOfHoles = extrusionsCopy.stream()
                                                               .filter(region -> GeometryPolygonTools.isPolygonInsideOtherPolygon(region, uncroppedPolygon))
                                                               .collect(Collectors.toList());

      // now assign the holes to their containing region region
      List<StepConstraintRegion> constraintRegions = new ArrayList<>();
      for (ConcavePolygon2DBasics croppedPolygon : croppedPolygons)
      {
         List<ConcavePolygon2DBasics> holesInRegion = new ArrayList<>();
         i = 0;
         while (i < listOfHoles.size())
         {
            ConcavePolygon2DBasics holeCandidate = listOfHoles.get(i);
            if (isObstacleAHole(croppedPolygon, holeCandidate))
            {
               holesInRegion.add(holeCandidate);
               listOfHoles.remove(i);
            }
            else
            {
               i++;
            }
         }

         constraintRegions.add(new StepConstraintRegion(transformToWorld, croppedPolygon, holesInRegion));
      }

      return constraintRegions;
   }

   /**
    * Returns whether or not it should be removed from this list
    **/
   private boolean applyExtrusionClip(ConcavePolygon2DReadOnly clippingPolygon, List<ConcavePolygon2DBasics> polygonsToModify)
   {
      boolean doesNotIntersect = polygonsToModify.stream().noneMatch(region -> GeometryPolygonTools.doPolygonsIntersect(clippingPolygon, region));
      if (doesNotIntersect)
      {
         if (polygonsToModify.stream().noneMatch(region -> GeometryPolygonTools.isPolygonInsideOtherPolygon(clippingPolygon, region)))
            return true;

         return false;
      }

      List<ConcavePolygon2DBasics> clippedPolygons = new ArrayList<>();
      for (ConcavePolygon2DBasics polygonToClip : polygonsToModify)
      {
         clippedPolygons.addAll(PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip));
      }
      polygonsToModify.clear();
      polygonsToModify.addAll(clippedPolygons);

      return true;
   }

   private static boolean isObstacleAHole(ConcavePolygon2DBasics constraintArea, ConcavePolygon2DReadOnly obstacleConcaveHull)
   {
      return GeometryPolygonTools.isPolygonInsideOtherPolygon(obstacleConcaveHull, constraintArea);
   }

   private static boolean isRegionMasked(ConcavePolygon2DBasics region, ConcavePolygon2DReadOnly candidateMask)
   {
      return GeometryPolygonTools.isPolygonInsideOtherPolygon(region, candidateMask);
   }

   private List<ConcavePolygon2DBasics> createObstacleExtrusions(PlanarRegion candidateRegion, List<PlanarRegion> obstacleRegions)
   {
      double zThresholdBeforeOrthogonal = Math.cos(orthogonalAngle.getDoubleValue());
      List<ConcavePolygon2DBasics> obstacleExtrusions = obstacleRegions.stream()
                                                                       .map(region -> createObstacleExtrusion(candidateRegion,
                                                                                                              region,
                                                                                                              obstacleExtrusionDistanceCalculator,
                                                                                                              zThresholdBeforeOrthogonal))
                                                                       .collect(Collectors.toList());

      PolygonClippingAndMerging.removeHolesFromList(obstacleExtrusions);

      return obstacleExtrusions;
   }

   static ConcavePolygon2D createObstacleExtrusion(PlanarRegion homeRegion,
                                                   PlanarRegion obstacleRegion,
                                                   ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator,
                                                   double zThresholdBeforeOrthogonal)
   {
      List<Point2D> concaveHull = obstacleRegion.getConcaveHull();

      RigidBodyTransformReadOnly transformFromObstacleToWorld = obstacleRegion.getTransformToWorld();

      // Transform the obstacle to world and also Project the obstacle to z = 0:
      List<Point3DReadOnly> obstacleClustersInWorld = new ArrayList<>();
      ClusterTools.calculatePointsInWorldAtRegionHeight(concaveHull, transformFromObstacleToWorld, homeRegion, null, obstacleClustersInWorld);

      if (!GeometryPolygonTools.isClockwiseOrdered3DZUp(obstacleClustersInWorld, obstacleClustersInWorld.size()))
         Collections.reverse(obstacleClustersInWorld);

      Vector3DReadOnly obstacleNormal = obstacleRegion.getNormal();
      boolean isObstacleWall = Math.abs(obstacleNormal.getZ()) < zThresholdBeforeOrthogonal;

      ClusterType obstacleClusterType = isObstacleWall ? ClusterType.MULTI_LINE : ClusterType.POLYGON;
      if (isObstacleWall)
         obstacleClustersInWorld = ClusterTools.filterVerticalPolygonForMultiLineExtrusion(obstacleClustersInWorld, POPPING_MULTILINE_POINTS_THRESHOLD);

      // actually extrude the points
      List<? extends Point2DReadOnly> extrusionInFlatWorld = ClusterTools.computeObstacleNavigableExtrusionsInLocal(obstacleClusterType,
                                                                                                                    obstacleClustersInWorld,
                                                                                                                    extrusionDistanceCalculator,
                                                                                                                    true);

      // Project the points back up to the home region.
      RigidBodyTransformReadOnly transformFromWorldToHome = homeRegion.getTransformToLocal();
      ExtrusionHull extrusionsInHomeRegion = ClusterTools.projectPointsVerticallyToPlanarRegionLocal(homeRegion,
                                                                                                     extrusionInFlatWorld,
                                                                                                     transformFromWorldToHome);

      removeDuplicatedPoints(extrusionsInHomeRegion, 1e-5);

      return new ConcavePolygon2D(Vertex2DSupplier.asVertex2DSupplier(extrusionsInHomeRegion.getPoints()));
   }

   private static void removeDuplicatedPoints(ExtrusionHull hullToFilter, double distanceEpsilon)
   {
      double epsilonSquared = distanceEpsilon * distanceEpsilon;
      int i = 0;
      while (i < hullToFilter.getPoints().size())
      {
         Point2DReadOnly point = hullToFilter.get(i);
         int j = i + 1;
         while (j < hullToFilter.getPoints().size())
         {
            Point2DReadOnly otherPoint = hullToFilter.get(j);
            if (point.distanceSquared(otherPoint) < epsilonSquared)
               hullToFilter.getPoints().remove(j);
            else
               j++;
         }
         i++;
      }
   }
}
