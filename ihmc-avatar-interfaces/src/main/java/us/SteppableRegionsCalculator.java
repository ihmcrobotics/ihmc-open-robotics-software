package us;

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
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DReadOnly;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;
import us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton.PolygonClippingAndMerging;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;
import java.util.stream.Collectors;

public class SteppableRegionsCalculator
{
   private static final double POPPING_MULTILINE_POINTS_THRESHOLD = MathTools.square(0.10);

   private static final double maxNormalAngleFromVertical = 0.3;
   private static final double minimumAreaToConsider = 0.01;
   private static final double defaultCanDuckUnderHeight = 2.0;
   private static final double defaultCanEasilyStepOverHeight = 0.1;
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

   private HashMap<StepConstraintRegion, List<ConcavePolygon2DBasics>> obstacleExtrusions = new HashMap<>();
   private List<StepConstraintRegion> steppableRegions = new ArrayList<>();
   private List<PlanarRegion> allPlanarRegions = new ArrayList<>();

   private final FramePoint2D stanceFootPosition = new FramePoint2D();
   private final Random random = new Random(1738L);


   /** See notes in {@link VisibilityGraphsparametersReadOnly} */
   private final ObstacleRegionFilter obstacleRegionFilter = new ObstacleRegionFilter()
   {
      @Override
      public boolean isRegionValidObstacle(PlanarRegion potentialObstacleRegion, PlanarRegion navigableRegion)
      {
         if (!PlanarRegionTools.isRegionAOverlappingWithRegionB(potentialObstacleRegion, navigableRegion, minimumDistanceFromCliffBottoms.getDoubleValue()))
            return false;

         if (potentialObstacleRegion.getBoundingBox3dInWorld().getMinZ()
             > navigableRegion.getBoundingBox3dInWorld().getMaxZ() + canDuckUnderHeight.getDoubleValue())
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
            return 0.01;
//            double alpha = obstacleHeight / canEasilyStepOverHeight.getDoubleValue();
//            return InterpolationTools.linearInterpolate(0.0, minimumDistanceFromCliffBottoms.getDoubleValue(), alpha);
         }
         else
         {
            return minimumDistanceFromCliffBottoms.getDoubleValue();
         }
      }
   };

   public SteppableRegionsCalculator(double maximumReach, YoVariableRegistry registry)
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
      List<PlanarRegion> candidateRegions = allPlanarRegions.stream().filter(this::isRegionValidForStepping).collect(Collectors.toList());

      steppableRegions = new ArrayList<>();
      obstacleExtrusions = new HashMap<>();
      for (PlanarRegion candidateRegion : candidateRegions)
      {
         List<StepConstraintRegion> regions = createSteppableRegionsFromPlanarRegion(candidateRegion, allPlanarRegions);
         if (regions != null)
            steppableRegions.addAll(regions);
      }

      for (StepConstraintRegion stepConstraintRegion : steppableRegions)
         stepConstraintRegion.setRegionId(random.nextInt());

      return steppableRegions;
   }

   public HashMap<StepConstraintRegion, List<ConcavePolygon2DBasics>> getObstacleExtrusions()
   {
      return obstacleExtrusions;
   }

   private boolean isRegionValidForStepping(PlanarRegion planarRegion)
   {
      double angle = planarRegion.getNormal().angle(verticalAxis);

      if (angle > maxAngleForSteppable.getValue())
         return false;

      if (PlanarRegionTools.computePlanarRegionArea(planarRegion) < minimumAreaForSteppable.getValue())
         return false;

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
      List<StepConstraintRegion> stepConstraintRegions = createSteppableRegions(candidateRegion.getTransformToWorld(), candidateConstraintRegion, obstacleExtrusions);

      if (stepConstraintRegions != null)
      {
         for (StepConstraintRegion stepConstraintRegion : stepConstraintRegions)
            this.obstacleExtrusions.put(stepConstraintRegion, obstacleExtrusions);
      }

      return stepConstraintRegions;
   }

   private List<StepConstraintRegion> createSteppableRegions(RigidBodyTransformReadOnly transformToWorld,
                                                             ConcavePolygon2DBasics uncroppedPolygon,
                                                             List<ConcavePolygon2DBasics> obstacleExtrusions)
   {
      if (obstacleExtrusions.stream().anyMatch(region -> isRegionMasked(uncroppedPolygon, region)))
         return null;

      List<ConcavePolygon2DBasics> listOfHoles = obstacleExtrusions.stream()
                                                                   .filter(region -> isObstacleAHole(uncroppedPolygon, region))
                                                                   .collect(Collectors.toList());
      obstacleExtrusions.removeAll(listOfHoles);

      List<ConcavePolygon2DBasics> croppedPolygons = new ArrayList<>();
      croppedPolygons.add(uncroppedPolygon);

      // apply the polygons that we know will cause a clip
      for (ConcavePolygon2DBasics obstacleExtrusion : obstacleExtrusions)
      {
         croppedPolygons = clipPolygons(obstacleExtrusion, croppedPolygons);
      }

      // TODO clean this thing up
      // now assign the holes to the right region
      List<StepConstraintRegion> constraintRegions = new ArrayList<>();
      for (ConcavePolygon2DBasics croppedPolygon : croppedPolygons)
      {
         List<ConcavePolygon2DBasics> holesInRegion = new ArrayList<>();
         int i = 0;
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

   private List<ConcavePolygon2DBasics> clipPolygons(ConcavePolygon2DReadOnly clippingPolygon, List<ConcavePolygon2DBasics> polygonsToClip)
   {
      List<ConcavePolygon2DBasics> clippedPolygons = new ArrayList<>();
      for (ConcavePolygon2DBasics polygonToClip : polygonsToClip)
         clippedPolygons.addAll(PolygonClippingAndMerging.removeAreaInsideClip(clippingPolygon, polygonToClip));

      return clippedPolygons;
   }

   private boolean isObstacleAHole(ConcavePolygon2DBasics constraintArea, ConcavePolygon2DReadOnly obstacleConcaveHull)
   {
      return GeometryPolygonTools.isPolygonInsideOtherPolygon(obstacleConcaveHull, constraintArea);
   }

   private boolean isRegionMasked(ConcavePolygon2DBasics region, ConcavePolygon2DReadOnly candidateMask)
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
      PolygonClippingAndMerging.mergeAllPossible(obstacleExtrusions);

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
