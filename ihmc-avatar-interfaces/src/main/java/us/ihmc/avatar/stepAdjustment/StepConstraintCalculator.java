package us.ihmc.avatar.stepAdjustment;

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
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintListConverter;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.StepConstraintRegion;
import us.ihmc.robotics.RegionInWorldInterface;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.GeometryPolygonTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.*;
import java.util.stream.Collectors;

public class StepConstraintCalculator
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

   private HashMap<RegionInWorldInterface<?>, List<ConcavePolygon2DBasics>> obstacleExtrusionsMap = new HashMap<>();
   private List<StepConstraintRegion> steppableRegions = new ArrayList<>();
   private List<PlanarRegion> allPlanarRegions = new ArrayList<>();
   private List<PlanarRegion> tooSmallRegions = new ArrayList<>();
   private List<PlanarRegion> tooSteepRegions = new ArrayList<>();
   private List<PlanarRegion> maskedRegions = new ArrayList<>();
   private HashMap<RegionInWorldInterface<?>, List<ConcavePolygon2DBasics>> maskedRegionsExtrusions = new HashMap<>();

   private final FramePoint2D stanceFootPosition = new FramePoint2D();

   public StepConstraintCalculator(double maximumReach, YoRegistry registry)
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

      // first, filter out all the regions that are invalid for stepping
      List<PlanarRegion> candidateRegions = allPlanarRegions.stream().filter(this::isRegionValidForStepping).collect(Collectors.toList());

  
      obstacleExtrusionsMap = new HashMap<>();
      maskedRegionsExtrusions = new HashMap<>();

      steppableRegions = StepConstraintListConverter.convertPlanarRegionListToStepConstraintRegion(candidateRegions);

      return steppableRegions;
   }

   public HashMap<RegionInWorldInterface<?>, List<ConcavePolygon2DBasics>> getObstacleExtrusions()
   {
      return obstacleExtrusionsMap;
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

   public HashMap<RegionInWorldInterface<?>, List<ConcavePolygon2DBasics>> getMaskedRegionsObstacleExtrusions()
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


   static ConcavePolygon2D extrudePlanarRegionToCreateObstacleExtrusion(PlanarRegion homeRegion,
                                                                        PlanarRegion obstacleRegion,
                                                                        ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator,
                                                                        double zThresholdBeforeOrthogonal)
   {
      List<? extends Point2DReadOnly> concaveHull = obstacleRegion.getConcaveHull();

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
      List<Point2D> extrusionsInHomeRegion = ClusterTools.projectPointsVerticallyToPlanarRegionLocal(homeRegion,
                                                                                                     extrusionInFlatWorld,
                                                                                                     transformFromWorldToHome);

      removeDuplicatedPoints(extrusionsInHomeRegion, 1e-5);

      return new ConcavePolygon2D(Vertex2DSupplier.asVertex2DSupplier(extrusionsInHomeRegion));
   }

   private static void removeDuplicatedPoints(List<Point2D> hullToFilter, double distanceEpsilon)
   {
      double epsilonSquared = distanceEpsilon * distanceEpsilon;
      int i = 0;
      while (i < hullToFilter.size())
      {
         Point2DReadOnly point = hullToFilter.get(i);
         int j = i + 1;
         while (j < hullToFilter.size())
         {
            Point2DReadOnly otherPoint = hullToFilter.get(j);
            if (point.distanceSquared(otherPoint) < epsilonSquared)
               hullToFilter.remove(j);
            else
               j++;
         }
         i++;
      }
   }
}
