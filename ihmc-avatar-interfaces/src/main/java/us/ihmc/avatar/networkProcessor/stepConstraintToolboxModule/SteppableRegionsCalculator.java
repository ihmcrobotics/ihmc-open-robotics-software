package us.ihmc.avatar.networkProcessor.stepConstraintToolboxModule;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ClusterType;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.ExtrusionHull;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleExtrusionDistanceCalculator;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.ObstacleRegionFilter;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2D;
import us.ihmc.robotics.geometry.concavePolygon2D.ConcavePolygon2DBasics;
import us.ihmc.robotics.geometry.concavePolygon2D.weilerAtherton.WeilerAthertonPolygonClipping;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
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

   private List<PlanarRegion> steppableRegions = new ArrayList<>();
   private List<PlanarRegion> allPlanarRegions = new ArrayList<>();

   private final FramePoint2D stanceFootPosition = new FramePoint2D();

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
            double alpha = obstacleHeight / canEasilyStepOverHeight.getDoubleValue();
            return InterpolationTools.linearInterpolate(0.0, minimumDistanceFromCliffBottoms.getDoubleValue(), alpha);
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
      steppableRegions = planarRegions.stream().filter(this::isRegionValidForStepping).collect(Collectors.toList());
   }

   public void setStanceFootPosition(FramePoint3DReadOnly stanceFootPosition)
   {
      this.stanceFootPosition.set(stanceFootPosition);
   }

   public List<PlanarRegion> computeSteppableRegions()
   {
      List<PlanarRegion> candidateRegions = allPlanarRegions.stream().filter(this::isRegionValidForStepping).collect(Collectors.toList());

      steppableRegions = candidateRegions.stream().map(region -> createSteppableRegionFromPlanarRegion(region, allPlanarRegions)).collect(Collectors.toList());
      return steppableRegions;
   }

   private boolean isRegionValidForStepping(PlanarRegion planarRegion)
   {
      double angle = planarRegion.getNormal().angle(verticalAxis);

      if (angle > maxAngleForSteppable.getValue())
         return false;

      if (PlanarRegionTools.computePlanarRegionArea(planarRegion) > minimumAreaForSteppable.getValue())
         return false;

      return isRegionWithinReach(stanceFootPosition, maximumStepReach.getDoubleValue(), planarRegion);
   }

   private static boolean isRegionWithinReach(Point2DReadOnly point, double reach, PlanarRegion planarRegion)
   {
      // TODO do a check on the bounding box distance first

      if (planarRegion.getConvexHull().distance(point) > reach)
         return false;

      boolean closeEnough = false;
      for (ConvexPolygon2DReadOnly convexPolygon : planarRegion.getConvexPolygons())
      {
         if (convexPolygon.distance(point) < reach)
         {
            closeEnough = true;
            break;
         }
      }
      return closeEnough;
   }

   private PlanarRegion createSteppableRegionFromPlanarRegion(PlanarRegion candidateRegion, List<PlanarRegion> allOtherRegions)
   {
      ConcavePolygon2D candidateConstraintRegion = new ConcavePolygon2D();
      candidateConstraintRegion.addVertices(Vertex2DSupplier.asVertex2DSupplier(candidateRegion.getConcaveHull()));
      candidateConstraintRegion.update();

      List<PlanarRegion> obstacleRegions = allOtherRegions.stream()
                                                          .filter(candidate -> obstacleRegionFilter.isRegionValidObstacle(candidate, candidateRegion))
                                                          .collect(Collectors.toList());

      double zThresholdBeforeOrthogonal = Math.cos(orthogonalAngle.getDoubleValue());

      obstacleRegions.forEach(region -> removeObstacleFromSteppableArea(candidateConstraintRegion, candidateRegion, region, zThresholdBeforeOrthogonal));

      return new PlanarRegion(candidateRegion.getTransformToWorld(), candidateConstraintRegion);
   }

   private void removeObstacleFromSteppableArea(ConcavePolygon2DBasics constraintArea,
                                                PlanarRegion constraintRegion,
                                                PlanarRegion obstacleRegion,
                                                double zThresholdBeforeOrthogonal)
   {
      List<Point2DReadOnly> obstacleExtrusion = createObstacleExtrusion(constraintRegion,
                                                                        obstacleRegion,
                                                                        obstacleExtrusionDistanceCalculator,
                                                                        zThresholdBeforeOrthogonal);

      ConcavePolygon2D obstacleConcaveHull = new ConcavePolygon2D();
      obstacleConcaveHull.addVertices(Vertex2DSupplier.asVertex2DSupplier(obstacleExtrusion));

      // TODO check out if it's contained in the concave hull
      ConcavePolygon2D clippedArea = new ConcavePolygon2D();
      WeilerAthertonPolygonClipping.clip(obstacleConcaveHull, constraintArea, clippedArea);

      constraintArea.set(clippedArea);
   }

   private static List<Point2DReadOnly> createObstacleExtrusion(PlanarRegion homeRegion,
                                                                PlanarRegion obstacleRegion,
                                                                ObstacleExtrusionDistanceCalculator extrusionDistanceCalculator,
                                                                double zThresholdBeforeOrthogonal)
   {
      List<Point2D> concaveHull = obstacleRegion.getConcaveHull();

      RigidBodyTransformReadOnly transformFromObstacleToWorld = obstacleRegion.getTransformToWorld();

      // Transform the obstacle to world and also Project the obstacle to z = 0:
      List<Point3DReadOnly> obstacleClusterPointsWithZeroZ = new ArrayList<>();
      for (int i = 0; i < concaveHull.size(); i++)
      {
         Point2DReadOnly obstacleConcaveHullVertexInLocal = concaveHull.get(i);
         Point3D obstacleConcaveHullVertexInWorld = new Point3D(obstacleConcaveHullVertexInLocal);
         obstacleConcaveHullVertexInWorld.applyTransform(transformFromObstacleToWorld);

         double zInHomeRegion = homeRegion.getPlaneZGivenXY(obstacleConcaveHullVertexInWorld.getX(), obstacleConcaveHullVertexInWorld.getY());

         double obstacleHeight = obstacleConcaveHullVertexInWorld.getZ() - zInHomeRegion;
         Point3D temporaryClusterPoint = new Point3D(obstacleConcaveHullVertexInWorld);
         temporaryClusterPoint.setZ(obstacleHeight);

         obstacleClusterPointsWithZeroZ.add(temporaryClusterPoint);
      }

      Vector3D obstacleNormal = obstacleRegion.getNormal();
      boolean verticalObstacle = Math.abs(obstacleNormal.getZ()) < zThresholdBeforeOrthogonal;

      ClusterType obstacleClusterType = verticalObstacle ? ClusterType.MULTI_LINE : ClusterType.POLYGON;
      if (verticalObstacle)
         obstacleClusterPointsWithZeroZ = ClusterTools.filterVerticalPolygonForMultiLineExtrusion(obstacleClusterPointsWithZeroZ,
                                                                                                  POPPING_MULTILINE_POINTS_THRESHOLD);

      // actually extrude the points
      List<? extends Point2DReadOnly> nonNavigableExtrusionsInFlatWorld = ClusterTools.computeObstacleNonNavigableExtrusionsInLocal(obstacleClusterType,
                                                                                                                                    obstacleClusterPointsWithZeroZ,
                                                                                                                                    extrusionDistanceCalculator);

      // Project the points back up to the home region.
      RigidBodyTransformReadOnly transformFromWorldToHome = homeRegion.getTransformToLocal();
      ExtrusionHull nonNavigableExtrusionsInHomeRegionLocal = ClusterTools.projectPointsVerticallyToPlanarRegionLocal(homeRegion,
                                                                                                                      nonNavigableExtrusionsInFlatWorld,
                                                                                                                      transformFromWorldToHome);

      return nonNavigableExtrusionsInHomeRegionLocal.getPoints();
   }
}
