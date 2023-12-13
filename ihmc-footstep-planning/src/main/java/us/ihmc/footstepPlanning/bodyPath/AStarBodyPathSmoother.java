package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.*;
import java.util.stream.Collectors;

public class AStarBodyPathSmoother
{
   static final int maxPoints = 80;
   static final double gradientEpsilon = 1e-6;

   static final int minIterations = 20;

   static final double yOffsetTraversibilityNominalWindow = 0.27;
   static final double yOffsetTraversibilityGradientWindow = 0.12;
   static final double traversibilitySampleWindowX = 0.2;
   static final double traversibilitySampleWindowY = 0.14;

   static final int iterations = 180;

   static final int turnPointIteration = 12;
   private final TIntArrayList turnPointIndices = new TIntArrayList();
   static final int minTurnPointProximity = 7;
   static final double turnPointYawThreshold = Math.toRadians(30.0);

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoInteger iteration = new YoInteger("iterations", registry);
   private final YoDouble maxCollision = new YoDouble("maxCollision", registry);
   private final TickAndUpdatable tickAndUpdatable;
   private final boolean visualize;

   private int pathSize;
   private final AStarBodyPathPlannerParametersReadOnly plannerParameters;
   private HeightMapLeastSquaresNormalCalculator leastSquaresNormalCalculator = new HeightMapLeastSquaresNormalCalculator();
   private HeightMapRANSACNormalCalculator ransacNormalCalculator = new HeightMapRANSACNormalCalculator();

   private final AStarBodyPathSmootherWaypoint[] waypoints = new AStarBodyPathSmootherWaypoint[maxPoints];

   /* Cost gradient in the direction of higher cost for each coordinate */
   private final YoVector2D[] gradients = new YoVector2D[maxPoints];

   public AStarBodyPathSmoother(AStarBodyPathPlannerParametersReadOnly plannerParameters)
   {
      this(plannerParameters, null, null, null);
   }

   public AStarBodyPathSmoother(AStarBodyPathPlannerParametersReadOnly plannerParameters, TickAndUpdatable tickAndUpdatable, YoGraphicsListRegistry graphicsListRegistry, YoRegistry parentRegistry)
   {
      this.plannerParameters = plannerParameters;
      for (int i = 0; i < maxPoints; i++)
      {
         gradients[i] = new YoVector2D("gradient" + i, registry);
      }

      for (int i = 0; i < maxPoints; i++)
      {
         waypoints[i] = new AStarBodyPathSmootherWaypoint(i, plannerParameters, graphicsListRegistry, (parentRegistry == null) ? null : registry);
      }

      if (parentRegistry == null)
      {
         visualize = false;
         this.tickAndUpdatable = null;
      }
      else
      {
         graphicsListRegistry.getYoGraphicsLists().stream().filter(list -> list.getLabel().equals("Collisions")).forEach(list -> list.setVisible(false));
         graphicsListRegistry.getYoGraphicsLists().stream().filter(list -> list.getLabel().equals("Normals")).forEach(list -> list.setVisible(false));
         graphicsListRegistry.getYoGraphicsLists().stream().filter(list -> list.getLabel().equals("Step Poses")).forEach(list -> list.setVisible(false));
         this.tickAndUpdatable = tickAndUpdatable;
         parentRegistry.addChild(registry);
         visualize = true;
      }
   }

   public List<Pose3D> doSmoothing(List<Point3D> bodyPath, HeightMapData heightMapData)
   {
      LogTools.info("Starting waypoint optimization");
      pathSize = bodyPath.size();
      turnPointIndices.clear();

      if (pathSize > maxPoints)
      {
         throw new RuntimeException("Too many body path waypoints to smooth. Path size = " + bodyPath.size() + ", Max points = " + maxPoints);
      }

      if (pathSize == 2)
      {
         Point3D start = bodyPath.get(0);
         Point3D goal = bodyPath.get(1);
         double yaw = Math.atan2(goal.getY() - start.getY(), goal.getX() - start.getX());
         List<Pose3D> waypoints = new ArrayList<>();
         waypoints.add(new Pose3D(start, new Quaternion(yaw, 0.0, 0.0)));
         waypoints.add(new Pose3D(goal, new Quaternion(yaw, 0.0, 0.0)));
         return waypoints;
      }

      for (int i = 0; i < pathSize; i++)
      {
         waypoints[i].setNeighbors(waypoints);
      }

      if (heightMapData != null)
      {
         double patchWidth = 0.7;
         leastSquaresNormalCalculator.computeSurfaceNormals(heightMapData, patchWidth);
         ransacNormalCalculator.initialize(heightMapData);
      }

      for (int i = 0; i < maxPoints; i++)
      {
         waypoints[i].initialize(bodyPath, heightMapData, ransacNormalCalculator, leastSquaresNormalCalculator);
      }

      for (int i = 1; i < bodyPath.size() - 1; i++)
      {
         waypoints[i].update(true);
      }

      if (visualize)
      {
         iteration.set(-1);
         tickAndUpdatable.tickAndUpdate();
      }

      for (iteration.set(0); iteration.getValue() < iterations; iteration.increment())
      {
         maxCollision.set(0.0);

         for (int waypointIndex = 1; waypointIndex < pathSize - 1; waypointIndex++)
         {
            computeSmoothenessGradient(waypointIndex, gradients[waypointIndex]);

//            Tuple3DReadOnly displacementGradient = waypoints[waypointIndex].computeDisplacementGradient();
//            gradients[waypointIndex].add(displacementGradient.getX(), displacementGradient.getY());
         }

         if (heightMapData != null)
         {
            for (int waypointIndex = 1; waypointIndex < pathSize - 1; waypointIndex++)
            {
               waypoints[waypointIndex].computeCurrentTraversibility();
            }

            for (int waypointIndex = 2; waypointIndex < pathSize - 2; waypointIndex++)
            {
               /* Collision gradient */
               gradients[waypointIndex].add(waypoints[waypointIndex].computeCollisionGradient());
               maxCollision.set(Math.max(waypoints[waypointIndex].getMaxCollision(), maxCollision.getValue()));

               /* Traversibility gradient */
               Tuple3DReadOnly traversibilityGradient = waypoints[waypointIndex].computeTraversibilityGradient();
               gradients[waypointIndex].sub(traversibilityGradient.getX(), traversibilityGradient.getY());

               if (waypoints[waypointIndex].isTurnPoint())
               {
                  continue;
               }

               /* Ground plane gradient */
               Tuple3DReadOnly groundPlaneGradient = waypoints[waypointIndex].computeGroundPlaneGradient();
               gradients[waypointIndex].sub(groundPlaneGradient.getX(), groundPlaneGradient.getY());

               /* Roll-z gradient */
               Vector2DBasics rollGradient = waypoints[waypointIndex].computeRollInclineGradient(heightMapData);
               gradients[waypointIndex - 1].sub(rollGradient);
               gradients[waypointIndex + 1].add(rollGradient);

               if (visualize)
               {
                  if (waypointIndex - 1 != 0)
                  {
                     waypoints[waypointIndex - 1].updateRollGraphics(-rollGradient.getX(), -rollGradient.getY());
                  }
                  if (waypointIndex + 1 != pathSize - 1)
                  {
                     waypoints[waypointIndex + 1].updateRollGraphics(rollGradient.getX(), rollGradient.getY());
                  }
               }
            }

            double gradientMagnitudeSq = 0.0;
            for (int i = 0; i < gradients.length; i++)
            {
               gradientMagnitudeSq += EuclidCoreTools.normSquared(gradients[i].getX(), gradients[i].getY());
            }

            if (gradientMagnitudeSq < plannerParameters.getSmootherGradientThresholdToTerminate() && iteration.getValue() > minIterations)
            {
               break;
            }
         }

         for (int j = 1; j < pathSize - 1; j++)
         {
            waypoints[j].getPosition().subX(plannerParameters.getSmootherHillClimbGain() * gradients[j].getX());
            waypoints[j].getPosition().subY(plannerParameters.getSmootherHillClimbGain() * gradients[j].getY());
         }

         for (int j = 1; j < pathSize - 1; j++)
         {
            waypoints[j].update(false);
         }

         if (iteration.getValue() == turnPointIteration)
         {
            computeTurnPoints();
         }

         if (visualize)
         {
            tickAndUpdatable.tickAndUpdate();
         }
      }

      List<Pose3D> smoothedPath = new ArrayList<>();
      for (int i = 0; i < pathSize; i++)
      {
         smoothedPath.add(new Pose3D(waypoints[i].getPose()));
      }

      return smoothedPath;
   }

   private void computeSmoothenessGradient(int waypointIndex, Vector2DBasics gradientToSet)
   {
      boolean isTurnPoint = waypoints[waypointIndex].isTurnPoint();

      double x0 = waypoints[waypointIndex - 1].getPosition().getX();
      double y0 = waypoints[waypointIndex - 1].getPosition().getY();
      double x1 = waypoints[waypointIndex].getPosition().getX();
      double y1 = waypoints[waypointIndex].getPosition().getY();
      double x2 = waypoints[waypointIndex + 1].getPosition().getX();
      double y2 = waypoints[waypointIndex + 1].getPosition().getY();

      /* Equal spacing gradient */
      double spacingGradientX = -4.0 * plannerParameters.getSmootherEqualSpacingWeight() * (x2 - 2.0 * x1 + x0);
      double spacingGradientY = -4.0 * plannerParameters.getSmootherEqualSpacingWeight() * (y2 - 2.0 * y1 + y0);
      double alphaTurnPoint = isTurnPoint ?  plannerParameters.getSmootherTurnPointSmoothnessDiscount() : 1.0;
      gradientToSet.setX(alphaTurnPoint * spacingGradientX);
      gradientToSet.setY(alphaTurnPoint * spacingGradientY);

      /* Smoothness gradient */
      double smoothnessGradientX, smoothnessGradientY;
      if (isTurnPoint)
      {
         smoothnessGradientX = 0.0;
         smoothnessGradientY = 0.0;
      }
      else
      {
         double minCurvatureToPenalize = Math.toRadians(plannerParameters.getSmootherMinCurvatureToPenalize());
         double exp = 1.5;
         double f0 = Math.pow(computeDeltaHeadingMagnitude(x0, y0, x1, y1, x2, y2, minCurvatureToPenalize), exp);
         double fPdx = Math.pow(computeDeltaHeadingMagnitude(x0, y0, x1 + gradientEpsilon, y1, x2, y2, minCurvatureToPenalize), exp);
         double fPdy = Math.pow(computeDeltaHeadingMagnitude(x0, y0, x1, y1 + gradientEpsilon, x2, y2, minCurvatureToPenalize), exp);
         smoothnessGradientX = plannerParameters.getSmootherSmoothnessWeight() * (fPdx - f0) / gradientEpsilon;
         smoothnessGradientY = plannerParameters.getSmootherSmoothnessWeight() * (fPdy - f0) / gradientEpsilon;
         gradientToSet.addX(smoothnessGradientX);
         gradientToSet.addY(smoothnessGradientY);
      }

      if (visualize)
      {
         waypoints[waypointIndex].updateGradientGraphics(spacingGradientX, spacingGradientY, smoothnessGradientX, smoothnessGradientY);
      }
   }

   private static double computeDeltaHeadingMagnitude(double x0, double y0, double x1, double y1, double x2, double y2, double deadband)
   {
      double heading0 = Math.atan2(y1 - y0, x1 - x0);
      double heading1 = Math.atan2(y2 - y1, x2 - x1);
      return Math.max(Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(heading1, heading0)) - deadband, 0.0);
   }

   private void computeTurnPoints()
   {
      int minPathSizeForTurnPoints = 5;
      if (pathSize < minPathSizeForTurnPoints)
      {
         return;
      }

      List<Pair<Double, Integer>> headingIndexList = new ArrayList<>();
      for (int i = 2; i < pathSize - 2; i++)
      {
         double x0 = waypoints[i - 1].getPosition().getX();
         double y0 = waypoints[i - 1].getPosition().getY();
         double x1 = waypoints[i].getPosition().getX();
         double y1 = waypoints[i].getPosition().getY();
         double x2 = waypoints[i + 1].getPosition().getX();
         double y2 = waypoints[i + 1].getPosition().getY();

         double deltaHeading = computeDeltaHeadingMagnitude(x0, y0, x1, y1, x2, y2, 0.0);
         headingIndexList.add(Pair.of(deltaHeading, i));
      }

      headingIndexList = headingIndexList.stream()
                                         .filter(hi -> hi.getKey() > turnPointYawThreshold)
                                         .sorted(Comparator.comparingDouble(Pair::getKey))
                                         .collect(Collectors.toList());

      boolean[] canBeTurnPoint = new boolean[pathSize];
      Arrays.fill(canBeTurnPoint, true);

      for (int i = headingIndexList.size() - 1; i >= 0; i--)
      {
         int index = headingIndexList.get(i).getRight();
         if (canBeTurnPoint[index])
         {
            waypoints[index].setTurnPoint();

            for (int j = 1; j < minTurnPointProximity; j++)
            {
               canBeTurnPoint[Math.max(0, index - j)] = false;
               canBeTurnPoint[Math.min(pathSize - 1, index + j)] = false;
            }
         }
      }
   }

   public void setLeastSquaresNormalCalculator(HeightMapLeastSquaresNormalCalculator leastSquaresNormalCalculator)
   {
      this.leastSquaresNormalCalculator = leastSquaresNormalCalculator;
   }

   public void setRansacNormalCalculator(HeightMapRANSACNormalCalculator ransacNormalCalculator)
   {
      this.ransacNormalCalculator = ransacNormalCalculator;
   }
}
