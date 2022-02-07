package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

public class AStarBodyPathSmoother
{
   static final double collisionWeight = 700.0;
   static final double smoothnessWeight = 1.0;
   static final double equalSpacingWeight = 2.0;
   static final double rollWeight = 150.0;

   private static final int maxPoints = 200;
   private static final double gradientEpsilon = 1e-6;
   private static final double hillClimbGain = 5e-4;
   private static final double minCurvatureToPenalize = Math.toRadians(5.0);

   private static final int iterations = 500;

   private static final int turnPointIteration = 12;
   private final TIntArrayList turnPointIndices = new TIntArrayList();
   private static final int minTurnPointProximity = 7;
   private static final double turnPointYawThreshold = Math.toRadians(30.0);

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoInteger iteration = new YoInteger("iterations", registry);
   private final YoDouble maxCollision = new YoDouble("maxCollision", registry);
   private final TickAndUpdatable tickAndUpdatable;
   private final boolean visualize;

   private int pathSize;
   private List<Point3D> initialBodyPath;
   private final HeightMapLeastSquaresNormalCalculator surfaceNormalCalculator = new HeightMapLeastSquaresNormalCalculator();

   private final AStarBodyPathSmootherWaypoint[] waypoints = new AStarBodyPathSmootherWaypoint[maxPoints];

   /* Cost gradient in the direction of higher cost for each coordinate */
   private final YoVector2D[] gradients = new YoVector2D[maxPoints];

   public AStarBodyPathSmoother()
   {
      this(null, null, null);
   }

   public AStarBodyPathSmoother(TickAndUpdatable tickAndUpdatable, YoGraphicsListRegistry graphicsListRegistry, YoRegistry parentRegistry)
   {
      for (int i = 0; i < maxPoints; i++)
      {
         gradients[i] = new YoVector2D("gradient" + i, registry);
      }

      for (int i = 0; i < maxPoints; i++)
      {
         waypoints[i] = new AStarBodyPathSmootherWaypoint(i, surfaceNormalCalculator, graphicsListRegistry, (parentRegistry == null) ? null : registry);
      }

      for (int i = 1; i < maxPoints - 1; i++)
      {
         waypoints[i].setNeighbors(waypoints[i - 1], waypoints[i + 1]);
      }

      if (parentRegistry == null)
      {
         visualize = false;
         this.tickAndUpdatable = null;
      }
      else
      {
         graphicsListRegistry.getYoGraphicsLists().stream().filter(list -> list.getLabel().equals("Collisions")).forEach(list -> list.setVisible(false));
         this.tickAndUpdatable = tickAndUpdatable;
         parentRegistry.addChild(registry);
         visualize = true;
      }
   }

   public List<Point3D> doSmoothing(List<Point3D> bodyPath, HeightMapData heightMapData)
   {
      initialBodyPath = bodyPath;
      pathSize = bodyPath.size();
      turnPointIndices.clear();

      if (pathSize > maxPoints)
      {
         throw new RuntimeException("Too many body path waypoints to smooth. Path size = " + bodyPath.size() + ", Max points = " + maxPoints);
      }

      if (pathSize == 2)
      {
         return bodyPath;
      }

      if (heightMapData != null)
      {
         double patchWidth = 0.7;
         surfaceNormalCalculator.computeSurfaceNormals(heightMapData, patchWidth);
      }

      for (int i = 0; i < maxPoints; i++)
      {
         waypoints[i].initialize(bodyPath, heightMapData);
      }

      for (int i = 1; i < bodyPath.size() - 1; i++)
      {
         updateHeading(i);
      }

      if (visualize)
      {
         iteration.set(-1);
         tickAndUpdatable.tickAndUpdate();
      }

      for (iteration.set(0); iteration.getValue() < iterations; iteration.increment())
      {
         maxCollision.set(0.0);

         for (int j = 1; j < pathSize - 1; j++)
         {
            computeSmoothenessGradient(j, gradients[j]);
         }

         if (heightMapData != null)
         {
            for (int j = 1; j < pathSize - 1; j++)
            {
               gradients[j].add(waypoints[j].computeCollisionGradient());
               maxCollision.set(Math.max(waypoints[j].getMaxCollision(), maxCollision.getValue()));

               if (waypoints[j].isTurnPoint())
               {
                  continue;
               }

               Vector2DBasics rollGradient = waypoints[j].computeRollInclineGradient(heightMapData);
               gradients[j - 1].sub(rollGradient);
               gradients[j + 1].add(rollGradient);

               if (visualize)
               {
                  if (j - 1 != 0)
                  {
                     waypoints[j - 1].updateRollGraphics(-rollGradient.getX(), -rollGradient.getY());
                  }
                  if (j + 1 != pathSize - 1)
                  {
                     waypoints[j + 1].updateRollGraphics(rollGradient.getX(), rollGradient.getY());
                  }
               }
            }
         }

         for (int j = 1; j < pathSize - 1; j++)
         {
            waypoints[j].getPosition().subX(hillClimbGain * gradients[j].getX());
            waypoints[j].getPosition().subY(hillClimbGain * gradients[j].getY());
         }

         for (int j = 1; j < pathSize - 1; j++)
         {
            updateHeading(j);
            waypoints[j].updateHeight();
         }

         if (iteration.getValue() == turnPointIteration)
         {
            computeTurnPoints();
         }

         if (visualize) // && this.iteration.getValue() % 100 == 0)
         {
            tickAndUpdatable.tickAndUpdate();
         }
      }

      List<Point3D> smoothedPath = new ArrayList<>();
      for (int i = 0; i < pathSize; i++)
      {
         smoothedPath.add(new Point3D(waypoints[i].getPosition()));
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
      double spacingGradientX = -4.0 * equalSpacingWeight * (x2 - 2.0 * x1 + x0);
      double spacingGradientY = -4.0 * equalSpacingWeight * (y2 - 2.0 * y1 + y0);
      double alphaTurnPoint = isTurnPoint ? 0.1 : 0.0;
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
         double f0 = EuclidCoreTools.square(computeDeltaHeadingMagnitude(x0, y0, x1, y1, x2, y2, minCurvatureToPenalize));
         double fPdx = EuclidCoreTools.square(computeDeltaHeadingMagnitude(x0, y0, x1 + gradientEpsilon, y1, x2, y2, minCurvatureToPenalize));
         double fPdy = EuclidCoreTools.square(computeDeltaHeadingMagnitude(x0, y0, x1, y1 + gradientEpsilon, x2, y2, minCurvatureToPenalize));
         smoothnessGradientX = smoothnessWeight * (fPdx - f0) / gradientEpsilon;
         smoothnessGradientY = smoothnessWeight * (fPdy - f0) / gradientEpsilon;
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

   public void updateHeading(int i)
   {
      double x0 = waypoints[i - 1].getPosition().getX();
      double y0 = waypoints[i - 1].getPosition().getY();
      double x1 = waypoints[i].getPosition().getX();
      double y1 = waypoints[i].getPosition().getY();
      double x2 = waypoints[i + 1].getPosition().getX();
      double y2 = waypoints[i + 1].getPosition().getY();

      double heading0 = Math.atan2(y1 - y0, x1 - x0);
      double heading1 = Math.atan2(y2 - y1, x2 - x1);
      waypoints[i].setHeading(heading0, heading1);
   }

   private void computeTurnPoints()
   {
      if (pathSize < 5)
      {
         return;
      }

      List<Pair<Double, Integer>> headingIndexList = new ArrayList<>();
      for (int i = 2; i < pathSize - 2; i++)
      {
         double deltaHeading = Math.abs(EuclidCoreTools.angleDifferenceMinusPiToPi(waypoints[i + 1].getHeading(), waypoints[i - 1].getHeading()));
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
}
