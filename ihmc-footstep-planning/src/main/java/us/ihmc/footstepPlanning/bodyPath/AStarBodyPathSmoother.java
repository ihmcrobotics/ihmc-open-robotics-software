package us.ihmc.footstepPlanning.bodyPath;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.YoVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.List;

public class AStarBodyPathSmoother
{
   static final double collisionWeight = 700.0;
   static final double smoothnessWeight = 1.0;
   static final double equalSpacingWeight = 2.0;
   static final double rollWeight = 20.0;

   private static final int maxPoints = 200;
   private static final double gradientEpsilon = 1e-6;
   private static final double hillClimbGainSingleWaypoint = 0.007;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoInteger iteration = new YoInteger("iterations", registry);
   private final YoDouble maxCollision = new YoDouble("maxCollision", registry);
   private final TickAndUpdatable tickAndUpdatable;
   private final boolean visualize;

   private double hillClimbGain;
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

      if (parentRegistry == null)
      {
         visualize = false;
         this.tickAndUpdatable = null;
      }
      else
      {
         for (int i = 0; i < maxPoints; i++)
         {
            waypoints[i] = new AStarBodyPathSmootherWaypoint(i, surfaceNormalCalculator, graphicsListRegistry, registry);
         }
         for (int i = 1; i < maxPoints - 1; i++)
         {
            waypoints[i].setNeighbors(waypoints[i - 1], waypoints[i + 1]);
         }

         this.tickAndUpdatable = tickAndUpdatable;
         parentRegistry.addChild(registry);
         visualize = true;
      }
   }

   public List<Point3D> doSmoothing(List<Point3D> bodyPath, HeightMapData heightMapData)
   {
      initialBodyPath = bodyPath;
      pathSize = bodyPath.size();
      hillClimbGain = hillClimbGainSingleWaypoint / pathSize;

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
         double patchWidth = 0.4;
         surfaceNormalCalculator.computeSurfaceNormals(heightMapData, patchWidth);
      }

      for (int i = 0; i < maxPoints; i++)
      {
         waypoints[i].initialize(bodyPath);
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

      int maxIterations = 3000;
      for (iteration.set(0); iteration.getValue() < maxIterations; iteration.increment())
      {
         maxCollision.set(0.0);

         // TODO just compute cost delta from the 3 waypoints affected

         for (int j = 1; j < pathSize - 1; j++)
         {
            updateHeading(j);
         }

         double fx = computeCost(0,0.0, 0.0);
         for (int j = 1; j < pathSize - 1; j++)
         {
            gradients[j].setX((computeCost(j, gradientEpsilon, 0.0) - fx) / gradientEpsilon);
            gradients[j].setY((computeCost(j, 0.0, gradientEpsilon) - fx) / gradientEpsilon);
         }

         if (heightMapData != null)
         {
            maxCollision.set(0.0);
            for (int j = 1; j < pathSize - 1; j++)
            {
               Vector2D collisionGradient = waypoints[j].computeCollisionGradient(heightMapData);
               gradients[j].add(collisionGradient);
               maxCollision.set(Math.max(waypoints[j].getMaxCollision(), maxCollision.getValue()));

               Vector2DBasics rollGradient = waypoints[j].computeRollInclineGradient(heightMapData);

               gradients[j - 1].sub(rollGradient);
               gradients[j + 1].add(rollGradient);
            }
         }

         for (int j = 1; j < pathSize - 1; j++)
         {
            waypoints[j].getPosition().subX(hillClimbGain * gradients[j].getX());
            waypoints[j].getPosition().subY(hillClimbGain * gradients[j].getY());
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

   private double computeCost(int waypointIndex, double offsetX, double offsetY)
   {
      double displacementCost = 0.0;
      double curvatureCost = 0.0;

      for (int i = 1; i < pathSize - 1; i++)
      {
         double x0 = waypoints[i - 1].getPosition().getX();
         double y0 = waypoints[i - 1].getPosition().getY();
         double x1 = waypoints[i].getPosition().getX() + (i == waypointIndex ? offsetX : 0.0);
         double y1 = waypoints[i].getPosition().getY() + (i == waypointIndex ? offsetY : 0.0);
         double x2 = waypoints[i + 1].getPosition().getX();
         double y2 = waypoints[i + 1].getPosition().getY();

         double displacement0 = EuclidCoreTools.norm(x1 - x0, y1 - y0);
         double displacement1 = EuclidCoreTools.norm(x2 - x1, y2 - y1);
         displacementCost += equalSpacingWeight * EuclidCoreTools.square(displacement1 - displacement0);

         double heading0 = Math.atan2(y1 - y0, x1 - x0);
         double heading1 = Math.atan2(y2 - y1, x2 - x1);
         double deltaHeading = EuclidCoreTools.angleDifferenceMinusPiToPi(heading1, heading0);

         curvatureCost += smoothnessWeight * EuclidCoreTools.square(deltaHeading);
      }

      return displacementCost + curvatureCost;
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
}
