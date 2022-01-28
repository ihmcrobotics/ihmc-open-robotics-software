package us.ihmc.footstepPlanning.bodyPath;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.simulationconstructionset.util.TickAndUpdatable;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class AStarBodyPathSmoother
{
   private static final int maxPoints = 200;
   private static final double maxCurvatureToPenalize = 0.0;
   private static final double gradientEpsilon = 1e-6;
   private static final double hillClimbGainSingleWaypoint = 0.006;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoInteger iterations = new YoInteger("iterations", registry);
   private final TickAndUpdatable tickAndUpdatable;
   private final boolean visualize;

   private double hillClimbGain;
   private final List<YoFramePoint3D> yoWaypoints = new ArrayList<>();
   private int pathSize;

   /* Vector to optimize */
   private double[] x;
   private double[] gradient;

   private final Point3D start = new Point3D();
   private final Point3D goal = new Point3D();

   public AStarBodyPathSmoother()
   {
      this(null, null, null);
   }

   public AStarBodyPathSmoother(TickAndUpdatable tickAndUpdatable, YoGraphicsListRegistry graphicsListRegistry, YoRegistry parentRegistry)
   {
      if (parentRegistry == null)
      {
         visualize = false;
         this.tickAndUpdatable = null;
      }
      else
      {
         for (int i = 0; i < maxPoints; i++)
         {
            yoWaypoints.add(new YoFramePoint3D("waypoint" + i, ReferenceFrame.getWorldFrame(), registry));
         }

         this.tickAndUpdatable = tickAndUpdatable;
         YoGraphicsList graphicsList = new YoGraphicsList(getClass().getSimpleName());
         for (int i = 0; i < maxPoints; i++)
         {
            YoGraphicPosition waypointGraphic = new YoGraphicPosition("waypointViz" + i, yoWaypoints.get(i), 0.0075, YoAppearance.Red());
            graphicsList.add(waypointGraphic);
         }

         graphicsListRegistry.registerYoGraphicsList(graphicsList);
         parentRegistry.addChild(registry);
         visualize = true;
      }
   }

   public void doSmoothing(List<BodyPathLatticePoint> bodyPath)
   {
      pathSize = bodyPath.size();
      hillClimbGain = hillClimbGainSingleWaypoint / pathSize;

      int variablesToOptimize = 2 * (pathSize - 2);
      x = new double[variablesToOptimize];
      gradient = new double[variablesToOptimize];

      double startX = bodyPath.get(0).getX();
      double startY = bodyPath.get(0).getY();
      double goalX = bodyPath.get(pathSize - 1).getX();
      double goalY = bodyPath.get(pathSize - 1).getY();

      start.set(startX, startY, 0.0);
      goal.set(goalX, goalY, 0.0);

      for (int i = 1; i < pathSize - 1; i++)
      {
         double x = bodyPath.get(i).getX();
         double y = bodyPath.get(i).getY();
         this.x[2*(i - 1)] = x;
         this.x[2*(i - 1) + 1] = y;
      }

      if (visualize)
      {
         yoWaypoints.get(0).set(start);
         yoWaypoints.get(pathSize - 1).set(goal);
         updatePoints();
         for (int i = pathSize; i < maxPoints; i++)
         {
            yoWaypoints.get(i).setToNaN();
         }
         iterations.set(-1);

         tickAndUpdatable.tickAndUpdate();
      }

      int iterations = 100;
      for (int i = 0; i < iterations; i++)
      {
         this.iterations.set(i);
         double fx = computeCost(x);

         for (int j = 0; j < x.length; j++)
         {
            double[] xj = Arrays.copyOf(x, x.length);
            xj[j] += gradientEpsilon;
            gradient[j] = (computeCost(xj) - fx) / gradientEpsilon;
         }

         for (int j = 0; j < x.length; j++)
         {
            x[j] -= hillClimbGain * gradient[j];
         }

         if (visualize)
         {
            updatePoints();
            tickAndUpdatable.tickAndUpdate();
         }
      }
   }

   private void updatePoints()
   {
      for (int j = 1; j < pathSize - 1; j++)
      {
         yoWaypoints.get(j).set(getX(j, x), getY(j, x), 0.0);
      }
   }

   private double computeCost(double[] x)
   {
      double displacementCost = 0.0;
      double curvatureCost = 0.0;

      for (int i = 1; i < pathSize - 1; i++)
      {
         double x0 = getX(i - 1, x);
         double y0 = getY(i - 1, x);
         double x1 = getX(i, x);
         double y1 = getY(i, x);
         double x2 = getX(i + 1, x);
         double y2 = getY(i + 1, x);

         double displacement0 = EuclidCoreTools.norm(x1 - x0, y1 - y0);
         double displacement1 = EuclidCoreTools.norm(x2 - x1, y2 - y1);
         displacementCost += EuclidCoreTools.square(displacement1 - displacement0);

         double heading0 = Math.atan2(y1 - y0, x1 - x0);
         double heading1 = Math.atan2(y2 - y1, x2 - x1);
         curvatureCost += EuclidCoreTools.square(EuclidCoreTools.angleDifferenceMinusPiToPi(heading0, heading1));
      }

      return displacementCost + curvatureCost;
   }

   private double getX(int waypointIndex, double[] x)
   {
      if (waypointIndex == 0)
      {
         return start.getX();
      }
      else if (waypointIndex == pathSize - 1)
      {
         return goal.getX();
      }
      else
      {
         return x[2 * (waypointIndex - 1)];
      }
   }

   private double getY(int waypointIndex, double[] x)
   {
      if (waypointIndex == 0)
      {
         return start.getY();
      }
      else if (waypointIndex == pathSize - 1)
      {
         return goal.getY();
      }
      else
      {
         return x[2 * (waypointIndex - 1) + 1];
      }
   }
}
