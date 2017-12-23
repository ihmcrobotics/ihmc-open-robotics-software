package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static org.junit.Assert.assertEquals;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.jgrapht.alg.DijkstraShortestPath;
import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;
import org.junit.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.Connection;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.ConnectionPoint3D;

public class JGraphToolsTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test(timeout = 10000)
   public void testRedundantPathBugTwoStages()
   {
      Random random = new Random(34667);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConnectionPoint3D start = new ConnectionPoint3D(EuclidCoreRandomTools.nextPoint3D(random, 10.0), 0);
         ConnectionPoint3D goal = new ConnectionPoint3D(EuclidCoreRandomTools.nextPoint3D(random, 10.0), 0);
         int stageSize = random.nextInt(200) + 1;

         ConnectionPoint3D[] stage1 = nextConnectionPoint3DArray(random, 10.0, stageSize);
         ConnectionPoint3D[] stage2 = nextConnectionPoint3DArray(random, 10.0, stageSize);

         int shortestPathIndex = -1;
         double minDistance = Double.POSITIVE_INFINITY;
         List<Connection> connections = new ArrayList<>();
         for (int pathIndex = 0; pathIndex < stageSize; pathIndex++)
         {
            Connection toStage1 = new Connection(start, stage1[pathIndex]);
            Connection toStage2 = new Connection(stage1[pathIndex], stage2[pathIndex]);
            Connection toGoal = new Connection(stage2[pathIndex], goal);

            double pathLength = toStage1.length() + toStage2.length() + toGoal.length();

            if (pathLength < minDistance)
            {
               minDistance = pathLength;
               shortestPathIndex = pathIndex;
            }

            connections.add(toStage1);
            connections.add(toStage2);
            connections.add(toGoal);
         }
         List<ConnectionPoint3D> expectedPath = new ArrayList<>();
         expectedPath.add(start);
         expectedPath.add(stage1[shortestPathIndex]);
         expectedPath.add(stage2[shortestPathIndex]);
         expectedPath.add(goal);
         SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> graph = JGraphTools.createGraphFromConnections(connections);
         List<DefaultWeightedEdge> solution = DijkstraShortestPath.findPathBetween(graph, start, goal);
         List<Point3DReadOnly> actualPath = JGraphTools.convertVisibilityGraphSolutionToPath(solution, start, graph);

         assertEquals("Iteration: " + i, expectedPath.size(), actualPath.size());
         for (int j = 0; j < expectedPath.size(); j++)
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedPath.get(j), actualPath.get(j), EPSILON);
      }
   }

   @Test(timeout = 10000)
   public void testRedundantPathBugRandomNumberOfStages()
   {
      Random random = new Random(34667);

      for (int i = 0; i < ITERATIONS; i++)
      {
         ConnectionPoint3D start = new ConnectionPoint3D(EuclidCoreRandomTools.nextPoint3D(random, 10.0), 0);
         ConnectionPoint3D goal = new ConnectionPoint3D(EuclidCoreRandomTools.nextPoint3D(random, 10.0), 0);
         int numberOfStages = random.nextInt(20);
         int stageSize = random.nextInt(200) + 1;

         Stage[] stages = nextStageArray(random, stageSize, numberOfStages);

         int shortestPathIndex = -1;
         double minDistance = Double.POSITIVE_INFINITY;
         List<Connection> connections = new ArrayList<>();
         for (int pathIndex = 0; pathIndex < stageSize; pathIndex++)
         {
            ConnectionPoint3D source = start;
            double pathLength = 0.0;

            for (int stageIndex = 0; stageIndex < numberOfStages; stageIndex++)
            {
               ConnectionPoint3D target = stages[stageIndex].points[pathIndex];
               Connection toStage = new Connection(source, target);
               connections.add(toStage);
               pathLength += toStage.length();
               source = target;
            }

            Connection toGoal = new Connection(source, goal);
            pathLength += toGoal.length();

            if (pathLength < minDistance)
            {
               minDistance = pathLength;
               shortestPathIndex = pathIndex;
            }

            connections.add(toGoal);
         }

         List<ConnectionPoint3D> expectedPath = new ArrayList<>();
         expectedPath.add(start);
         for (Stage stage : stages)
            expectedPath.add(stage.points[shortestPathIndex]);
         expectedPath.add(goal);

         SimpleWeightedGraph<ConnectionPoint3D, DefaultWeightedEdge> graph = JGraphTools.createGraphFromConnections(connections);
         List<DefaultWeightedEdge> solution = DijkstraShortestPath.findPathBetween(graph, start, goal);
         List<Point3DReadOnly> actualPath = JGraphTools.convertVisibilityGraphSolutionToPath(solution, start, graph);

         assertEquals("Iteration: " + i, expectedPath.size(), actualPath.size());
         for (int j = 0; j < expectedPath.size(); j++)
            EuclidCoreTestTools.assertTuple3DEquals("Iteration: " + i, expectedPath.get(j), actualPath.get(j), EPSILON);
      }
   }

   public static Stage[] nextStageArray(Random random, int stageSize, int numberOfStages)
   {
      Stage[] stages = new Stage[numberOfStages];

      for (int i = 0; i < stages.length; i++)
         stages[i] = new Stage(random, stageSize);

      return stages;
   }

   private static class Stage
   {
      private final ConnectionPoint3D[] points;

      public Stage(Random random, int size)
      {
         points = nextConnectionPoint3DArray(random, 1.0, size);
      }
   }

   private static ConnectionPoint3D[] nextConnectionPoint3DArray(Random random, double minMax, int numberOfPoints)
   {
      ConnectionPoint3D[] points = new ConnectionPoint3D[numberOfPoints];

      for (int i = 0; i < points.length; i++)
         points[i] = new ConnectionPoint3D(EuclidCoreRandomTools.nextPoint3D(random, minMax), 0);

      return points;
   }
}
