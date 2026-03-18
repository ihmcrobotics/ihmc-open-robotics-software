package us.ihmc.pathPlanning.rrt;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Function;

public class RRTStarPathPlanner
{
   private int maxIterations = 20;
   private double searchRadius;
   private Function<Point3D, Boolean> collider;
   private final Point3D start = new Point3D();
   private final Point3D goal = new Point3D();
   private final Point3D center = new Point3D();
   private final Point3D sample = new Point3D();
   private final Vector3D direction = new Vector3D();
   private final Point3D newPosition = new Point3D();
   private final Random random = new Random();
   public static class Node
   {
      public Point3D position;
      public double cost;
      public Node parent;
      public List<Node> children;

      Node(Point3D position, double cost, Node parent, ArrayList<Node> children)
      {
         this.position = position;
         this.cost = cost;
         this.parent = parent;
         this.children = children;
      }
   }
   public Node rootNode;
   private int i;
   private int size;
   private final ArrayDeque<Node> stack = new ArrayDeque<>();
   private final ArrayList<Node> neighbors = new ArrayList<>();
   public final ArrayList<Point3D> path = new ArrayList<>();
   public Node bestGoalNode;

   public List<Point3D> plan(Tuple3DReadOnly start, Tuple3DReadOnly goal, Function<Point3D, Boolean> collider)
   {
      this.start.set(start);
      this.goal.set(goal);
      this.collider = collider;
      plan();
      return path;
   }

   private void plan()
   {
      path.clear();
      bestGoalNode = null;
      searchRadius = 1.5 * start.distance(goal);
      center.interpolate(start, goal, 0.5);

      rootNode = new Node(start, 0.0, null, new ArrayList<>());

      for (i = 0; i < maxIterations; i++)
      {
         sample();

         Node closestNode = rootNode;
         double closestDistance = rootNode.position.distance(sample);
         size = 1;
         stack.clear();
         stack.addAll(rootNode.children);
         while (!stack.isEmpty()) // Find closest node to sample
         {
            Node node = stack.pop();
            ++size;
            double distance = node.position.distance(sample);
            if (distance < closestDistance)
            {
               closestNode = node;
               closestDistance = distance;
            }
            stack.addAll(node.children);
         }

         direction.sub(sample, closestNode.position); // Steer
         direction.normalize();
         direction.scale(Math.min(0.2, sample.distance(closestNode.position)));
         newPosition.add(closestNode.position, direction);

         boolean atGoal = newPosition.distance(goal) < 0.1;
         if (atGoal || !collider.apply(newPosition)) // TODO: Check edge for collisions
         {
            double rMax = 0.125 * searchRadius;
            double gamma = 2.6944 * searchRadius;
            double dynamicRadius = size <= 1 ? rMax : Math.min(rMax, gamma * Math.sqrt(Math.log(size) / size)); // Avoid O(n^2)

            neighbors.clear();
            Node cheapestNeighbor = closestNode;
            double lowestCost = closestNode.cost + newPosition.distance(closestNode.position);
            stack.clear();
            stack.addAll(rootNode.children);
            while (!stack.isEmpty()) // Choose best parent
            {
               Node node = stack.pop();
               double distance = node.position.distance(newPosition);

               if (distance < dynamicRadius)
               {
                  neighbors.add(node);
                  double cost = node.cost + distance;
                  if (cost < lowestCost) // TODO: Check edge for collisions
                  {
                     cheapestNeighbor = node;
                     lowestCost = cost;
                  }
               }
               stack.addAll(node.children);
            }

            Node newNode = new Node(new Point3D(newPosition), lowestCost, cheapestNeighbor, new ArrayList<>());
            cheapestNeighbor.children.add(newNode);

            for (Node neighbor : neighbors) // Rewire
            {
               double cost = newNode.cost + newNode.position.distance(neighbor.position);
               if (neighbor.parent != null && cost < neighbor.cost) // TODO: Check edge for collisions
               {
                  neighbor.parent.children.remove(neighbor);
                  neighbor.parent = newNode;
                  neighbor.cost = cost;
                  newNode.children.add(neighbor);

                  stack.clear();
                  stack.add(neighbor);
                  while (!stack.isEmpty())
                  {
                     Node node = stack.pop();
                     for (Node child : node.children)
                     {
                        child.cost = node.cost + node.position.distance(child.position);
                        stack.add(child);
                     }
                  }
               }
            }

            if (atGoal && (bestGoalNode == null || newNode.cost < bestGoalNode.cost))
               bestGoalNode = newNode;
         }
      }

      if (bestGoalNode != null)
      {
         path.clear();
         Node node = bestGoalNode;
         while (node.parent != null)
         {
            path.add(0, node.position);
            node = node.parent;
         }
         path.add(0, node.position);
      }
   }

   private void sample() // TODO: Informed sampling. Start with entire config space, then go to ellipse with C_best
   {
      double r = searchRadius * Math.sqrt(random.nextDouble());
      double theta = 2.0 * Math.PI * random.nextDouble();
      direction.set(r * Math.cos(theta), r * Math.sin(theta), 0.0);
      sample.add(center, direction);
   }

   public Node getRootNode()
   {
      return rootNode;
   }

   public void setMaxIterations(int maxIterations)
   {
      this.maxIterations = maxIterations;
   }

   public int getIterationCount()
   {
      return i;
   }

   public int getTreeSize()
   {
      return size;
   }

   public Point3D getCenter()
   {
      return center;
   }

   public double getSearchRadius()
   {
      return searchRadius;
   }
}
