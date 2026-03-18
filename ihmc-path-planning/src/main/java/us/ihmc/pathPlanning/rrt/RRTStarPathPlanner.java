package us.ihmc.pathPlanning.rrt;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
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
   private final Point3D start = new Point3D();
   private final Point3D goal = new Point3D();
   private final Vector3D direction = new Vector3D();
   private final Random random = new Random();
   public record Node(Point3D position, List<Node> children) { }
   public Node rootNode;

   public List<Point3D> plan(Tuple3DReadOnly start, Tuple3DReadOnly goal, Function<Point3D, Boolean> collision)
   {
      this.start.set(start);
      this.goal.set(goal);

      rootNode = new Node(this.start, new ArrayList<>());

      for (int i = 0; i < 20; i++)
      {
         Point3D sample = sample();

         Node closestNode = rootNode;
         double closestDistance = rootNode.position.distance(sample);
         ArrayDeque<Node> stack = new ArrayDeque<>(rootNode.children);
         while (!stack.isEmpty())
         {
            Node currentNode = stack.pop();
            double distance = currentNode.position.distance(sample);
            if (distance < closestDistance)
            {
               closestNode = currentNode;
               closestDistance = distance;
            }
            stack.addAll(currentNode.children);
         }

         direction.sub(sample, closestNode.position);
         direction.normalize();
         direction.scale(0.2);

         Point3D newPosition = new Point3D(closestNode.position);
         newPosition.add(direction);
         if (!collision.apply(newPosition))
            closestNode.children.add(new Node(newPosition, new ArrayList<>()));

      }


      ArrayList<Point3D> path = new ArrayList<>();

      return path;
   }

   private Point3D sample()
   {
      double maxDistance = 2.0 * start.distance(goal);
      return EuclidCoreRandomTools.nextPoint3D(random, start.getX() - maxDistance, start.getX() + maxDistance,
                                                       start.getY() - maxDistance, start.getY() + maxDistance,
                                                       0.0, 0.0);
   }

   public Node getRootNode()
   {
      return rootNode;
   }
}
