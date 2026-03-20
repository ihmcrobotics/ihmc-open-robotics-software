package us.ihmc.pathPlanning.rrt;

import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.Function;

public class RRTConnectPathPlanner
{
   private int maxIterations = 1000;
   private double searchRadius;
   private Function<LineSegment3D, Boolean> collider;
   private double stepSize = 0.2;
   private double goalTolerance = 0.1;
   private int shortcutIterations = 150;
   private final Point3D start = new Point3D();
   private final Point3D goal = new Point3D();
   private final Point3D center = new Point3D();
   private final Point3D sample = new Point3D();
   private final Vector3D direction = new Vector3D();
   private final Point3D newPosition = new Point3D();
   private final LineSegment3D collisionSegment = new LineSegment3D();
   private final Random random = new Random();
   public static class Node
   {
      public Point3D position;
      public Node parent;
      public List<Node> children;

      Node(Point3D position, Node parent, ArrayList<Node> children)
      {
         this.position = position;
         this.parent = parent;
         this.children = children;
      }
   }
   public Node rootNodeA;
   public Node rootNodeB;
   private int i;
   private int treeASize;
   private int treeBSize;
   private boolean treeAIsStart;
   private final ArrayDeque<Node> stack = new ArrayDeque<>();
   public final ArrayList<Point3D> path = new ArrayList<>();

   public List<Point3D> plan(Tuple3DReadOnly start, Tuple3DReadOnly goal, Function<LineSegment3D, Boolean> collider)
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
      searchRadius = 1.5 * start.distance(goal);
      center.interpolate(start, goal, 0.5);

      rootNodeA = new Node(new Point3D(start), null, new ArrayList<>());
      rootNodeB = new Node(new Point3D(goal), null, new ArrayList<>());
      treeASize = 1;
      treeBSize = 1;
      treeAIsStart = true;

      for (i = 0; i < maxIterations; i++)
      {
         sample();
         Node newNodeA = extend(rootNodeA, sample);
         if (newNodeA != null)
         {
            Node newNodeB = connect(rootNodeB, newNodeA.position);
            if (newNodeB != null && newNodeB.position.distance(newNodeA.position) <= goalTolerance)
            {
               buildPath(newNodeA, newNodeB, treeAIsStart);
               shortcutPath();
               return;
            }
         }

         swapTrees();
      }
   }

   private void swapTrees()
   {
      Node tempRoot = rootNodeA;
      rootNodeA = rootNodeB;
      rootNodeB = tempRoot;

      int tempSize = treeASize;
      treeASize = treeBSize;
      treeBSize = tempSize;

      treeAIsStart = !treeAIsStart;
   }

   private Node extend(Node rootNode, Point3D target)
   {
      Node closestNode = findClosestNode(rootNode, target);
      if (closestNode == null)
         return null;

      direction.sub(target, closestNode.position);
      double distance = direction.norm();
      if (distance < 1.0e-9)
         return null;
      direction.scale(Math.min(stepSize, distance) / distance);
      newPosition.add(closestNode.position, direction);

      if (!isLineCollisionFree(closestNode.position, newPosition))
         return null;

      Node newNode = new Node(new Point3D(newPosition), closestNode, new ArrayList<>());
      closestNode.children.add(newNode);
      treeASize++;
      return newNode;
   }

   private Node connect(Node rootNode, Point3D target)
   {
      Node currentNode = findClosestNode(rootNode, target);
      if (currentNode == null)
         return null;

      while (true)
      {
         direction.sub(target, currentNode.position);
         double distance = direction.norm();
         if (distance < 1.0e-9)
            return currentNode;
         direction.scale(Math.min(stepSize, distance) / distance);
         newPosition.add(currentNode.position, direction);

         if (!isLineCollisionFree(currentNode.position, newPosition))
            return null;

         Node newNode = new Node(new Point3D(newPosition), currentNode, new ArrayList<>());
         currentNode.children.add(newNode);
         currentNode = newNode;
         treeBSize++;

         if (currentNode.position.distance(target) <= goalTolerance)
            return currentNode;
      }
   }

   private Node findClosestNode(Node rootNode, Point3D target)
   {
      if (rootNode == null)
         return null;
      Node closestNode = rootNode;
      double closestDistance = rootNode.position.distance(target);
      stack.clear();
      stack.addAll(rootNode.children);
      while (!stack.isEmpty())
      {
         Node node = stack.pop();
         double distance = node.position.distance(target);
         if (distance < closestDistance)
         {
            closestNode = node;
            closestDistance = distance;
         }
         stack.addAll(node.children);
      }
      return closestNode;
   }

   private void buildPath(Node nodeFromTreeA, Node nodeFromTreeB, boolean treeAIsStart)
   {
      ArrayList<Point3D> pathFromA = pathFromNodeToRoot(nodeFromTreeA);
      ArrayList<Point3D> pathFromB = pathFromNodeToRoot(nodeFromTreeB);

      if (treeAIsStart)
      {
         reverseInPlace(pathFromA);
         path.clear();
         path.addAll(pathFromA);
         appendPathSkippingDuplicateStart(path, pathFromB);
      }
      else
      {
         reverseInPlace(pathFromB);
         path.clear();
         path.addAll(pathFromB);
         appendPathSkippingDuplicateStart(path, pathFromA);
      }
   }

   private ArrayList<Point3D> pathFromNodeToRoot(Node node)
   {
      ArrayList<Point3D> pathFromNode = new ArrayList<>();
      Node current = node;
      while (current != null)
      {
         pathFromNode.add(new Point3D(current.position));
         current = current.parent;
      }
      return pathFromNode;
   }

   private void reverseInPlace(ArrayList<Point3D> list)
   {
      for (int left = 0, right = list.size() - 1; left < right; left++, right--)
      {
         Point3D temp = list.get(left);
         list.set(left, list.get(right));
         list.set(right, temp);
      }
   }

   private void appendPathSkippingDuplicateStart(ArrayList<Point3D> destination, ArrayList<Point3D> source)
   {
      int startIndex = 0;
      if (!destination.isEmpty() && !source.isEmpty() && destination.get(destination.size() - 1).distance(source.get(0)) < 1.0e-9)
         startIndex = 1;
      for (int index = startIndex; index < source.size(); index++)
         destination.add(source.get(index));
   }

   private void shortcutPath()
   {
      if (path.size() < 3)
         return;
      for (int attempt = 0; attempt < shortcutIterations; attempt++)
      {
         if (path.size() < 3)
            return;

         int indexA = random.nextInt(path.size());
         int indexB = random.nextInt(path.size());
         if (indexA == indexB)
            continue;
         if (indexA > indexB)
         {
            int temp = indexA;
            indexA = indexB;
            indexB = temp;
         }
         if (indexB - indexA < 2)
            continue;

         Point3D startPoint = path.get(indexA);
         Point3D endPoint = path.get(indexB);
         if (isLineCollisionFree(startPoint, endPoint))
         {
            for (int index = indexB - 1; index > indexA; index--)
               path.remove(index);
         }
      }
   }

   private boolean isLineCollisionFree(Point3D startPoint, Point3D endPoint)
   {
      collisionSegment.set(startPoint, endPoint);
      return !collider.apply(collisionSegment);
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
      return rootNodeA;
   }

   public Node getRootNodeA()
   {
      return rootNodeA;
   }

   public Node getRootNodeB()
   {
      return rootNodeB;
   }

   public void setMaxIterations(int maxIterations)
   {
      this.maxIterations = maxIterations;
   }

   public int getIterationCount()
   {
      return i;
   }

   public int getTreeASize()
   {
      return treeASize;
   }

   public int getTreeBSize()
   {
      return treeBSize;
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
