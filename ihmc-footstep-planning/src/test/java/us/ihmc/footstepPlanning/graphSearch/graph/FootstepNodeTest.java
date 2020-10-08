package us.ihmc.footstepPlanning.graphSearch.graph;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class FootstepNodeTest
{
   @Test
   public void testEqualsAndHashcode()
   {
      Random random = new Random(3823L);
      int numTrials = 100;
      FootstepNode nodeA, nodeB;

      for (int i = 0; i < numTrials; i++)
      {
         RobotSide robotSide = RobotSide.generateRandomRobotSide(random);
         double x = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double y = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double yaw = EuclidCoreRandomTools.nextDouble(random, 1.0);

         nodeA = new FootstepNode(x, y, yaw, robotSide);
         nodeB = new FootstepNode(x, y, yaw, robotSide);

         assertTrue(nodeA.equals(nodeB));
         assertTrue(nodeA.hashCode() == nodeB.hashCode());

         nodeB = new FootstepNode(x + 0.1, y, yaw, robotSide);
         assertFalse(nodeA.equals(nodeB));
         nodeB = new FootstepNode(x, y + 0.1, yaw, robotSide);
         assertFalse(nodeA.equals(nodeB));
         nodeB = new FootstepNode(x, y, yaw + 0.5, robotSide);
         assertFalse(nodeA.equals(nodeB));
         nodeB = new FootstepNode(x, y, yaw, robotSide.getOppositeSide());
         assertFalse(nodeA.equals(nodeB));
      }
   }

   @Test
   public void testFootstepNode()
   {
      double gridX = LatticeNode.gridSizeXY;
      double gridY = LatticeNode.gridSizeXY;
      FootstepNode node;

      node = new FootstepNode(gridX * 0.3, 0.0);
      assertEquals(0.0, node.getX(), 1.0e-10);
      assertEquals(0.0, node.getY(), 1.0e-10);
      int hash1 = node.hashCode();

      node = new FootstepNode(gridX * 0.1, -gridY * 0.2);
      assertEquals(0.0, node.getX(), 1.0e-10);
      assertEquals(0.0, node.getY(), 1.0e-10);
      int hash2 = node.hashCode();

      assertEquals(hash1, hash2);

      node = new FootstepNode(gridX * 0.8, 0.0);
      assertEquals(gridX, node.getX(), 1.0e-10);
      assertEquals(0.0, node.getY(), 1.0e-10);

      node = new FootstepNode(gridX * 3.8, -gridY * 8.1);
      assertEquals(4.0 * gridX, node.getX(), 1.0e-10);
      assertEquals(-8.0 * gridY, node.getY(), 1.0e-10);
   }

   @Test
   public void testYawIndexDistance()
   {
      FootstepNode n1 = new FootstepNode(0, 0, 0, RobotSide.LEFT);
      FootstepNode n2 = new FootstepNode(0, 0, 3, RobotSide.LEFT);
      Assertions.assertEquals(n1.computeYawIndexDistance(n2), 3);
      Assertions.assertEquals(n2.computeYawIndexDistance(n1), 3);

      n1 = new FootstepNode(0, 0, LatticeNode.yawDivisions - 1, RobotSide.LEFT);
      n2 = new FootstepNode(0, 0, 0, RobotSide.LEFT);
      Assertions.assertEquals(n1.computeYawIndexDistance(n2), 1);
      Assertions.assertEquals(n2.computeYawIndexDistance(n1), 1);

      n1 = new FootstepNode(0, 0, LatticeNode.yawDivisions - 5, RobotSide.LEFT);
      n2 = new FootstepNode(0, 0, 5, RobotSide.LEFT);
      Assertions.assertEquals(n1.computeYawIndexDistance(n2), 10);
      Assertions.assertEquals(n2.computeYawIndexDistance(n1), 10);
   }

   @Test
   public void testManhattanDistance()
   {
      Random random = new Random(32980L);

      int tests = 100000;
      for (int i = 0; i < tests; i++)
      {
         FootstepNode n1 = FootstepNode.generateRandomFootstepNode(random, 10.0);

         int bound = 1000;
         int dx = random.nextInt(bound) - bound / 2;
         int dy = random.nextInt(bound) - bound / 2;
         int dyaw = random.nextInt(LatticeNode.yawDivisions + 1) - LatticeNode.yawDivisions / 2;

         FootstepNode n2 = new FootstepNode(n1.getXIndex() + dx, n1.getYIndex() + dy, n1.getYawIndex() + dyaw, RobotSide.generateRandomRobotSide(random));

         int manhattanDistance = Math.abs(dx) + Math.abs(dy) + Math.abs(dyaw);
         Assertions.assertEquals(n1.computeManhattanDistance(n2), manhattanDistance);
         Assertions.assertEquals(n2.computeManhattanDistance(n1), manhattanDistance);
      }
   }
}
