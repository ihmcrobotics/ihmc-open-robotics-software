package us.ihmc.footstepPlanning.graphSearch.graph;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class DiscreteFootstepTest
{
   @Test
   public void testEqualsAndHashcode()
   {
      Random random = new Random(3823L);
      int numTrials = 100;
      DiscreteFootstep stepA, stepB;

      for (int i = 0; i < numTrials; i++)
      {
         RobotSide robotSide = RobotSide.generateRandomRobotSide(random);
         double x = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double y = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double yaw = EuclidCoreRandomTools.nextDouble(random, 1.0);

         stepA = new DiscreteFootstep(x, y, yaw, robotSide);
         stepB = new DiscreteFootstep(x, y, yaw, robotSide);

         assertTrue(stepA.equals(stepB));
         assertTrue(stepA.hashCode() == stepB.hashCode());

         stepB = new DiscreteFootstep(x + 0.1, y, yaw, robotSide);
         assertFalse(stepA.equals(stepB));
         stepB = new DiscreteFootstep(x, y + 0.1, yaw, robotSide);
         assertFalse(stepA.equals(stepB));
         stepB = new DiscreteFootstep(x, y, yaw + 0.5, robotSide);
         assertFalse(stepA.equals(stepB));
         stepB = new DiscreteFootstep(x, y, yaw, robotSide.getOppositeSide());
         assertFalse(stepA.equals(stepB));
      }
   }

   @Test
   public void testDiscreteFootstep()
   {
      double gridX = LatticePoint.gridSizeXY;
      double gridY = LatticePoint.gridSizeXY;
      DiscreteFootstep footstep;

      footstep = new DiscreteFootstep(gridX * 0.3, 0.0);
      assertEquals(0.0, footstep.getX(), 1.0e-10);
      assertEquals(0.0, footstep.getY(), 1.0e-10);
      int hash1 = footstep.hashCode();

      footstep = new DiscreteFootstep(gridX * 0.1, -gridY * 0.2);
      assertEquals(0.0, footstep.getX(), 1.0e-10);
      assertEquals(0.0, footstep.getY(), 1.0e-10);
      int hash2 = footstep.hashCode();

      assertEquals(hash1, hash2);

      footstep = new DiscreteFootstep(gridX * 0.8, 0.0);
      assertEquals(gridX, footstep.getX(), 1.0e-10);
      assertEquals(0.0, footstep.getY(), 1.0e-10);

      footstep = new DiscreteFootstep(gridX * 3.8, -gridY * 8.1);
      assertEquals(4.0 * gridX, footstep.getX(), 1.0e-10);
      assertEquals(-8.0 * gridY, footstep.getY(), 1.0e-10);
   }

   @Test
   public void testYawIndexDistance()
   {
      DiscreteFootstep n1 = new DiscreteFootstep(0, 0, 0, RobotSide.LEFT);
      DiscreteFootstep n2 = new DiscreteFootstep(0, 0, 3, RobotSide.LEFT);
      Assertions.assertEquals(n1.computeYawIndexDistance(n2), 3);
      Assertions.assertEquals(n2.computeYawIndexDistance(n1), 3);

      n1 = new DiscreteFootstep(0, 0, LatticePoint.yawDivisions - 1, RobotSide.LEFT);
      n2 = new DiscreteFootstep(0, 0, 0, RobotSide.LEFT);
      Assertions.assertEquals(n1.computeYawIndexDistance(n2), 1);
      Assertions.assertEquals(n2.computeYawIndexDistance(n1), 1);

      n1 = new DiscreteFootstep(0, 0, LatticePoint.yawDivisions - 5, RobotSide.LEFT);
      n2 = new DiscreteFootstep(0, 0, 5, RobotSide.LEFT);
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
         DiscreteFootstep n1 = DiscreteFootstep.generateRandomFootstep(random, 10.0);

         int bound = 1000;
         int dx = random.nextInt(bound) - bound / 2;
         int dy = random.nextInt(bound) - bound / 2;
         int dyaw = random.nextInt(LatticePoint.yawDivisions + 1) - LatticePoint.yawDivisions / 2;

         DiscreteFootstep n2 = new DiscreteFootstep(n1.getXIndex() + dx, n1.getYIndex() + dy, n1.getYawIndex() + dyaw, RobotSide.generateRandomRobotSide(random));

         int manhattanDistance = Math.abs(dx) + Math.abs(dy) + Math.abs(dyaw);
         Assertions.assertEquals(n1.computeManhattanDistance(n2), manhattanDistance);
         Assertions.assertEquals(n2.computeManhattanDistance(n1), manhattanDistance);
      }
   }
}
