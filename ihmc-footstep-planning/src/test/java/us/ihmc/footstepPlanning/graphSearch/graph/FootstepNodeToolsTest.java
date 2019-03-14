package us.ihmc.footstepPlanning.graphSearch.graph;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class FootstepNodeToolsTest
{
   private final Random random = new Random(456789L);
   private final double epsilon = 1e-8;

   @Test
   public void testGetNodeTransform()
   {
      int numTests = 1000;

      for (int i = 0; i < numTests; i++)
      {
         int xLatticeIndex = random.nextInt(1000) - 500;
         int yLatticeIndex = random.nextInt(1000) - 500;
         int yawLatticeIndex = random.nextInt(100) - 50;

         double x = xLatticeIndex * LatticeNode.gridSizeXY;
         double y = yLatticeIndex * LatticeNode.gridSizeXY;
         double yaw = AngleTools.trimAngleMinusPiToPi(yawLatticeIndex * LatticeNode.gridSizeYaw);
         RobotSide robotSide = RobotSide.generateRandomRobotSide(random);

         checkNodeTransform(x, y, yaw, robotSide, 0.0, 0.0, 0.0);
         checkNodeTransform(x, y, yaw, robotSide, 0.4995 * LatticeNode.gridSizeXY, 0.0, 0.0);
         checkNodeTransform(x, y, yaw, robotSide, -0.4995 * LatticeNode.gridSizeXY, 0.0, 0.0);
         checkNodeTransform(x, y, yaw, robotSide, 0.0, 0.4995 * LatticeNode.gridSizeXY, 0.0);
         checkNodeTransform(x, y, yaw, robotSide, 0.0, -0.4995 * LatticeNode.gridSizeXY, 0.0);
         checkNodeTransform(x, y, yaw, robotSide, 0.0, 0.0, 0.4995 * LatticeNode.gridSizeYaw);
         checkNodeTransform(x, y, yaw, robotSide, 0.0, 0.0, -0.4995 * LatticeNode.gridSizeYaw);
      }
   }

   private void checkNodeTransform(double x, double y, double yaw, RobotSide robotSide, double xOffset, double yOffset, double yawOffset)
   {
      FootstepNode node = new FootstepNode(x + xOffset, y + yOffset, yaw + yawOffset, robotSide);

      RigidBodyTransform nodeTransform = new RigidBodyTransform();
      FootstepNodeTools.getNodeTransform(node, nodeTransform);

      double[] rotationYawPitchRoll = new double[3];
      nodeTransform.getRotationYawPitchRoll(rotationYawPitchRoll);

      assertEquals(nodeTransform.getTranslationX(), x, epsilon);
      assertEquals(nodeTransform.getTranslationY(), y, epsilon);
      assertEquals(nodeTransform.getTranslationZ(), 0.0, epsilon);
      assertEquals(AngleTools.trimAngleMinusPiToPi(rotationYawPitchRoll[0] - yaw), 0.0, epsilon);
      assertEquals(rotationYawPitchRoll[1], 0.0, epsilon);
      assertEquals(rotationYawPitchRoll[2], 0.0, epsilon);
   }

   @Test
   public void testGetSnappedNodeTransform()
   {
      int numTests = 10;

      for (int i = 0; i < numTests; i++)
      {
         double x = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double y = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double yaw = EuclidCoreRandomTools.nextDouble(random, 4.0);
         RobotSide robotSide = RobotSide.generateRandomRobotSide(random);

         FootstepNode node = new FootstepNode(x, y, yaw, robotSide);
         RigidBodyTransform snapTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform snappedNodeTransform = new RigidBodyTransform();
         FootstepNodeTools.getSnappedNodeTransform(node, snapTransform, snappedNodeTransform);

         RigidBodyTransform expectedSnappedNodeTransform = new RigidBodyTransform();
         FootstepNodeTools.getNodeTransform(node, expectedSnappedNodeTransform);
         snapTransform.transform(expectedSnappedNodeTransform);

         assertTrue(expectedSnappedNodeTransform.epsilonEquals(snappedNodeTransform, epsilon));
      }
   }
}
