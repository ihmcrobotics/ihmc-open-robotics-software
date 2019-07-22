package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class FootstepNodeToolsTest
{
   private final Random random = new Random(456789L);
   private static final int iters = 1000;
   private final double epsilon = 1e-8;

   @Test
   public void testGetNodeTransform()
   {
      for (int i = 0; i < iters; i++)
      {
         RobotQuadrant quadrant = RobotQuadrant.generateRandomRobotQuadrant(random);
         int frontLeftXLatticeIndex = random.nextInt(1000) - 500;
         int frontLeftYLatticeIndex = random.nextInt(1000) - 500;
         int frontRightXLatticeIndex = random.nextInt(1000) - 500;
         int frontRightYLatticeIndex = random.nextInt(1000) - 500;
         int hindLeftXLatticeIndex = random.nextInt(1000) - 500;
         int hindLeftYLatticeIndex = random.nextInt(1000) - 500;
         int hindRightXLatticeIndex = random.nextInt(1000) - 500;
         int hindRightYLatticeIndex = random.nextInt(1000) - 500;

         double frontLeftX = frontLeftXLatticeIndex * FootstepNode.gridSizeXY;
         double frontLeftY = frontLeftYLatticeIndex * FootstepNode.gridSizeXY;
         double frontRightX = frontRightXLatticeIndex * FootstepNode.gridSizeXY;
         double frontRightY = frontRightYLatticeIndex * FootstepNode.gridSizeXY;
         double hindLeftX = hindLeftXLatticeIndex * FootstepNode.gridSizeXY;
         double hindLeftY = hindLeftYLatticeIndex * FootstepNode.gridSizeXY;
         double hindRightX = hindRightXLatticeIndex * FootstepNode.gridSizeXY;
         double hindRightY = hindRightYLatticeIndex * FootstepNode.gridSizeXY;

         checkNodeTransform(quadrant, frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX, hindRightY, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0);
         checkNodeTransform(quadrant, frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX, hindRightY,
                            0.4995 * FootstepNode.gridSizeXY, 0.0, 0.4995 * FootstepNode.gridSizeXY, 0.0, 0.4995 * FootstepNode.gridSizeXY, 0.0,
                            0.4995 * FootstepNode.gridSizeXY, 0.0);
         checkNodeTransform(quadrant, frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX, hindRightY,
                            -0.4995 * FootstepNode.gridSizeXY, 0.0, -0.4995 * FootstepNode.gridSizeXY, 0.0, -0.4995 * FootstepNode.gridSizeXY, 0.0,
                            -0.4995 * FootstepNode.gridSizeXY, 0.0);
         checkNodeTransform(quadrant, frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX, hindRightY, 0.0,
                            0.4995 * FootstepNode.gridSizeXY, 0.0, 0.4995 * FootstepNode.gridSizeXY, 0.0, 0.4995 * FootstepNode.gridSizeXY, 0.0,
                            0.4995 * FootstepNode.gridSizeXY);
         checkNodeTransform(quadrant, frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX, hindRightY, 0.0,
                            -0.4995 * FootstepNode.gridSizeXY, 0.0, -0.4995 * FootstepNode.gridSizeXY, 0.0, -0.4995 * FootstepNode.gridSizeXY, 0.0,
                            -0.4995 * FootstepNode.gridSizeXY);
      }
   }

   private void checkNodeTransform(RobotQuadrant quadrant, double frontLeftX, double frontLeftY, double frontRightX, double frontRightY, double hindLeftX,
                                   double hindLeftY, double hindRightX, double hindRightY, double frontLeftXOffset, double frontLeftYOffset,
                                   double frontRightXOffset, double frontRightYOffset, double hindLeftXOffset, double hindLeftYOffset, double hindRightXOffset,
                                   double hindRightYOffset)
   {
      double length = frontLeftX + frontLeftXOffset - hindRightX - hindRightXOffset;
      double width = frontLeftY + frontLeftYOffset - hindRightY - hindRightYOffset;


      double yaw = FootstepNode.computeNominalYaw(frontLeftX + frontLeftXOffset, frontLeftY + frontLeftYOffset, frontRightX + frontRightXOffset,
                                                  frontRightY + frontRightYOffset, hindLeftX + hindLeftXOffset, hindLeftY + hindLeftYOffset,
                                                  hindRightX + hindRightXOffset, hindRightY + hindRightYOffset);

      FootstepNode node = new FootstepNode(quadrant, frontLeftX + frontLeftXOffset, frontLeftY + frontLeftYOffset, frontRightX + frontRightXOffset,
                                           frontRightY + frontRightYOffset, hindLeftX + hindLeftXOffset, hindLeftY + hindLeftYOffset,
                                           hindRightX + hindRightXOffset, hindRightY + hindRightYOffset, yaw, length, width);

      RigidBodyTransform frontLeftNodeTransform = new RigidBodyTransform();
      RigidBodyTransform frontRightNodeTransform = new RigidBodyTransform();
      RigidBodyTransform hindLeftNodeTransform = new RigidBodyTransform();
      RigidBodyTransform hindRightNodeTransform = new RigidBodyTransform();
      FootstepNodeTools.getNodeTransformToWorld(RobotQuadrant.FRONT_LEFT, node, frontLeftNodeTransform);
      FootstepNodeTools.getNodeTransformToWorld(RobotQuadrant.FRONT_RIGHT, node, frontRightNodeTransform);
      FootstepNodeTools.getNodeTransformToWorld(RobotQuadrant.HIND_LEFT, node, hindLeftNodeTransform);
      FootstepNodeTools.getNodeTransformToWorld(RobotQuadrant.HIND_RIGHT, node, hindRightNodeTransform);

      double[] frontLeftRotationYawPitchRoll = new double[3];
      double[] frontRightRotationYawPitchRoll = new double[3];
      double[] hindLeftRotationYawPitchRoll = new double[3];
      double[] hindRightRotationYawPitchRoll = new double[3];
      frontLeftNodeTransform.getRotationYawPitchRoll(frontLeftRotationYawPitchRoll);
      frontRightNodeTransform.getRotationYawPitchRoll(frontRightRotationYawPitchRoll);
      hindLeftNodeTransform.getRotationYawPitchRoll(hindLeftRotationYawPitchRoll);
      hindRightNodeTransform.getRotationYawPitchRoll(hindRightRotationYawPitchRoll);

      assertEquals(frontLeftNodeTransform.getTranslationX(), frontLeftX, epsilon);
      assertEquals(frontLeftNodeTransform.getTranslationY(), frontLeftY, epsilon);
      assertEquals(frontLeftNodeTransform.getTranslationZ(), 0.0, epsilon);
      assertEquals(frontLeftRotationYawPitchRoll[1], 0.0, epsilon);
      assertEquals(frontLeftRotationYawPitchRoll[2], 0.0, epsilon);

      assertEquals(frontRightNodeTransform.getTranslationX(), frontRightX, epsilon);
      assertEquals(frontRightNodeTransform.getTranslationY(), frontRightY, epsilon);
      assertEquals(frontRightNodeTransform.getTranslationZ(), 0.0, epsilon);
      assertEquals(frontRightRotationYawPitchRoll[1], 0.0, epsilon);
      assertEquals(frontRightRotationYawPitchRoll[2], 0.0, epsilon);

      assertEquals(hindLeftNodeTransform.getTranslationX(), hindLeftX, epsilon);
      assertEquals(hindLeftNodeTransform.getTranslationY(), hindLeftY, epsilon);
      assertEquals(hindLeftNodeTransform.getTranslationZ(), 0.0, epsilon);
      assertEquals(hindLeftRotationYawPitchRoll[1], 0.0, epsilon);
      assertEquals(hindLeftRotationYawPitchRoll[2], 0.0, epsilon);

      assertEquals(hindRightNodeTransform.getTranslationX(), hindRightX, epsilon);
      assertEquals(hindRightNodeTransform.getTranslationY(), hindRightY, epsilon);
      assertEquals(hindRightNodeTransform.getTranslationZ(), 0.0, epsilon);
      assertEquals(hindRightRotationYawPitchRoll[1], 0.0, epsilon);
      assertEquals(hindRightRotationYawPitchRoll[2], 0.0, epsilon);
   }

   @Test
   public void testGetSnappedNodeTransform()
   {
      int numTests = 10;

      for (int i = 0; i < numTests; i++)
      {
         RobotQuadrant robotQuadrant = RobotQuadrant.generateRandomRobotQuadrant(random);
         double frontLeftX = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double frontLeftY = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double frontRightX = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double frontRightY = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double hindLeftX = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double hindLeftY = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double hindRightX = EuclidCoreRandomTools.nextDouble(random, 1.0);
         double hindRightY = EuclidCoreRandomTools.nextDouble(random, 1.0);

         double length = frontLeftX - hindRightX;
         double width = frontLeftY - hindRightY;


         double yaw = FootstepNode.computeNominalYaw(frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX, hindRightY);

         FootstepNode node = new FootstepNode(robotQuadrant, frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX, hindRightY,
                                              yaw, length, width);

         RigidBodyTransform frontLeftSnapTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform frontRightSnapTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform hindLeftSnapTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform hindRightSnapTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform frontLeftSnappedNodeTransform = new RigidBodyTransform();
         RigidBodyTransform frontRightSnappedNodeTransform = new RigidBodyTransform();
         RigidBodyTransform hindLeftSnappedNodeTransform = new RigidBodyTransform();
         RigidBodyTransform hindRightSnappedNodeTransform = new RigidBodyTransform();
         FootstepNodeTools.getSnappedNodeTransformToWorld(RobotQuadrant.FRONT_LEFT, node, frontLeftSnapTransform, frontLeftSnappedNodeTransform);
         FootstepNodeTools.getSnappedNodeTransformToWorld(RobotQuadrant.FRONT_RIGHT, node, frontRightSnapTransform, frontRightSnappedNodeTransform);
         FootstepNodeTools.getSnappedNodeTransformToWorld(RobotQuadrant.HIND_LEFT, node, hindLeftSnapTransform, hindLeftSnappedNodeTransform);
         FootstepNodeTools.getSnappedNodeTransformToWorld(RobotQuadrant.HIND_RIGHT, node, hindRightSnapTransform, hindRightSnappedNodeTransform);

         RigidBodyTransform frontLeftExpectedSnappedNodeTransform = new RigidBodyTransform();
         RigidBodyTransform frontRightExpectedSnappedNodeTransform = new RigidBodyTransform();
         RigidBodyTransform hindLeftExpectedSnappedNodeTransform = new RigidBodyTransform();
         RigidBodyTransform hindRightExpectedSnappedNodeTransform = new RigidBodyTransform();
         FootstepNodeTools.getNodeTransformToWorld(RobotQuadrant.FRONT_LEFT, node, frontLeftExpectedSnappedNodeTransform);
         FootstepNodeTools.getNodeTransformToWorld(RobotQuadrant.FRONT_RIGHT, node, frontRightExpectedSnappedNodeTransform);
         FootstepNodeTools.getNodeTransformToWorld(RobotQuadrant.HIND_LEFT, node, hindLeftExpectedSnappedNodeTransform);
         FootstepNodeTools.getNodeTransformToWorld(RobotQuadrant.HIND_RIGHT, node, hindRightExpectedSnappedNodeTransform);
         frontLeftSnapTransform.transform(frontLeftExpectedSnappedNodeTransform);
         frontRightSnapTransform.transform(frontRightExpectedSnappedNodeTransform);
         hindLeftSnapTransform.transform(hindLeftExpectedSnappedNodeTransform);
         hindRightSnapTransform.transform(hindRightExpectedSnappedNodeTransform);

         EuclidCoreTestTools.assertRigidBodyTransformEquals(frontLeftExpectedSnappedNodeTransform, frontLeftSnappedNodeTransform, epsilon);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(frontRightExpectedSnappedNodeTransform, frontRightSnappedNodeTransform, epsilon);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(hindLeftExpectedSnappedNodeTransform, hindLeftSnappedNodeTransform, epsilon);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(hindRightExpectedSnappedNodeTransform, hindRightSnappedNodeTransform, epsilon);
      }
   }
}
