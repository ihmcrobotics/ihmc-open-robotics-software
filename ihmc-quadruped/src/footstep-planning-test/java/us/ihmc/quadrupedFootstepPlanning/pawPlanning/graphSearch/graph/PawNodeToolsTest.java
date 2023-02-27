package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class PawNodeToolsTest
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

         double frontLeftX = frontLeftXLatticeIndex * PawNode.gridSizeXY;
         double frontLeftY = frontLeftYLatticeIndex * PawNode.gridSizeXY;
         double frontRightX = frontRightXLatticeIndex * PawNode.gridSizeXY;
         double frontRightY = frontRightYLatticeIndex * PawNode.gridSizeXY;
         double hindLeftX = hindLeftXLatticeIndex * PawNode.gridSizeXY;
         double hindLeftY = hindLeftYLatticeIndex * PawNode.gridSizeXY;
         double hindRightX = hindRightXLatticeIndex * PawNode.gridSizeXY;
         double hindRightY = hindRightYLatticeIndex * PawNode.gridSizeXY;

         checkNodeTransform(quadrant,
                            frontLeftX,
                            frontLeftY,
                            frontRightX,
                            frontRightY,
                            hindLeftX,
                            hindLeftY,
                            hindRightX,
                            hindRightY,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0);
         checkNodeTransform(quadrant,
                            frontLeftX,
                            frontLeftY,
                            frontRightX,
                            frontRightY,
                            hindLeftX,
                            hindLeftY,
                            hindRightX,
                            hindRightY,
                            0.4995 * PawNode.gridSizeXY,
                            0.0,
                            0.4995 * PawNode.gridSizeXY,
                            0.0,
                            0.4995 * PawNode.gridSizeXY,
                            0.0,
                            0.4995 * PawNode.gridSizeXY,
                            0.0);
         checkNodeTransform(quadrant,
                            frontLeftX,
                            frontLeftY,
                            frontRightX,
                            frontRightY,
                            hindLeftX,
                            hindLeftY,
                            hindRightX,
                            hindRightY,
                            -0.4995 * PawNode.gridSizeXY,
                            0.0,
                            -0.4995 * PawNode.gridSizeXY,
                            0.0,
                            -0.4995 * PawNode.gridSizeXY,
                            0.0,
                            -0.4995 * PawNode.gridSizeXY,
                            0.0);
         checkNodeTransform(quadrant,
                            frontLeftX,
                            frontLeftY,
                            frontRightX,
                            frontRightY,
                            hindLeftX,
                            hindLeftY,
                            hindRightX,
                            hindRightY,
                            0.0,
                            0.4995 * PawNode.gridSizeXY,
                            0.0,
                            0.4995 * PawNode.gridSizeXY,
                            0.0,
                            0.4995 * PawNode.gridSizeXY,
                            0.0,
                            0.4995 * PawNode.gridSizeXY);
         checkNodeTransform(quadrant,
                            frontLeftX,
                            frontLeftY,
                            frontRightX,
                            frontRightY,
                            hindLeftX,
                            hindLeftY,
                            hindRightX,
                            hindRightY,
                            0.0,
                            -0.4995 * PawNode.gridSizeXY,
                            0.0,
                            -0.4995 * PawNode.gridSizeXY,
                            0.0,
                            -0.4995 * PawNode.gridSizeXY,
                            0.0,
                            -0.4995 * PawNode.gridSizeXY);
      }
   }

   private void checkNodeTransform(RobotQuadrant quadrant, double frontLeftX, double frontLeftY, double frontRightX, double frontRightY, double hindLeftX,
                                   double hindLeftY, double hindRightX, double hindRightY, double frontLeftXOffset, double frontLeftYOffset,
                                   double frontRightXOffset, double frontRightYOffset, double hindLeftXOffset, double hindLeftYOffset, double hindRightXOffset,
                                   double hindRightYOffset)
   {
      double length = frontLeftX + frontLeftXOffset - hindRightX - hindRightXOffset;
      double width = frontLeftY + frontLeftYOffset - hindRightY - hindRightYOffset;

      double yaw = PawNode.computeNominalYaw(frontLeftX + frontLeftXOffset,
                                             frontLeftY + frontLeftYOffset,
                                             frontRightX + frontRightXOffset,
                                             frontRightY + frontRightYOffset,
                                             hindLeftX + hindLeftXOffset,
                                             hindLeftY + hindLeftYOffset,
                                             hindRightX + hindRightXOffset,
                                             hindRightY + hindRightYOffset);

      PawNode node = new PawNode(quadrant,
                                 frontLeftX + frontLeftXOffset,
                                 frontLeftY + frontLeftYOffset,
                                 frontRightX + frontRightXOffset,
                                 frontRightY + frontRightYOffset,
                                 hindLeftX + hindLeftXOffset,
                                 hindLeftY + hindLeftYOffset,
                                 hindRightX + hindRightXOffset,
                                 hindRightY + hindRightYOffset,
                                 yaw,
                                 length,
                                 width);

      RigidBodyTransform frontLeftNodeTransform = new RigidBodyTransform();
      RigidBodyTransform frontRightNodeTransform = new RigidBodyTransform();
      RigidBodyTransform hindLeftNodeTransform = new RigidBodyTransform();
      RigidBodyTransform hindRightNodeTransform = new RigidBodyTransform();
      PawNodeTools.getNodeTransformToWorld(RobotQuadrant.FRONT_LEFT, node, frontLeftNodeTransform);
      PawNodeTools.getNodeTransformToWorld(RobotQuadrant.FRONT_RIGHT, node, frontRightNodeTransform);
      PawNodeTools.getNodeTransformToWorld(RobotQuadrant.HIND_LEFT, node, hindLeftNodeTransform);
      PawNodeTools.getNodeTransformToWorld(RobotQuadrant.HIND_RIGHT, node, hindRightNodeTransform);

      YawPitchRoll frontLeftRotationYawPitchRoll = new YawPitchRoll();
      YawPitchRoll frontRightRotationYawPitchRoll = new YawPitchRoll();
      YawPitchRoll hindLeftRotationYawPitchRoll = new YawPitchRoll();
      YawPitchRoll hindRightRotationYawPitchRoll = new YawPitchRoll();
      frontLeftRotationYawPitchRoll.set(frontLeftNodeTransform.getRotation());
      frontRightRotationYawPitchRoll.set(frontRightNodeTransform.getRotation());
      hindLeftRotationYawPitchRoll.set(hindLeftNodeTransform.getRotation());
      hindRightRotationYawPitchRoll.set(hindRightNodeTransform.getRotation());

      assertEquals(frontLeftNodeTransform.getTranslationX(), frontLeftX, epsilon);
      assertEquals(frontLeftNodeTransform.getTranslationY(), frontLeftY, epsilon);
      assertEquals(frontLeftNodeTransform.getTranslationZ(), 0.0, epsilon);
      assertEquals(frontLeftRotationYawPitchRoll.getPitch(), 0.0, epsilon);
      assertEquals(frontLeftRotationYawPitchRoll.getRoll(), 0.0, epsilon);

      assertEquals(frontRightNodeTransform.getTranslationX(), frontRightX, epsilon);
      assertEquals(frontRightNodeTransform.getTranslationY(), frontRightY, epsilon);
      assertEquals(frontRightNodeTransform.getTranslationZ(), 0.0, epsilon);
      assertEquals(frontRightRotationYawPitchRoll.getPitch(), 0.0, epsilon);
      assertEquals(frontRightRotationYawPitchRoll.getRoll(), 0.0, epsilon);

      assertEquals(hindLeftNodeTransform.getTranslationX(), hindLeftX, epsilon);
      assertEquals(hindLeftNodeTransform.getTranslationY(), hindLeftY, epsilon);
      assertEquals(hindLeftNodeTransform.getTranslationZ(), 0.0, epsilon);
      assertEquals(hindLeftRotationYawPitchRoll.getPitch(), 0.0, epsilon);
      assertEquals(hindLeftRotationYawPitchRoll.getRoll(), 0.0, epsilon);

      assertEquals(hindRightNodeTransform.getTranslationX(), hindRightX, epsilon);
      assertEquals(hindRightNodeTransform.getTranslationY(), hindRightY, epsilon);
      assertEquals(hindRightNodeTransform.getTranslationZ(), 0.0, epsilon);
      assertEquals(hindRightRotationYawPitchRoll.getPitch(), 0.0, epsilon);
      assertEquals(hindRightRotationYawPitchRoll.getRoll(), 0.0, epsilon);
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

         double yaw = PawNode.computeNominalYaw(frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX, hindRightY);

         PawNode node = new PawNode(robotQuadrant,
                                    frontLeftX,
                                    frontLeftY,
                                    frontRightX,
                                    frontRightY,
                                    hindLeftX,
                                    hindLeftY,
                                    hindRightX,
                                    hindRightY,
                                    yaw,
                                    length,
                                    width);

         RigidBodyTransform frontLeftSnapTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform frontRightSnapTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform hindLeftSnapTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         RigidBodyTransform hindRightSnapTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

         RigidBodyTransform frontLeftSnappedNodeTransform = new RigidBodyTransform();
         RigidBodyTransform frontRightSnappedNodeTransform = new RigidBodyTransform();
         RigidBodyTransform hindLeftSnappedNodeTransform = new RigidBodyTransform();
         RigidBodyTransform hindRightSnappedNodeTransform = new RigidBodyTransform();
         PawNodeTools.getSnappedNodeTransformToWorld(RobotQuadrant.FRONT_LEFT, node, frontLeftSnapTransform, frontLeftSnappedNodeTransform);
         PawNodeTools.getSnappedNodeTransformToWorld(RobotQuadrant.FRONT_RIGHT, node, frontRightSnapTransform, frontRightSnappedNodeTransform);
         PawNodeTools.getSnappedNodeTransformToWorld(RobotQuadrant.HIND_LEFT, node, hindLeftSnapTransform, hindLeftSnappedNodeTransform);
         PawNodeTools.getSnappedNodeTransformToWorld(RobotQuadrant.HIND_RIGHT, node, hindRightSnapTransform, hindRightSnappedNodeTransform);

         RigidBodyTransform frontLeftExpectedSnappedNodeTransform = new RigidBodyTransform();
         RigidBodyTransform frontRightExpectedSnappedNodeTransform = new RigidBodyTransform();
         RigidBodyTransform hindLeftExpectedSnappedNodeTransform = new RigidBodyTransform();
         RigidBodyTransform hindRightExpectedSnappedNodeTransform = new RigidBodyTransform();
         PawNodeTools.getNodeTransformToWorld(RobotQuadrant.FRONT_LEFT, node, frontLeftExpectedSnappedNodeTransform);
         PawNodeTools.getNodeTransformToWorld(RobotQuadrant.FRONT_RIGHT, node, frontRightExpectedSnappedNodeTransform);
         PawNodeTools.getNodeTransformToWorld(RobotQuadrant.HIND_LEFT, node, hindLeftExpectedSnappedNodeTransform);
         PawNodeTools.getNodeTransformToWorld(RobotQuadrant.HIND_RIGHT, node, hindRightExpectedSnappedNodeTransform);
         frontLeftSnapTransform.transform(frontLeftExpectedSnappedNodeTransform);
         frontRightSnapTransform.transform(frontRightExpectedSnappedNodeTransform);
         hindLeftSnapTransform.transform(hindLeftExpectedSnappedNodeTransform);
         hindRightSnapTransform.transform(hindRightExpectedSnappedNodeTransform);

         EuclidCoreTestTools.assertEquals(frontLeftExpectedSnappedNodeTransform, frontLeftSnappedNodeTransform, epsilon);
         EuclidCoreTestTools.assertEquals(frontRightExpectedSnappedNodeTransform, frontRightSnappedNodeTransform, epsilon);
         EuclidCoreTestTools.assertEquals(hindLeftExpectedSnappedNodeTransform, hindLeftSnappedNodeTransform, epsilon);
         EuclidCoreTestTools.assertEquals(hindRightExpectedSnappedNodeTransform, hindRightSnappedNodeTransform, epsilon);
      }
   }
}
