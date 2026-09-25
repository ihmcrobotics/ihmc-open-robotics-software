package us.ihmc.rdx.simulation.environment.object.objects;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * A Mixamo joint read straight off {@code node.globalTransform} is in model space: Y is up and the
 * person's own facing is not applied yet. Gestures are only read correctly once the model offset
 * and the placement pose are composed on top, which is what these numbers protect.
 */
public class RDXPersonSkeletonFrameTest
{
   /** Mixamo metres, model space, from personAnimated.glb at the mid-clip hold. */
   private static final Point3D RIGHT_SHOULDER_IN_MODEL = new Point3D(-0.14817, 1.41046, -0.00403);
   private static final Point3D POINTING_RIGHT_HAND_IN_MODEL = new Point3D(-0.70471, 1.36079, 0.04762);
   private static final Point3D LEFT_SHOULDER_IN_MODEL = new Point3D(0.15454, 1.41025, 0.00198);

   @Test
   public void theModelOffsetTurnsMixamoYUpIntoZUp()
   {
      Point3D world = new Point3D(LEFT_SHOULDER_IN_MODEL);
      modelOffset().transform(world);

      assertEquals(0.00198, world.getX(), 1.0e-5, "Mixamo +Z forward becomes world +X");
      assertEquals(0.15454, world.getY(), 1.0e-5, "Mixamo +X becomes world +Y");
      assertEquals(1.41025, world.getZ(), 1.0e-5, "Mixamo +Y up becomes world +Z");
   }

   @Test
   public void aPersonFacingTheRobotHasTheirLeftOnTheRobotsRight()
   {
      Point3D leftShoulder = worldPointOf(LEFT_SHOULDER_IN_MODEL);
      Point3D rightShoulder = worldPointOf(RIGHT_SHOULDER_IN_MODEL);

      assertTrue(leftShoulder.getY() < 0.0,
                 "they face the robot, so their left shoulder is on the robot's right: " + leftShoulder);
      assertTrue(rightShoulder.getY() > 0.0,
                 "and their right shoulder is on the robot's left: " + rightShoulder);
   }

   @Test
   public void theWorldJointsMatchTheGestureTestFixture()
   {
      Point3D shoulder = worldPointOf(RIGHT_SHOULDER_IN_MODEL);
      Point3D wrist = worldPointOf(POINTING_RIGHT_HAND_IN_MODEL);

      // MixamoHoldPoses.pointRight() in alex-commands carries exactly these world joints.
      assertEquals(2.0040, shoulder.getX(), 1.0e-3);
      assertEquals(0.1482, shoulder.getY(), 1.0e-3);
      assertEquals(1.4105, shoulder.getZ(), 1.0e-3);
      assertEquals(1.9524, wrist.getX(), 1.0e-3);
      assertEquals(0.7047, wrist.getY(), 1.0e-3);
      assertEquals(1.3608, wrist.getZ(), 1.0e-3);
   }

   @Test
   public void aRightArmPointAimsAtTheRobotsLeft()
   {
      Point3D shoulder = worldPointOf(RIGHT_SHOULDER_IN_MODEL);
      Point3D wrist = worldPointOf(POINTING_RIGHT_HAND_IN_MODEL);
      double yaw = Math.atan2(wrist.getY() - shoulder.getY(), wrist.getX() - shoulder.getX());

      assertEquals(Math.PI / 2.0, yaw, Math.toRadians(20.0),
                   "the point clip raises their right arm, which reaches across to the robot's left");
   }

   @Test
   public void readingTheJointsInModelSpaceAimsSomewhereElseEntirely()
   {
      double world = Math.atan2(worldPointOf(POINTING_RIGHT_HAND_IN_MODEL).getY() - worldPointOf(RIGHT_SHOULDER_IN_MODEL).getY(),
                                worldPointOf(POINTING_RIGHT_HAND_IN_MODEL).getX() - worldPointOf(RIGHT_SHOULDER_IN_MODEL).getX());
      double modelSpace = Math.atan2(POINTING_RIGHT_HAND_IN_MODEL.getY() - RIGHT_SHOULDER_IN_MODEL.getY(),
                                     POINTING_RIGHT_HAND_IN_MODEL.getX() - RIGHT_SHOULDER_IN_MODEL.getX());

      double error = Math.abs(Math.atan2(Math.sin(world - modelSpace), Math.cos(world - modelSpace)));
      assertTrue(error > Math.toRadians(70.0),
                 "skipping the instance transform is not a small error - it aims outside the picker's cone");
   }

   /** The offset {@code RDXPersonObject} applies to every person. */
   private static RigidBodyTransform modelOffset()
   {
      RigidBodyTransform offset = new RigidBodyTransform();
      offset.appendRollRotation(Math.PI / 2.0);
      offset.prependYawRotation(Math.PI / 2.0);
      return offset;
   }

   /** Person standing at (2, 0) turned to face the robot at the origin, as the sim places them. */
   private static Point3D worldPointOf(Point3D modelPoint)
   {
      RigidBodyTransform placement = new RigidBodyTransform();
      placement.getTranslation().set(2.0, 0.0, 0.0);
      placement.getRotation().setToYawOrientation(Math.atan2(0.0, -2.0));

      Point3D world = new Point3D(modelPoint);
      modelOffset().transform(world);
      placement.transform(world);
      return world;
   }
}
