package us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.Test;

import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandPosePacket;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/13/13
 * Time: 7:42 PM
 * To change this template use File | Settings | File Templates.
 */
public class HandPosePacketTransformerTest
{
   private final static int numberOfArmJoints = 6;

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTransformHandPosePacket()
   {
      int numberOfTests = 10;
      Random random = new Random(100L);
      RigidBodyTransform transform3D;
      HandPosePacket.Frame frame;
      RobotSide robotSide;
      HandPosePacket.DataType dataType;
      for (int i = 0; i < numberOfTests; i++)
      {
         AxisAngle4d axisAngle = RandomTools.generateRandomRotation(random);
         Quat4d quat = new Quat4d();
         quat.set(axisAngle);

         if (i % 2 == 0)
            frame = HandPosePacket.Frame.CHEST;
         else
            frame = HandPosePacket.Frame.WORLD;

         if (i % 2 == 0)
            robotSide = RobotSide.LEFT;
         else
            robotSide = RobotSide.RIGHT;
         
         if (i % 2 == 0)
            dataType = HandPosePacket.DataType.HAND_POSE;
         else
            dataType = HandPosePacket.DataType.JOINT_ANGLES;

         Point3d point3d = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
         double trajectoryTime = RandomTools.generateRandomDouble(random, 1.0, 10.0);

         double[] randomJointAngles = new double[numberOfArmJoints];
         randomJointAngles = RandomTools.generateRandomDoubleArray(random, numberOfArmJoints, -2 * Math.PI, 2 * Math.PI);

         HandPosePacket starting = new HandPosePacket(robotSide, frame, dataType, point3d, quat, i % 2 == 1, trajectoryTime, randomJointAngles);

         transform3D = RigidBodyTransform.generateRandomTransform(random);

         HandPosePacket ending = starting.transform(transform3D);

         performEqualsTest(starting, transform3D, ending);
      }
   }

   private static void performEqualsTest(HandPosePacket starting, RigidBodyTransform transform3D, HandPosePacket ending)
   {
      // RobotSide robotSide;
      assertTrue(starting.getRobotSide().equals(ending.getRobotSide()));

      // boolean toHomePosition;
      assertTrue(starting.isToHomePosition() == ending.isToHomePosition());

      if (!ending.isToHomePosition())
      {
         // Frame referenceFrame;
         assertTrue(starting.getReferenceFrame().equals(ending.getReferenceFrame()));



         // Point3d position;
         double distance = getDistanceBetweenPoints(starting.getPosition(), transform3D, ending.getPosition());
         assertEquals("not equal", 0.0, distance, 1e-6);

         // Quat4d orientation;
         Quat4d startQuat = starting.getOrientation();
         Quat4d endQuat = ending.getOrientation();
         assertTrue(areOrientationsEqualWithTransform(startQuat, transform3D, endQuat));

      }
   }

   private static double getDistanceBetweenPoints(Point3d startingPoint, RigidBodyTransform transform3D, Point3d endPoint)
   {
      ReferenceFrame ending = ReferenceFrame.constructARootFrame("ending", false, true, true);
      ReferenceFrame starting = ReferenceFrame.constructFrameWithUnchangingTransformToParent("starting", ending, transform3D, false, true, true);

      FramePoint start = new FramePoint(starting, startingPoint);
      FramePoint end = new FramePoint(ending, endPoint);

      end.changeFrame(starting);

      return end.distance(start);
   }

   private static boolean areOrientationsEqualWithTransform(Quat4d orientationStart, RigidBodyTransform transform3D, Quat4d orientationEnd)
   {
      ReferenceFrame ending = ReferenceFrame.constructARootFrame("ending", false, true, true);
      ReferenceFrame starting = ReferenceFrame.constructFrameWithUnchangingTransformToParent("starting", ending, transform3D, false, true, true);

      FrameOrientation start = new FrameOrientation(starting, orientationStart);
      FrameOrientation end = new FrameOrientation(ending, orientationEnd);

      end.changeFrame(starting);

      return equalsFrameOrientation(start, end);
   }

   private static boolean equalsFrameOrientation(FrameOrientation frameOrientation1, FrameOrientation frameOrientation2)
   {
      // Check reference frame first
      if (frameOrientation1.getReferenceFrame() != frameOrientation2.getReferenceFrame())
         return false;

      double[] rpyThis = frameOrientation1.getYawPitchRoll();
      double[] rpyThat = frameOrientation2.getYawPitchRoll();

      for (int i = 0; i < rpyThat.length; i++)
      {
         if (Math.abs(rpyThis[i] - rpyThat[i]) > 1e-6)
            return false;
      }

      return true;
   }
}
