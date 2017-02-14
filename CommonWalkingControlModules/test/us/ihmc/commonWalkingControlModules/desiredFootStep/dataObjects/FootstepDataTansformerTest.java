package us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/11/13
 * Time: 3:55 PM
 * To change this template use File | Settings | File Templates.
 */
public class FootstepDataTansformerTest
{
   private static Random random = new Random(100L);

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test()
   {
      RigidBodyTransform transform3D;
      FootstepDataMessage originalFootstepData;
      FootstepDataMessage transformedFootstepData;

      int numberOfTests = 1;

      for (int i = 0; i < numberOfTests; i++)
      {
         originalFootstepData = getTestFootstepData();
         transform3D = new RigidBodyTransform();
         transform3D = RigidBodyTransform.generateRandomTransform(random);

         transformedFootstepData = originalFootstepData.transform(transform3D);

         performEqualsTestsWithTransform(originalFootstepData, transform3D, transformedFootstepData);
      }
   }

   private static FootstepDataMessage getTestFootstepData()
   {
      FootstepDataMessage ret = new FootstepDataMessage();
      ret.robotSide = RobotSide.LEFT;
      ret.location = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);
      AxisAngle4d axisAngle = RandomTools.generateRandomRotation(random);
      ret.orientation = new Quat4d();
      ret.orientation.set(axisAngle);

      List<Point3d> listOfPoints = new ArrayList<Point3d>();
      {
         for (int i = 0; i < 30; i++)
         {
            listOfPoints.add(RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0));
         }
      }

      return ret;
   }

   private static void performEqualsTestsWithTransform(FootstepDataMessage footstepData, RigidBodyTransform transform3D,
         FootstepDataMessage transformedFootstepData)
   {
      double distance;

      // public String rigidBodyName;
      assertTrue(footstepData.getRobotSide() == transformedFootstepData.getRobotSide());

      // public Point3d location;
      distance = getDistanceBetweenPoints(footstepData.getLocation(), transform3D, transformedFootstepData.getLocation());
      assertEquals("not equal", 0.0, distance, 1e-6);

      // public Quat4d orientation;
      Quat4d startQuat = footstepData.getOrientation();
      Quat4d endQuat = transformedFootstepData.getOrientation();
      assertTrue(areOrientationsEqualWithTransform(startQuat, transform3D, endQuat));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistance()
   {
      Point3d startPoint = new Point3d(2.0, 6.0, 5.0);
      RigidBodyTransform transform3D = new RigidBodyTransform();
      transform3D.setTranslationAndIdentityRotation(new Vector3d(1.0, 2.0, 3.0));

      Point3d endPoint = new Point3d();
      transform3D.transform(startPoint, endPoint);

      double distance = getDistanceBetweenPoints(startPoint, transform3D, endPoint);
      assertEquals("not equal", 0.0, distance, 1e-6);
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

   private static double getDistanceBetweenPoints(Point3d startingPoint, RigidBodyTransform transform3D, Point3d endPoint)
   {
      ReferenceFrame ending = ReferenceFrame.constructARootFrame("ending", false, true, true);
      ReferenceFrame starting = ReferenceFrame.constructFrameWithUnchangingTransformToParent("starting", ending, transform3D, false, true, true);

      FramePoint start = new FramePoint(starting, startingPoint);
      FramePoint end = new FramePoint(ending, endPoint);

      end.changeFrame(starting);

      return end.distance(start);
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
