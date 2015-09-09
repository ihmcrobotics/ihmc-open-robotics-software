package us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.Test;

import us.ihmc.communication.packets.walking.PelvisPosePacket;
import us.ihmc.tools.random.RandomTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/13/13
 * Time: 7:29 PM
 * To change this template use File | Settings | File Templates.
 */
public class PelvisPosePacketTransformerTest
{

	@DeployableTestMethod(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPelvisPosePacketTransformer()
   {
      int numberOfTests = 10;
      Random random = new Random(100L);
      RigidBodyTransform transform3D;
      for (int i = 0; i < numberOfTests; i++)
      {
         AxisAngle4d axisAngle = RandomTools.generateRandomRotation(random);
         Quat4d quat = new Quat4d();
         quat.set(axisAngle);

         PelvisPosePacket starting = new PelvisPosePacket(quat);

         transform3D = RigidBodyTransform.generateRandomTransform(random);

         PelvisPosePacket ending = starting.transform(transform3D);

         performEqualsTestForQuat(starting, transform3D, ending);
      }

      for (int i = 0; i < numberOfTests; i++)
      {
         Point3d point3d = RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0);

         PelvisPosePacket starting = new PelvisPosePacket(point3d);
         transform3D = RigidBodyTransform.generateRandomTransform(random);
         PelvisPosePacket ending = starting.transform(transform3D);
         performEqualsTestForPoint(starting, transform3D, ending);
      }
   }

   private static void performEqualsTestForQuat(PelvisPosePacket starting, RigidBodyTransform transform3D, PelvisPosePacket ending)
   {
//    public Quat4d quaternion;
      Quat4d startQuat = starting.getOrientation();
      Quat4d endQuat = ending.getOrientation();
      assertTrue(areOrientationsEqualWithTransform(startQuat, transform3D, endQuat));

//    public Point3d point;
      assertTrue(starting.getPosition() == null);
      assertTrue(ending.getPosition() == null);

   }

   private static void performEqualsTestForPoint(PelvisPosePacket starting, RigidBodyTransform transform3D, PelvisPosePacket ending)
   {
//    public Quat4d quaternion;
      Point3d startPoint = starting.getPosition();
      Point3d endPoint = ending.getPosition();
      double distance = getDistanceBetweenPoints(startPoint, transform3D, endPoint);
      assertEquals("not equal", 0.0, distance, 1e-6);

//    public Point3d point;
      assertTrue(starting.getOrientation() == null);
      assertTrue(ending.getOrientation() == null);
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
