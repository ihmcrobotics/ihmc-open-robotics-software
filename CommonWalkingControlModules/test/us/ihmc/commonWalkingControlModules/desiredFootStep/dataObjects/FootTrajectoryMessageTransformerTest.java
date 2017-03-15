package us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/20/13
 * Time: 12:23 PM
 * To change this template use File | Settings | File Templates.
 */
public class FootTrajectoryMessageTransformerTest
{

   private static void performEqualsTest(FootTrajectoryMessage starting, RigidBodyTransform transform3D, FootTrajectoryMessage ending)
   {
      // RobotSide robotSide;
      assertTrue(starting.getRobotSide().equals(ending.getRobotSide()));
      assertTrue(starting.getNumberOfTrajectoryPoints() == ending.getNumberOfTrajectoryPoints());

      for (int i = 0; i < starting.getNumberOfTrajectoryPoints(); i++)
      {
         SE3TrajectoryPointMessage startingWaypoint = starting.getTrajectoryPoint(i);
         SE3TrajectoryPointMessage endingWaypoint = ending.getTrajectoryPoint(i);

         // Point3D position;
         double distance = getDistanceBetweenPoints(startingWaypoint.position, transform3D, endingWaypoint.position);
         assertEquals("not equal", 0.0, distance, 1e-6);

         // Quat4d orientation;
         Quaternion startQuat = startingWaypoint.orientation;
         Quaternion endQuat = endingWaypoint.orientation;
         assertTrue(areOrientationsEqualWithTransform(startQuat, transform3D, endQuat));
      }
   }

   private static double getDistanceBetweenPoints(Point3D startingPoint, RigidBodyTransform transform3D, Point3D endPoint)
   {
      ReferenceFrame ending = ReferenceFrame.constructARootFrame("ending", false, true, true);
      ReferenceFrame starting = ReferenceFrame.constructFrameWithUnchangingTransformToParent("starting", ending, transform3D, false, true, true);

      FramePoint start = new FramePoint(starting, startingPoint);
      FramePoint end = new FramePoint(ending, endPoint);

      end.changeFrame(starting);

      return end.distance(start);
   }

   private static boolean areOrientationsEqualWithTransform(Quaternion orientationStart, RigidBodyTransform transform3D, Quaternion orientationEnd)
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
