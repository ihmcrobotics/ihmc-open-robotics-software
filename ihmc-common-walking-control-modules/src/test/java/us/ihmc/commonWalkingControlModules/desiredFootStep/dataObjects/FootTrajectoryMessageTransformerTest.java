package us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.SE3TrajectoryPointMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;

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
      assertTrue(starting.getSe3Trajectory().getNumberOfTrajectoryPoints() == ending.getSe3Trajectory().getNumberOfTrajectoryPoints());

      for (int i = 0; i < starting.getSe3Trajectory().getNumberOfTrajectoryPoints(); i++)
      {
         SE3TrajectoryPointMessage startingWaypoint = starting.getSe3Trajectory().getTrajectoryPoint(i);
         SE3TrajectoryPointMessage endingWaypoint = ending.getSe3Trajectory().getTrajectoryPoint(i);

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
      ReferenceFrame ending = ReferenceFrame.constructARootFrame("ending");
      ReferenceFrame starting = ReferenceFrame.constructFrameWithUnchangingTransformToParent("starting", ending, transform3D);

      FramePoint3D start = new FramePoint3D(starting, startingPoint);
      FramePoint3D end = new FramePoint3D(ending, endPoint);

      end.changeFrame(starting);

      return end.distance(start);
   }

   private static boolean areOrientationsEqualWithTransform(Quaternion orientationStart, RigidBodyTransform transform3D, Quaternion orientationEnd)
   {
      ReferenceFrame ending = ReferenceFrame.constructARootFrame("ending");
      ReferenceFrame starting = ReferenceFrame.constructFrameWithUnchangingTransformToParent("starting", ending, transform3D);

      FrameQuaternion start = new FrameQuaternion(starting, orientationStart);
      FrameQuaternion end = new FrameQuaternion(ending, orientationEnd);

      end.changeFrame(starting);

      return equalsFrameOrientation(start, end);
   }

   private static boolean equalsFrameOrientation(FrameQuaternion frameOrientation1, FrameQuaternion frameOrientation2)
   {
      // Check reference frame first
      if (frameOrientation1.getReferenceFrame() != frameOrientation2.getReferenceFrame())
         return false;

      double[] rpyThis = new double[3];
      frameOrientation1.getYawPitchRoll(rpyThis);
      double[] rpyThat = new double[3];
      frameOrientation2.getYawPitchRoll(rpyThat);

      for (int i = 0; i < rpyThat.length; i++)
      {
         if (Math.abs(rpyThis[i] - rpyThat[i]) > 1e-6)
            return false;
      }

      return true;
   }
}
