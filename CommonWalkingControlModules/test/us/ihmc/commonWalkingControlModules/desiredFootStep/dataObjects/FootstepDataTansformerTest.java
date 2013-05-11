package us.ihmc.commonWalkingControlModules.desiredFootStep.dataObjects;

import com.yobotics.simulationconstructionset.util.trajectory.TrajectoryWaypointGenerationMethod;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.desiredFootStep.Footstep;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Created with IntelliJ IDEA.
 * User: pneuhaus
 * Date: 5/11/13
 * Time: 3:55 PM
 * To change this template use File | Settings | File Templates.
 */
public class FootstepDataTansformerTest
{
   @Test
   public void test()
   {
      Transform3D transform3D = new Transform3D();

      FootstepData originalFootstepData = getTestFootstepData();



   }


   private static FootstepData getTestFootstepData()
   {
      Random random = new Random(100L);

      FootstepData ret = new FootstepData();
      ret.id = "name";
      ret.rigidBodyName = "rigidBodyNameHere";
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
      ret.expectedContactPoints = new ArrayList<Point3d>(listOfPoints);

      ret.trustHeight = false;

      listOfPoints = new ArrayList<Point3d>();
      {
         for (int i = 0; i < 30; i++)
         {
            listOfPoints.add(RandomTools.generateRandomPoint(random, 10.0, 10.0, 10.0));
         }
      }

      ret.trajectoryWaypoints = new ArrayList<Point3d>(listOfPoints);

      ret.trajectoryWaypointGroundClearance = random.nextDouble();

      //Ignore this for now
      ret.trajectoryBoxData = new FlatHorizontalBoxData();

      int index = (int) Math.floor(random.nextDouble() * TrajectoryWaypointGenerationMethod.values().length);
      ret.trajectoryWaypointGenerationMethod = TrajectoryWaypointGenerationMethod.values()[index];

      return ret;
   }

   private static void performEqualsTestsWithTransform(FootstepData footstepData, Transform3D transform3D, FootstepData transformedFootstepData)
   {
      double distance;
      List<Point3d> originalList;
      List<Point3d> transformedList;

      //public String id;
      assertTrue(footstepData.getId().equals(transformedFootstepData.getId()));

      //public String rigidBodyName;
      assertTrue(footstepData.getRigidBodyName().equals(transformedFootstepData.getRigidBodyName()));

      //public Point3d location;
      distance = getDistanceBetweenPoints(footstepData.getLocation(), transform3D, transformedFootstepData.getLocation());
      assertEquals("not equal", 0.0, distance, 1e-6);

      //public Quat4d orientation;


      //public List<Point3d> expectedContactPoints;
      originalList = footstepData.getExpectedContactPoints();
      transformedList = transformedFootstepData.getExpectedContactPoints();
      assertEquals("", originalList.size(), transformedList.size());
      for (int i = 0; i <originalList.size() ; i++)
      {
         distance = getDistanceBetweenPoints(originalList.get(i), transform3D, transformedList.get(i));
         assertEquals("not equal", 0.0, distance, 1e-6);
      }


      //public boolean trustHeight;
      assertEquals("", footstepData.getTrustHeight(), transformedFootstepData.getTrustHeight());

      //public List<Point3d> trajectoryWaypoints;
      originalList = footstepData.getTrajectoryWaypoints();
      transformedList = transformedFootstepData.getTrajectoryWaypoints();
      assertEquals("", originalList.size(), transformedList.size());
      for (int i = 0; i <originalList.size() ; i++)
      {
         distance = getDistanceBetweenPoints(originalList.get(i), transform3D, transformedList.get(i));
         assertEquals("not equal", 0.0, distance, 1e-6);
      }

      //public double trajectoryWaypointGroundClearance;
      assertEquals("", footstepData.getTrajectoryWaypointGroundClearance(), transformedFootstepData.getTrajectoryWaypointGroundClearance(), 1e-6);

      //public FlatHorizontalBoxData trajectoryBoxData;

      //public TrajectoryWaypointGenerationMethod trajectoryWaypointGenerationMethod;
      assertTrue("", footstepData.getTrajectoryWaypointGenerationMethod().equals(transformedFootstepData.getTrajectoryWaypointGenerationMethod()));
   }


   @Test
   public void testDistance()
   {
      Point3d startPoint = new Point3d(2.0, 6.0, 5.0);
      Transform3D transform3D = new Transform3D();
      transform3D.set(new Vector3d(1.0, 2.0, 3.0));

      Point3d endPoint = new Point3d();
      transform3D.transform(startPoint, endPoint);

      double distance = getDistanceBetweenPoints(startPoint, transform3D, endPoint);
      assertEquals("not equal", 0.0, distance, 1e-6);
   }

   private static double getDistanceBetweenPoints(Point3d startingPoint, Transform3D transform3D, Point3d endPoint)
   {
      ReferenceFrame ending = ReferenceFrame.constructARootFrame("ending", false, true, true);
      ReferenceFrame starting = ReferenceFrame.constructFrameWithUnchangingTransformToParent("starting", ending, transform3D, false, true, true);

      FramePoint start = new FramePoint(starting, startingPoint);
      FramePoint end = new FramePoint(ending, endPoint);

      end.changeFrame(starting);

      return end.distance(start);

   }
}
